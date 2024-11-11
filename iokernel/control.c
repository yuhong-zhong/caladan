/*
 * control.c - the control-plane for the I/O kernel
 */

#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/eventfd.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdio.h>
#include <fcntl.h>

#include <base/fd_transfer.h>
#include <base/stddef.h>
#include <base/mem.h>
#include <base/log.h>
#include <base/thread.h>
#include <iokernel/control.h>
#include <iokernel/directpath.h>

#include "hw_timestamp.h"
#include "defs.h"
#include "sched.h"

#define EPOLL_CONTROLFD_COOKIE 0
#define EPOLL_EFD_COOKIE 1
#define CTL_SOCK_BACKLOG 4096

static int controlfd;
static int epoll_fd;
static unsigned int nr_clients;
struct lrpc_params lrpc_control_to_data_params;
struct lrpc_params lrpc_data_to_control_params;
int data_to_control_efd;
static struct lrpc_chan_out lrpc_control_to_data;
static struct lrpc_chan_in lrpc_data_to_control;

struct iokernel_info *iok_info;

static int epoll_ctl_add(int fd, void *arg)
{
	struct epoll_event ev;

	ev.events = EPOLLIN | EPOLLERR;
	ev.data.fd = fd;
	ev.data.ptr = arg;
	if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &ev) < 0) {
		perror("epoll_ctl: EPOLL_CTL_ADD");
		return -errno;
	}

	return 0;
}


static int epoll_ctl_del(int fd)
{
	if (epoll_ctl(epoll_fd, EPOLL_CTL_DEL, fd, NULL) < 0) {
		perror("epoll_ctl: EPOLL_CTL_DEL");
		return -errno;
	}

	return 0;
}

static void *copy_shm_data(struct shm_region *r, shmptr_t ptr, size_t len)
{
	void *in, *out;

	in = shmptr_to_ptr(r, ptr, len);
	if (!in)
		return NULL;

	out = malloc(len);
	if (!out)
		return NULL;

	memcpy(out, in, len);

	return out;
}

static int control_setup_directpath(struct proc *p, int controlfd)
{
	int memfd, barfd, ret;
	size_t specsz;
	ssize_t wret;
	struct directpath_queue_spec *qspec;
	struct directpath_spec *spec;

	specsz = sizeof(*spec) + sizeof(*qspec) * p->thread_count;
	spec = malloc(specsz);
	if (!spec)
		return -ENOMEM;

	ret = alloc_directpath_ctx(p, p->vfio_directpath_rmp, spec, &memfd, &barfd);
	if (ret) {
		log_err("control: failed to allocate context");
		goto err;
	}

	ret = send_fd(controlfd, memfd);
	if (ret)
		goto err;

	ret = send_fd(controlfd, barfd);
	if (ret)
		goto err;

	wret = write(controlfd, spec, specsz);
	if (wret != specsz) {
		ret = -1;
		goto err;
	}

	free(spec);
	return 0;

err:
	release_directpath_ctx(p);
	free(spec);
	return ret;
}

static int control_init_hwq(struct shm_region *r,
	  struct hardware_queue_spec *hs, struct hwq *h)
{
	if (hs->hwq_type == HWQ_INVALID) {
		h->enabled = false;
		h->busy_since = UINT64_MAX;
		return 0;
	}

	h->descriptor_table = shmptr_to_ptr(r, hs->descriptor_table, (1 << hs->descriptor_log_size) * hs->nr_descriptors);
	h->consumer_idx = shmptr_to_ptr(r, hs->consumer_idx, sizeof(*h->consumer_idx));
	h->descriptor_log_size = hs->descriptor_log_size;
	h->nr_descriptors = hs->nr_descriptors;
	h->parity_byte_offset = hs->parity_byte_offset;
	h->parity_bit_mask = hs->parity_bit_mask;
	h->hwq_type = hs->hwq_type;
	h->enabled = true;

	if (!h->descriptor_table || !h->consumer_idx)
		return -EINVAL;

	if (!is_power_of_two(h->nr_descriptors))
		return -EINVAL;

	if (h->parity_byte_offset > (1 << h->descriptor_log_size))
		return -EINVAL;

	h->busy_since = UINT64_MAX;
	h->last_head = 0;
	h->last_tail = 0;

	return 0;
}

static struct proc *control_create_proc(void *shbuf, size_t len,
		 pid_t pid)
{
	struct control_hdr hdr;
	struct shm_region reg = {NULL};
	size_t nr_pages;
	struct proc *p = NULL;
	struct thread_spec *threads = NULL;
	int i, ret;

	/* attach the shared memory region */
	if (len < sizeof(hdr))
		goto fail;

	reg.base = shbuf;
	reg.len = len;

	/* parse the control header */
#ifdef NO_CACHE_COHERENCE
	batch_clflushopt(shbuf, sizeof(hdr));
#endif
	memcpy(&hdr, (struct control_hdr *)shbuf, sizeof(hdr)); /* TOCTOU */
	if (hdr.magic != CONTROL_HDR_MAGIC ||
		  hdr.version_no != CONTROL_HDR_VERSION) {
		log_err("bad control header: please make sure IOKernel and application are compiled from the same source");
		goto fail;
	}

	if (hdr.thread_count > NCPU || hdr.thread_count == 0)
		goto fail;

	/* copy arrays of threads, timers, and hwq specs */
#ifdef NO_CACHE_COHERENCE
	batch_clflushopt(shbuf + hdr.thread_specs, sizeof(*threads) * hdr.thread_count);
#endif
	threads = copy_shm_data(&reg, hdr.thread_specs, hdr.thread_count * sizeof(*threads));
	if (!threads)
		goto fail;

	/* create the process */
	nr_pages = div_up(len, PGSIZE_2MB);
	p = malloc(sizeof(*p) + nr_pages * sizeof(physaddr_t));
	if (!p)
		goto fail;
	memset(p, 0, sizeof(*p));

	p->pid = pid;
	ref_init(&p->ref);
	p->region = reg;
	p->removed = false;
	memcpy(&p->sched_cfg, &hdr.sched_cfg, sizeof(p->sched_cfg));
	p->thread_count = hdr.thread_count;
	if (!hdr.ip_addr)
		goto fail;
	p->ip_addr = hdr.ip_addr;
	p->runtime_info = shmptr_to_ptr(&reg, hdr.runtime_info,
					   sizeof(*p->runtime_info));
	if (!p->runtime_info)
		goto fail;
#ifdef NO_CACHE_COHERENCE
	batch_clflushopt(p->runtime_info, sizeof(*p->runtime_info));
#endif
	memset(&p->runtime_info->congestion, 0, sizeof(p->runtime_info->congestion));
	if (hdr.request_directpath_queues != DIRECTPATH_REQUEST_NONE) {
		p->has_vfio_directpath = p->has_directpath = true;
		if (hdr.request_directpath_queues == DIRECTPATH_REQUEST_STRIDED_RMP)
			p->vfio_directpath_rmp = true;
	}

	/* initialize the threads */
	for (i = 0; i < hdr.thread_count; i++) {
		struct thread *th = &p->threads[i];
		struct thread_spec *s = &threads[i];

		/* attach the RX queue */
		ret = shm_init_lrpc_out(&reg, &s->rxq, &th->rxq);
		if (ret)
			goto fail;

		/* attach the TX packet queue */
		ret = shm_init_lrpc_in(&reg, &s->txpktq, &th->txpktq);
		if (ret)
			goto fail;

		/* attach the TX command queue */
		ret = shm_init_lrpc_in(&reg, &s->txcmdq, &th->txcmdq);
		if (ret)
			goto fail;

		th->tid = s->tid;
		th->p = p;
		th->at_idx = UINT16_MAX;
		th->ts_idx = UINT16_MAX;

		/* initialize pointer to queue pointers in shared memory */
		th->q_ptrs = (struct q_ptrs *) shmptr_to_ptr(&reg, s->q_ptrs,
				sizeof(struct q_ptrs));
		if (!th->q_ptrs)
			goto fail;

		ret = control_init_hwq(&reg, &s->direct_rxq, &th->directpath_hwq);
		if (ret)
			goto fail;

		ret = control_init_hwq(&reg, &s->storage_hwq, &th->storage_hwq);
		if (ret)
			goto fail;

		p->has_directpath |= th->directpath_hwq.enabled;
		p->has_storage |= th->storage_hwq.enabled;
	}

	if (cfg.azure_arp_mode && !p->has_directpath) {
		log_err("control: runtimes must use directpath when running on Azure");
		goto fail;
	}

	/* initialize the table of physical page addresses */
	if (!cfg.vfio_directpath) {
		touch_mapping(p->region.base, p->region.len, hdr.shared_reg_page_size);
		ret = mem_lookup_page_phys_addrs(p->region.base, p->region.len, hdr.shared_reg_page_size,
				p->page_paddrs);
		if (ret) {
			log_err("control: failed to lookup phys addrs");
			goto fail;
		}

		p->max_overflows = hdr.egress_buf_count;
		p->nr_overflows = 0;
		p->overflow_queue = malloc(sizeof(unsigned long) * p->max_overflows);
		if (p->overflow_queue == NULL)
			goto fail;
	}

	/* free temporary allocations */
	free(threads);

	return p;

fail:
	if (p)
		free(p->overflow_queue);
	free(threads);
	free(p);
	if (reg.base)
		// munmap(reg.base, reg.len);
		cxl_free_client(reg.base - PGSIZE_2MB);
	kill(pid, SIGINT);
	log_err("control: couldn't attach pid %d", pid);
	return NULL;
}

static void control_destroy_proc(struct proc *p)
{
	if (p->has_vfio_directpath)
		release_directpath_ctx(p);

	nr_clients--;
	// munmap(p->region.base, p->region.len);
	cxl_free_client(p->region.base - PGSIZE_2MB);
	free(p->overflow_queue);
	free(p);
}

static void control_add_client(void)
{
	struct proc *p;
	struct ucred ucred;
	socklen_t len;
	uint64_t client_cxl_offset = 0, client_cxl_len;
	void *client_shm_buf = NULL;
	uint64_t client_status_code;
	ssize_t ret;
	int fd;
	// int mem_fd;

	fd = accept(controlfd, NULL, NULL);
	if (fd == -1) {
		log_err("control: accept() failed [%s]", strerror(errno));
		return;
	}

	if (nr_clients >= IOKERNEL_MAX_PROC) {
		log_err("control: hit client process limit");
		goto fail;
	}

	len = sizeof(struct ucred);
	if (getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &ucred, &len) == -1) {
		log_err("control: getsockopt() failed [%s]", strerror(errno));
		goto fail;
	}

	client_shm_buf = cxl_alloc_client(&client_cxl_offset);
	RT_BUG_ON(!client_shm_buf);

	*((struct iokernel_info *) client_shm_buf) = *iok_info;
#ifdef NO_CACHE_COHERENCE
	batch_clwb(client_shm_buf, sizeof(struct iokernel_info));
#endif

	ret = write(fd, &client_cxl_offset, sizeof(client_cxl_offset));
	if (ret != sizeof(client_cxl_offset)) {
		log_err("control_add_client: write(client_cxl_offset) failed [%s]", strerror(errno));
		RT_BUG_ON(true);
	}

	client_cxl_len = CXL_CLIENT_SIZE;
	ret = write(fd, &client_cxl_len, sizeof(client_cxl_len));
	if (ret != sizeof(client_cxl_len)) {
		log_err("control_add_client: write(client_cxl_len) failed [%s]", strerror(errno));
		RT_BUG_ON(true);
	}

	log_info("control_add_client: client_cxl_offset: 0x%lx, client_cxl_len: 0x%lx", client_cxl_offset, client_cxl_len);
	log_info("control_add_client: waiting for client to register to iokernel");

	ret = read(fd, &client_status_code, sizeof(client_status_code));
	if (ret != sizeof(client_status_code)) {
		log_err("control_add_client: read(client_status_code) failed, len=%ld [%s]",
			ret, strerror(errno));
		RT_BUG_ON(true);
	}
	RT_BUG_ON(client_status_code != 0);

	log_info("control_add_client: client registered to iokernel");

	// ret = recv_fd(fd, &mem_fd);
	// if (ret) {
	// 	log_err("control: recv_fd() failed [%s]", strerror(errno));
	// 	goto fail;
	// }

	// ret = read(fd, &shm_len, sizeof(shm_len));
	// if (ret != sizeof(shm_len)) {
	// 	log_err("control: read() failed, len=%ld [%s]",
	// 		ret, strerror(errno));
	// 	goto fail;
	// }

	p = control_create_proc(client_shm_buf + PGSIZE_2MB, client_cxl_len - PGSIZE_2MB, ucred.pid);
	if (!p) {
		log_err("control: failed to create process '%d'", ucred.pid);
		goto fail;
	}

	if (p->has_vfio_directpath) {
		ret = control_setup_directpath(p, fd);
		if (ret) {
			log_err("control: failed to setup directpath queues");
			goto fail_destroy_proc;
		}
	}

	ret = epoll_ctl_add(fd, p);
	if (ret) {
		log_err("control: failed to add proc to epoll set");
		goto fail_destroy_proc;
	}

	if (!lrpc_send(&lrpc_control_to_data, DATAPLANE_ADD_CLIENT,
			(unsigned long) p)) {
		log_err("control: failed to inform dataplane of new client '%d'",
				ucred.pid);
		goto fail_efd;
	}

	nr_clients++;
	p->control_fd = fd;
	return;

fail_efd:
	epoll_ctl_del(fd);
fail_destroy_proc:
	control_destroy_proc(p);
fail:
	close(fd);
}

static void control_instruct_dataplane_to_remove_client(struct proc *p)
{
	p->removed = true;

	if (!lrpc_send(&lrpc_control_to_data, DATAPLANE_REMOVE_CLIENT,
			(unsigned long)p)) {
		log_err("control: failed to inform dataplane of removed client");
	} else {
		/* remove fd from set once we have notified dataplane */
		epoll_ctl_del(p->control_fd);
		close(p->control_fd);
	}
}

static void control_remove_client(struct proc *p)
{
	/* client failed to attach to scheduler, notify with signal */
	if (p->attach_fail)
		kill(p->pid, SIGINT);

	if (!p->removed) {
		epoll_ctl_del(p->control_fd);
		close(p->control_fd);
	}

	control_destroy_proc(p);
}

static void control_loop(void)
{
	int ret;
	uint64_t cmd, efdval;
	unsigned long payload;
	struct proc *p;
	struct epoll_event ev;

	pthread_barrier_wait(&init_barrier);

	while (1) {
		ret = epoll_wait(epoll_fd, &ev, 1, -1);
		while (ret == -1 && errno == EINTR)
			ret = epoll_wait(epoll_fd, &ev, 1, -1);

		if (ret != 1) {
			log_err("control: epoll_wait got %d (errno %d)", ret, errno);
			exit(1);
		}

		if (ev.data.u64 == EPOLL_EFD_COOKIE) {
			/* do nothing */
		} else if (ev.data.fd == EPOLL_CONTROLFD_COOKIE) {
			/* accept a new connection */
			control_add_client();
		} else {
			p = (struct proc *)ev.data.ptr;
			control_instruct_dataplane_to_remove_client(p);
		}

		do {
			while (lrpc_recv(&lrpc_data_to_control, &cmd, &payload)) {
				p = (struct proc *) payload;
				assert(cmd == CONTROL_PLANE_REMOVE_CLIENT);
				/* it is now safe to remove data structures for this client */
				control_remove_client(p);
			}
		} while (read(data_to_control_efd, &efdval, sizeof(efdval)) == sizeof(efdval));
	}
}

/*
 * Pins thread tid to core. Returns 0 on success and < 0 on error. Note that
 * this function can always fail with error ESRCH, because threads can be
 * killed at any time.
 */
int pin_thread(pid_t tid, int core)
{
	cpu_set_t cpuset;
	int ret;

	CPU_ZERO(&cpuset);
	CPU_SET(core, &cpuset);

	ret = sched_setaffinity(tid, sizeof(cpu_set_t), &cpuset);
	if (ret < 0) {
		log_warn("cores: failed to set affinity for thread %d with err %d",
			 tid, errno);
		return -errno;
	}

	return 0;
}

static void *control_thread(void *data)
{
	int ret;

	/* pin to our assigned core */
	ret = pin_thread(thread_gettid(), sched_ctrl_core);
	if (ret < 0) {
		log_err("control: failed to pin control thread to core %d",
			sched_ctrl_core);
		/* continue running but performance is unpredictable */
	}

	control_loop();
	return NULL;
}

/*
 * Initialize channels for communicating with the I/O kernel dataplane.
 */
static int control_init_dataplane_comm(void)
{
	int ret;
	struct lrpc_msg *buffer_out, *buffer_in;
	uint32_t *wb_out, *wb_in;

	buffer_out = malloc(sizeof(struct lrpc_msg) *
			CONTROL_DATAPLANE_QUEUE_SIZE);
	if (!buffer_out)
		goto fail;
	wb_out = malloc(CACHE_LINE_SIZE);
	if (!wb_out)
		goto fail_free_buffer_out;

	lrpc_control_to_data_params.buffer = buffer_out;
	lrpc_control_to_data_params.wb = wb_out;

	ret = lrpc_init_out(&lrpc_control_to_data,
			lrpc_control_to_data_params.buffer, CONTROL_DATAPLANE_QUEUE_SIZE,
			lrpc_control_to_data_params.wb);
	if (ret < 0) {
		log_err("control: initializing LRPC to dataplane failed");
		goto fail_free_wb_out;
	}

	buffer_in = malloc(sizeof(struct lrpc_msg) * CONTROL_DATAPLANE_QUEUE_SIZE);
	if (!buffer_in)
		goto fail_free_wb_out;
	wb_in = malloc(CACHE_LINE_SIZE);
	if (!wb_in)
		goto fail_free_buffer_in;

	lrpc_data_to_control_params.buffer = buffer_in;
	lrpc_data_to_control_params.wb = wb_in;

	ret = lrpc_init_in(&lrpc_data_to_control,
			lrpc_data_to_control_params.buffer, CONTROL_DATAPLANE_QUEUE_SIZE,
			lrpc_data_to_control_params.wb);
	if (ret < 0) {
		log_err("control: initializing LRPC from dataplane failed");
		goto fail_free_wb_in;
	}

	data_to_control_efd = eventfd(0, EFD_NONBLOCK);
	if (data_to_control_efd < 0)
		return -errno;

	if (epoll_ctl_add(data_to_control_efd, (void *)EPOLL_EFD_COOKIE))
		return -1;

	return 0;

fail_free_wb_in:
	free(wb_in);
fail_free_buffer_in:
	free(buffer_in);
fail_free_wb_out:
	free(wb_out);
fail_free_buffer_out:
	free(buffer_out);
fail:
	return -1;
}

int control_init(void)
{
	struct sockaddr_un addr;
	pthread_t tid;
	int sfd, ret;
	void *shbuf;
	uint64_t shbuf_cxl_offset;

	if (!cfg.vfio_directpath) {
		shbuf = cxl_early_alloc(INGRESS_MBUF_SHM_SIZE, PGSIZE_2MB, &shbuf_cxl_offset);
		RT_BUG_ON(shbuf == NULL);
		// shbuf = mem_map_shm(INGRESS_MBUF_SHM_KEY, NULL, INGRESS_MBUF_SHM_SIZE,
		// 		cfg.no_hugepages ? PGSIZE_4KB : PGSIZE_2MB, true);
		// if (shbuf == MAP_FAILED) {
		// 	log_err("control: failed to map rx buffer area (%s)", strerror(errno));
		// 	if (errno == EEXIST)
		// 		log_err("Shared memory region is already mapped. Please close any "
		// 			    "running iokernels, and be sure to run "
		// 			    "scripts/setup_machine.sh to set proper sysctl parameters.");
		// 	return -1;
		// }
		dp.ingress_mbuf_region.base = shbuf;
		dp.ingress_mbuf_region.len = INGRESS_MBUF_SHM_SIZE;

	}

	shbuf = mem_map_shm(IOKERNEL_INFO_KEY, NULL, IOKERNEL_INFO_SIZE, PGSIZE_4KB, true);
	if (shbuf == MAP_FAILED) {
		log_err("control: failed to map iokernel control header");
		return -1;
	}

	iok_info = (struct iokernel_info *)shbuf;
	memcpy(iok_info->managed_cores, sched_allowed_cores, sizeof(sched_allowed_cores));
	iok_info->rx_cxl_shm_offset = shbuf_cxl_offset;
	iok_info->magic_number = 0xbeef;

	if (nic_pci_addr_str)
		memcpy(&iok_info->directpath_pci, &nic_pci_addr, sizeof(nic_pci_addr));

	addr.sun_family = AF_UNIX;

	// Make sure it's an abstract namespace path.
	assert(CONTROL_SOCK_PATH[0] == '\0');

	BUILD_ASSERT(sizeof(CONTROL_SOCK_PATH) <= sizeof(addr.sun_path));
	memcpy(addr.sun_path, CONTROL_SOCK_PATH, sizeof(CONTROL_SOCK_PATH));

	sfd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (sfd == -1) {
		log_err("control: socket() failed [%s]", strerror(errno));
		return -errno;
	}

	if (bind(sfd, (struct sockaddr *)&addr,
		 sizeof(addr.sun_family) + sizeof(CONTROL_SOCK_PATH)) == -1) {
		log_err("control: bind() failed %i [%s]", errno, strerror(errno));
		close(sfd);
		return -errno;
	}

	if (listen(sfd, CTL_SOCK_BACKLOG) == -1) {
		log_err("control: listen() failed[%s]", strerror(errno));
		close(sfd);
		return -errno;
	}

	epoll_fd = epoll_create1(0);
	if (epoll_fd < 0) {
		log_err("control: failed to create epoll fd");
		return -1;
	}

	if (epoll_ctl_add(sfd, EPOLL_CONTROLFD_COOKIE))
		return -1;

	ret = control_init_dataplane_comm();
	if (ret < 0) {
		log_err("control: cannot initialize communication with dataplane");
		return ret;
	}

	log_info("control: spawning control thread");
	controlfd = sfd;
	if (pthread_create(&tid, NULL, control_thread, NULL) == -1) {
		log_err("control: pthread_create() failed [%s]",
			strerror(errno));
		close(sfd);
		return -errno;
	}

	return 0;
}
