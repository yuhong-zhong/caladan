/*
 * control.h - the control interface for the I/O kernel
 */

#pragma once

#include <sys/types.h>

#include <base/atomic.h>
#include <base/bitmap.h>
#include <base/limits.h>
#include <base/pci.h>
#include <iokernel/shm.h>
#include <net/ethernet.h>
#include <stdio.h>

/*
 * WARNING: If you make any changes that impact the layout of
 * struct control_hdr, please increment the version number!
 */

#define CONTROL_HDR_VERSION 11

/* The abstract namespace path for the control socket. */
// #define CONTROL_SOCK_PATH	"\0/control/iokernel.sock"
#define CONTROL_SOCK_PATH_PREFIX	"\0/control/iokernel.sock"

enum {
	IOK_REGISTER_OK = 0,
	IOK_REGISTER_SECONDARY,
};

/* IOK2IOK communication */

#define IOK2IOK_QUEUE_SIZE	8192UL
#define MAX_NR_IOK2IOK		1UL

// All iok2iok QP head pointers must fit in a 2MB page
BUILD_ASSERT(MAX_NR_IOK2IOK * 6 * sizeof(uint32_t) <= PGSIZE_2MB);
// Each iok2iok QP buffer must fit in a 2MB page
BUILD_ASSERT(IOK2IOK_QUEUE_SIZE * 6 * sizeof(struct lrpc_msg) <= PGSIZE_2MB);

/* primary iokernel */
extern struct lrpc_chan_out iok_as_primary_rxq[MAX_NR_IOK2IOK];
extern struct lrpc_chan_out iok_as_primary_rxcmdq[MAX_NR_IOK2IOK];
extern struct lrpc_chan_in iok_as_primary_txpktq[MAX_NR_IOK2IOK];
extern struct lrpc_chan_in iok_as_primary_txcmdq[MAX_NR_IOK2IOK];

extern struct lrpc_chan_in iok_as_primary_cmdq_in[MAX_NR_IOK2IOK];
extern struct lrpc_chan_out iok_as_primary_cmdq_out[MAX_NR_IOK2IOK];

/* secondary iokernel */
extern struct lrpc_chan_in iok_as_secondary_rxq[MAX_NR_IOK2IOK];
extern struct lrpc_chan_in iok_as_secondary_rxcmdq[MAX_NR_IOK2IOK];
extern struct lrpc_chan_out iok_as_secondary_txpktq[MAX_NR_IOK2IOK];
extern struct lrpc_chan_out iok_as_secondary_txcmdq[MAX_NR_IOK2IOK];

extern struct lrpc_chan_out iok_as_secondary_cmdq_out[MAX_NR_IOK2IOK];
extern struct lrpc_chan_in iok_as_secondary_cmdq_in[MAX_NR_IOK2IOK];

enum {
	IOK2IOK_CMD_ADD_CLIENT = 0,
	IOK2IOK_CMD_CXL_OFFSET,
	IOK2IOK_CMD_CXL_LEN,
	IOK2IOK_CMD_STATUS_CODE,
	IOK2IOK_CMD_LRPC_FD,
	IOK2IOK_CMD_REMOVE_CLIENT,
	IOK2IOK_CMD_NR,
};

#define IOK2IOK_CMD_BITS 4
#define IOK2IOK_GET_RAWCMD(cmd) ((cmd) & ((1 << IOK2IOK_CMD_BITS) - 1))
#define IOK2IOK_GET_IP(cmd) ((cmd) >> IOK2IOK_CMD_BITS)
#define IOK2IOK_MAKE_CMD(cmd, ip) (((ip) << IOK2IOK_CMD_BITS) | (cmd))

// Leave 1b for parity
BUILD_ASSERT(IOK2IOK_CMD_BITS + 4 * 8 + 1 <= sizeof(uint64_t) * 8);

/* describes a queue */
struct q_ptrs {
	uint32_t		rxq_wb; /* must be first */
	uint32_t		rq_head;
	uint32_t		rq_tail;
	uint32_t		directpath_rx_tail;
	uint64_t		next_timer_tsc;
	uint32_t		storage_tail;
	uint32_t		pad1;
	uint64_t		oldest_tsc;
	uint64_t		rcu_gen;
	uint64_t		run_start_tsc;
	uint64_t		directpath_strides_consumed;

	/* second cache line contains information written by the scheduler */
	uint64_t		curr_grant_gen;
	uint64_t		cede_gen;
	uint64_t		yield_rcu_gen;
	uint64_t		park_gen;
	uint64_t		pad3[4];
};

BUILD_ASSERT(sizeof(struct q_ptrs) == 2 * CACHE_LINE_SIZE);
BUILD_ASSERT(offsetof(struct q_ptrs, curr_grant_gen) % CACHE_LINE_SIZE == 0);

struct congestion_info {
	float			load;
	uint64_t		delay_us;
};

struct runtime_info {
	struct congestion_info congestion;
	uint64_t directpath_strides_posted;
	atomic64_t directpath_strides_consumed;
};

enum {
	HWQ_INVALID = 0,
	HWQ_MLX5,
	HWQ_MLX5_QSTEER,
	HWQ_SPDK_NVME,
	NR_HWQ,
};

struct hardware_queue_spec {
	shmptr_t		descriptor_table;
	shmptr_t		consumer_idx;
	uint32_t		descriptor_log_size;
	uint32_t		nr_descriptors;
	uint32_t		parity_byte_offset;
	uint32_t		parity_bit_mask;
	uint32_t		hwq_type;
};

/* describes a runtime kernel thread */
struct thread_spec {
	struct queue_spec	rxq;
	struct queue_spec	txpktq;
	struct queue_spec	txcmdq;
	shmptr_t		q_ptrs;
	pid_t			tid;
	int32_t			park_efd;

	struct hardware_queue_spec	direct_rxq;
	struct hardware_queue_spec	storage_hwq;
};

enum {
	SCHED_PRIO_LC = 0, /* high priority, latency-critical task */
	SCHED_PRIO_BE,     /* low priority, best-effort task */
};

/* describes scheduler options */
struct sched_spec {
	unsigned int		priority;
	unsigned int		max_cores;
	unsigned int		guaranteed_cores;
	unsigned int		preferred_socket;
	uint64_t		qdelay_us;
	uint64_t		ht_punish_us;
	uint64_t		quantum_us;
#ifdef NO_SCHED
	DEFINE_BITMAP(rt_cores, NCPU);
#endif
};

#define CONTROL_HDR_MAGIC	0x696f6b3a /* "iok:" */

enum {
	DIRECTPATH_REQUEST_NONE = 0,
	DIRECTPATH_REQUEST_REGULAR = 1,
	DIRECTPATH_REQUEST_STRIDED_RMP = 2,
};

/* the main control header */
struct control_hdr {
	unsigned int		version_no;
	unsigned int		magic;
	unsigned int		thread_count;
	unsigned int		request_directpath_queues;
	unsigned long		egress_buf_count;
	shmptr_t		runtime_info;
	uint32_t		ip_addr;
	struct sched_spec	sched_cfg;
	shmptr_t		thread_specs;
	size_t			shared_reg_page_size;
};

/* information shared from iokernel to all runtimes */
struct iokernel_info {
	DEFINE_BITMAP(managed_cores, NCPU);
	unsigned char		rss_key[40];
	struct pci_addr		directpath_pci;
	int			cycles_per_us;
	struct eth_addr		host_mac;
	bool			external_directpath_enabled;
	bool			external_directpath_rmp;
	bool			transparent_hugepages;
	uint64_t		rx_cxl_shm_offset;
	uint64_t		magic_number;
};

BUILD_ASSERT(sizeof(struct iokernel_info) <= IOKERNEL_INFO_SIZE);
