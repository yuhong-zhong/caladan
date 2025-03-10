#include <unistd.h>
#include <getopt.h>
#include <signal.h>
#include <string.h>
#include <fcntl.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <sched.h>
#include <immintrin.h>
#include <x86intrin.h>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <chrono>
#include <fstream>
#include <limits>
#include <random>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <regex>
#include <thread>
#include <cstdlib>
#include <cstring>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <cassert>

using namespace std;
using namespace std::chrono;

#define CHAN_SIZE (8192ul)
#define BUF_SIZE (4ul << 30ul)

#define CXL_MEM_SIZE (64ul << 30ul)

#define BASE_TSC (2.2l)

#define CACHE_LINE_SHIFT 6ul
#define CACHE_LINE_SIZE (1ul << CACHE_LINE_SHIFT)
#define CACHE_LINE_MASK (CACHE_LINE_SIZE - 1ul)

#define PAGE_SHIFT 12ul
#define PAGE_SIZE (1ul << PAGE_SHIFT)
#define PAGE_MASK (PAGE_SIZE - 1ul)

#define HUGE_PAGE_SHIFT 21ul
#define HUGE_PAGE_SIZE (1ul << HUGE_PAGE_SHIFT)
#define HUGE_PAGE_MASK (HUGE_PAGE_SIZE - 1ul)

#define ROUND_DOWN(a, b) ((a) / (b) * (b))
#define ROUND_UP(a, b) (((a) + (b) - 1) / (b) * (b))

#define clflushopt(addr) asm volatile("clflushopt %0" : "+m" (*(volatile char *)(addr)))
#define clwb(addr) asm volatile("clwb %0" : "+m" (*(volatile char *)(addr)))
#define pause() asm volatile("pause")

#define BUILD_ASSERT(cond) \
	static_assert(cond, "build-time condition failed")

#ifndef likely
#define likely(x) __builtin_expect(!!(x), 1)
#endif
#ifndef unlikely
#define unlikely(x) __builtin_expect(!!(x), 0)
#endif

#define barrier() asm volatile("" ::: "memory")

#define	ACCESS_ONCE(x) (*(volatile typeof(x) *)&(x))

#define type_is_native(t) \
	(sizeof(t) == sizeof(char)  || \
	 sizeof(t) == sizeof(short) || \
	 sizeof(t) == sizeof(int)   || \
	 sizeof(t) == sizeof(long))

#define is_power_of_two(x) ((x) != 0 && !((x) & ((x) - 1)))

#define store_release(p, v)			\
do {						\
	BUILD_ASSERT(type_is_native(*p));	\
	barrier();				\
	ACCESS_ONCE(*p) = v;			\
} while (0)

#define load_acquire(p)				\
({						\
	BUILD_ASSERT(type_is_native(*p));	\
	typeof(*p) __p = ACCESS_ONCE(*p);	\
	barrier();				\
	__p;					\
})

#define log_ratelimited(fmt, ...)			\
({							\
	static uint64_t __last_us = 0;			\
	static uint64_t __suppressed = 0;		\
	uint64_t __cur_us = __rdtsc() / (1000 * BASE_TSC); \
	if (__cur_us - __last_us >= 1e6) {		\
		if (__suppressed) {			\
			fprintf(stderr, "%s:%d %s() suppressed %ld times\n", \
			        __FILE__, __LINE__, __func__, __suppressed); \
			__suppressed = 0;		\
		}					\
		fprintf(stderr, fmt, ##__VA_ARGS__);	\
		__last_us = __cur_us;			\
	} else						\
		__suppressed++;				\
})

#define batch_clflushopt(addr, len)												\
	do {															\
		const void *_addr = (addr);											\
		const uint64_t _len = (len);											\
		uint8_t *_ptr = (uint8_t *)(ROUND_DOWN((uint64_t)(_addr), CACHE_LINE_SIZE));					\
		uint64_t _num_lines = ((((uint64_t)(_addr)) - ((uint64_t)_ptr)) + _len + CACHE_LINE_SIZE - 1) / CACHE_LINE_SIZE; \
		for (uint64_t _i = 0; _i < _num_lines; ++_i) {									\
			asm volatile("clflushopt %0" : "+m" (*(volatile char *)(_ptr + _i * CACHE_LINE_SIZE)));			\
		}														\
	} while (0)

#define batch_clwb(addr, len)													\
	do {															\
		const void *_addr = (addr);											\
		const uint64_t _len = (len);											\
		uint8_t *_ptr = (uint8_t *)(ROUND_DOWN((uint64_t)(_addr), CACHE_LINE_SIZE));					\
		uint64_t _num_lines = ((((uint64_t)(_addr)) - ((uint64_t)_ptr)) + _len + CACHE_LINE_SIZE - 1) / CACHE_LINE_SIZE; \
		for (uint64_t _i = 0; _i < _num_lines; ++_i) {									\
			asm volatile("clwb %0" : "+m" (*(volatile char *)(_ptr + _i * CACHE_LINE_SIZE)));			\
		}														\
	} while (0)

#define BUG_ON(cond)										\
	do {											\
		if (cond) {									\
		fprintf(stderr, "BUG_ON: %s (L%d) %s\n", __FILE__, __LINE__, __FUNCTION__);	\
		raise(SIGABRT);									\
		}										\
	} while (0)

#define prefetch0(x) __builtin_prefetch((x), 0, 3)
#define prefetch1(x) __builtin_prefetch((x), 0, 2)
#define prefetch2(x) __builtin_prefetch((x), 0, 1)
#define prefetchnta(x) __builtin_prefetch((x), 0, 0)
#define prefetch(x) prefetch0(x)


struct lrpc_msg {
	uint64_t	cmd;
	unsigned long	payload;
};

#define LRPC_DONE_PARITY	(1UL << 63)
#define LRPC_CMD_MASK		(~LRPC_DONE_PARITY)

#define LRPC_BATCH_SIZE (CACHE_LINE_SIZE / sizeof(struct lrpc_msg))
struct batch_lrpc_msg {
	struct lrpc_msg msg_arr[LRPC_BATCH_SIZE];
} __attribute__((aligned(CACHE_LINE_SIZE)));

struct msg_chan_out {
	uint32_t	send_head;
	uint32_t	send_tail;
	struct lrpc_msg	*tbl;
	uint32_t 	*recv_head_wb;
	uint32_t	size;
	uint32_t	clwb_send_head;
} __attribute__((aligned(CACHE_LINE_SIZE)));

static inline int msg_init_out(struct msg_chan_out *chan, struct lrpc_msg *tbl,
			       unsigned int size, uint32_t *recv_head_wb)
{
	if (!is_power_of_two(size))
		return -EINVAL;

	memset(chan, 0, sizeof(*chan));
	chan->tbl = tbl;
	chan->size = size;
	chan->recv_head_wb = recv_head_wb;
	return 0;
}

bool msg_send(struct msg_chan_out *chan, uint64_t cmd,
	      unsigned long payload)
{
	struct lrpc_msg *dst;

	assert(!(cmd & LRPC_DONE_PARITY));

	if (unlikely(chan->send_head - chan->send_tail >= chan->size)) {
		clflushopt(chan->recv_head_wb);
		_mm_mfence();
		chan->send_tail = ACCESS_ONCE(*chan->recv_head_wb);
		if (chan->send_head - chan->send_tail == chan->size) {
			return false;
		}
	}

	dst = &chan->tbl[chan->send_head & (chan->size - 1)];
	cmd |= (chan->send_head++ & chan->size) ? 0 : LRPC_DONE_PARITY;
	dst->payload = payload;
	store_release(&dst->cmd, cmd);

	if (chan->send_head % (CACHE_LINE_SIZE / sizeof(*chan->tbl)) == 0)
		clwb(dst);

	return true;
}

bool huge_msg_send(struct msg_chan_out *chan, uint64_t cmd, unsigned long payload)
{
	struct lrpc_msg *dst;

	if (unlikely(chan->send_head - chan->send_tail + LRPC_BATCH_SIZE - 1 >= chan->size)) {
		clflushopt(chan->recv_head_wb);
		_mm_mfence();
		chan->send_tail = ACCESS_ONCE(*chan->recv_head_wb);
		if (chan->send_head - chan->send_tail + LRPC_BATCH_SIZE - 1 >= chan->size) {
			return false;
		}
	}

	cmd |= (chan->send_head & chan->size) ? 0 : LRPC_DONE_PARITY;

	struct batch_lrpc_msg batch_msg;
	batch_msg.msg_arr[0].payload = payload;
	batch_msg.msg_arr[0].cmd = cmd;
	__m512i zmm1 = _mm512_load_si512(&batch_msg);

	dst = &chan->tbl[chan->send_head & (chan->size - 1)];
	_mm512_stream_si512((__m512i*) dst, zmm1);
	// _mm_sfence();

	chan->send_head += LRPC_BATCH_SIZE;
	return true;
}

void msg_out_sync(struct msg_chan_out *chan)
{
	clwb(&chan->tbl[chan->send_head & (chan->size - 1)]);
}

// bool msg_send_ntstore_zero(struct msg_chan_out *chan)
// {
// 	struct lrpc_msg *dst;

// 	// if (unlikely(chan->send_head - chan->send_tail + LRPC_BATCH_SIZE - 1 >= chan->size)) {
// 	// 	clflushopt(chan->recv_head_wb);
// 	// 	_mm_mfence();
// 	// 	chan->send_tail = ACCESS_ONCE(*chan->recv_head_wb);
// 	// 	if (chan->send_head - chan->send_tail + LRPC_BATCH_SIZE - 1 >= chan->size) {
// 	// 		return false;
// 	// 	}
// 	// }

// 	struct batch_lrpc_msg batch_msg;
// 	__m512i zmm1 = _mm512_load_si512(&batch_msg);

// 	dst = &chan->tbl[chan->send_head & (chan->size - 1)];
// 	_mm512_stream_si512((__m512i*) dst, zmm1);
// 	// _mm_sfence();

// 	chan->send_head += LRPC_BATCH_SIZE;
// 	return true;
// }

struct msg_chan_in {
	struct lrpc_msg	*tbl;
	uint32_t 	*recv_head_wb;
	uint32_t	recv_head;
	uint32_t	size;
	uint32_t	new_recv_head;
	uint32_t	prefetch_len;
	uint32_t	hit_count;
} __attribute__((aligned(CACHE_LINE_SIZE)));

static inline int msg_init_in(struct msg_chan_in *chan, struct lrpc_msg *tbl,
			      unsigned int size, uint32_t *recv_head_wb)
{
	if (!is_power_of_two(size))
		return -EINVAL;

	memset(chan, 0, sizeof(*chan));
	chan->tbl = tbl;
	chan->size = size;
	chan->recv_head_wb = recv_head_wb;
	return 0;
}

#define PREFETCH_LEN 16

bool msg_recv(struct msg_chan_in *chan, uint64_t *cmd_out,
	      unsigned long *payload_out)
{
	struct lrpc_msg *m = &chan->tbl[chan->recv_head & (chan->size - 1)];
	uint64_t parity = (chan->recv_head & chan->size) ?
			  0 : LRPC_DONE_PARITY;
	uint64_t cmd;

	cmd = load_acquire(&m->cmd);
	if ((cmd & LRPC_DONE_PARITY) != parity) {
		for (int i = 0; i <= chan->prefetch_len; i++)
			clflushopt(&chan->tbl[(chan->recv_head + i * CACHE_LINE_SIZE / sizeof(*m)) & (chan->size - 1)]);
		// chan->prefetch_len = (chan->prefetch_len <= 5) ? 1 : (chan->prefetch_len - 4);
		// chan->hit_count = 0;
		return false;
	}
	*cmd_out = cmd & LRPC_CMD_MASK;
	*payload_out = m->payload;
	chan->recv_head++;

	if ((chan->recv_head % (CACHE_LINE_SIZE / sizeof(*m))) == 1) {
		for (int i = 1; i <= chan->prefetch_len; i++) {
			prefetch(&chan->tbl[(chan->recv_head + i * CACHE_LINE_SIZE / sizeof(*m)) & (chan->size - 1)]);
		}
	}

	// chan->hit_count += 1;
	// if (chan->hit_count >= (chan->prefetch_len + 1) * (CACHE_LINE_SIZE / sizeof(*m))) {
	// 	chan->prefetch_len = (chan->prefetch_len == PREFETCH_LEN) ? PREFETCH_LEN : (chan->prefetch_len + 1);
	// 	chan->hit_count = 0;
	// }

	store_release(chan->recv_head_wb, chan->recv_head);

	if ((chan->recv_head % (chan->size / 8)) == 0)
		clwb(chan->recv_head_wb);
	if ((chan->recv_head % (CACHE_LINE_SIZE / sizeof(*m))) == 0)
		clflushopt(m);

	return true;
}

bool huge_msg_recv(struct msg_chan_in *chan, uint64_t *cmd_out,
	      unsigned long *payload_out)
{
	struct lrpc_msg *m = &chan->tbl[chan->recv_head & (chan->size - 1)];
	uint64_t parity = (chan->recv_head & chan->size) ?
			  0 : LRPC_DONE_PARITY;
	uint64_t cmd;

	// prefetch(&chan->tbl[(chan->recv_head + PREFETCH_LEN * CACHE_LINE_SIZE / sizeof(*m)) & (chan->size - 1)]);

	cmd = load_acquire(&m->cmd);
	if ((cmd & LRPC_DONE_PARITY) != parity) {
		for (int i = 0; i <= chan->prefetch_len; i++)
			clflushopt(&chan->tbl[(chan->recv_head + i * CACHE_LINE_SIZE / sizeof(*m)) & (chan->size - 1)]);
		// chan->prefetch_len = (chan->prefetch_len <= 3) ? 1 : (chan->prefetch_len - 2);
		// chan->hit_count = 0;
		return false;
	}
	*cmd_out = cmd & LRPC_CMD_MASK;
	*payload_out = m->payload;
	chan->recv_head += LRPC_BATCH_SIZE;

	for (int i = 1; i <= chan->prefetch_len; i++) {
		prefetch(&chan->tbl[(chan->recv_head + i * CACHE_LINE_SIZE / sizeof(*m)) & (chan->size - 1)]);
	}

	// chan->hit_count += 1;
	// if (chan->hit_count - 1 >= chan->prefetch_len) {
	// 	chan->prefetch_len = (chan->prefetch_len == PREFETCH_LEN) ? PREFETCH_LEN : (chan->prefetch_len + 1);
	// 	chan->hit_count = 0;
	// }

	store_release(chan->recv_head_wb, chan->recv_head);

	if ((chan->recv_head % (chan->size / 8)) == 0)
		clwb(chan->recv_head_wb);
	clflushopt(m);

	return true;
}

void run_on_core(uint64_t core) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(core, &cpuset);
        int sched_result = sched_setaffinity(0, sizeof(cpuset), &cpuset);
        BUG_ON(sched_result != 0);
}

struct lrpc_chan_in {
	struct lrpc_msg	*tbl;
	uint32_t	*recv_head_wb;
	uint32_t	recv_head;
	uint32_t	size;
};

int lrpc_init_in(struct lrpc_chan_in *chan, struct lrpc_msg *tbl,
	unsigned int size, uint32_t *recv_head_wb)
{
	if (!is_power_of_two(size))
		return -EINVAL;

	memset(chan, 0, sizeof(*chan));
	chan->tbl = tbl;
	chan->size = size;
	chan->recv_head_wb = recv_head_wb;
	return 0;
}

bool lrpc_recv(struct lrpc_chan_in *chan, uint64_t *cmd_out,
	unsigned long *payload_out)
{
	struct lrpc_msg *m = &chan->tbl[chan->recv_head & (chan->size - 1)];
	uint64_t parity = (chan->recv_head & chan->size) ? 0 : LRPC_DONE_PARITY;
	uint64_t cmd;

	cmd = load_acquire(&m->cmd);
	if ((cmd & LRPC_DONE_PARITY) != parity)
		return false;
	chan->recv_head++;

	*cmd_out = cmd & LRPC_CMD_MASK;
	*payload_out = m->payload;
	store_release(chan->recv_head_wb, chan->recv_head);
	return true;
}

struct lrpc_chan_out {
	uint32_t	send_head;
	uint32_t	send_tail;
	struct lrpc_msg	*tbl;
	uint32_t	*recv_head_wb;
	uint32_t	size;
	uint32_t	pad;
};

int lrpc_init_out(struct lrpc_chan_out *chan, struct lrpc_msg *tbl,
	unsigned int size, uint32_t *recv_head_wb)
{
	if (!is_power_of_two(size))
		return -EINVAL;

	memset(chan, 0, sizeof(*chan));
	chan->tbl = tbl;
	chan->size = size;
	chan->recv_head_wb = recv_head_wb;
	return 0;
}

bool __lrpc_send(struct lrpc_chan_out *chan, uint64_t cmd,
	unsigned long payload)
{
	struct lrpc_msg *dst;

	assert(chan->send_head - chan->send_tail == chan->size);

	chan->send_tail = load_acquire(chan->recv_head_wb);
	if (chan->send_head - chan->send_tail == chan->size)
		return false;

	dst = &chan->tbl[chan->send_head & (chan->size - 1)];
	dst->payload = payload;

	cmd |= (chan->send_head++ & chan->size) ? 0 : LRPC_DONE_PARITY;
	store_release(&dst->cmd, cmd);
	return true;
}

bool lrpc_send(struct lrpc_chan_out *chan, uint64_t cmd,
	unsigned long payload)
{
	struct lrpc_msg *dst;

	assert(!(cmd & LRPC_DONE_PARITY));

	if (unlikely(chan->send_head - chan->send_tail >= chan->size))
		return __lrpc_send(chan, cmd, payload);

	dst = &chan->tbl[chan->send_head & (chan->size - 1)];
	cmd |= (chan->send_head++ & chan->size) ? 0 : LRPC_DONE_PARITY;
	dst->payload = payload;
	store_release(&dst->cmd, cmd);
	return true;
}

enum lrpc_command {
	LRPC_CMD_READ = 0,
	LRPC_CMD_WRITE = 1,
	LRPC_CMD_DONE = 2,
	LRPC_CMD_STOP = 3,
};

uint64_t block_size;
uint64_t num_blocks;

void send_recv_thread_fn(uint8_t *lrpc_in_buf, uint8_t *lrpc_out_buf) {
	struct lrpc_chan_in chan_in;
	memset(&chan_in, 0, sizeof(chan_in));
	lrpc_init_in(&chan_in, (struct lrpc_msg *) lrpc_in_buf, CHAN_SIZE, (uint32_t *) (lrpc_in_buf + HUGE_PAGE_SIZE));

	struct lrpc_chan_out chan_out;
	memset(&chan_out, 0, sizeof(chan_out));
	lrpc_init_out(&chan_out, (struct lrpc_msg *) lrpc_out_buf, CHAN_SIZE, (uint32_t *) (lrpc_out_buf + HUGE_PAGE_SIZE));

	uint8_t *local_buf = (uint8_t *) aligned_alloc(HUGE_PAGE_SIZE, block_size);
	memset(local_buf, 0, block_size);

	while (true) {
		uint64_t cmd;
		unsigned long payload;
		while (!lrpc_recv(&chan_in, &cmd, &payload)) {
			pause();
		}

		uint8_t *addr = (uint8_t *) payload;

		bool sent;
		switch (cmd) {
		case LRPC_CMD_READ:
			batch_clflushopt(addr, block_size);
			memcpy(local_buf, addr, block_size);

			sent = lrpc_send(&chan_out, LRPC_CMD_DONE, payload);
			BUG_ON(!sent);
			break;
		case LRPC_CMD_WRITE:
			memcpy(addr, local_buf, block_size);
			batch_clflushopt(addr, block_size);

			sent = lrpc_send(&chan_out, LRPC_CMD_DONE, payload);
			BUG_ON(!sent);
			break;
		case LRPC_CMD_STOP:
			return;
		default:
			printf("unexpected command: %lu\n", cmd);
			BUG_ON(true);
		}
	}
}

enum msg_command {
	MSG_CMD_SEND = 0,
};

bool has_pending_send(const vector<uint64_t> &send_to) {
	for (uint64_t size : send_to) {
		if (size > 0)
			return true;
	}
	return false;
}

bool has_pending_receive(const vector<uint64_t> &receive_from) {
	for (uint64_t size : receive_from) {
		if (size > 0)
			return true;
	}
	return false;
}

enum send_ordering_policy {
	SEND_UNORDERED = 0,
	SEND_ORDERED,
	SEND_ONCE,
	NR_SEND_POLICY,
};

int main(int argc, char *argv[]) {
	if (argc != 10) {
		fprintf(stderr, "Usage: %s <CXL dax> <group size> <rank> <thread count> <block size> <send-to list> <receive-from list> <send ordering> <compute size>\n", argv[0]);
		exit(1);
	}
	char *cxl_dax_path = argv[1];
	int group_size = atoi(argv[2]);
	int rank = atoi(argv[3]);
	int thread_count = atoi(argv[4]);
	block_size = stoll(argv[5]);
	num_blocks = BUF_SIZE / block_size;
	// send-to list format: "0:4096,1:8192", which means sending 4096B to rank 0 and 8192B to rank 1
	string send_to_list(argv[6]);
	// receive-from list format: "0:4096,1:8192", which means receiving 4096B from rank 0 and 8192B from rank 1
	string receive_from_list(argv[7]);
	int send_ordering = atoi(argv[8]);
	uint64_t compute_size = stoll(argv[9]);

	BUG_ON(group_size <= 1);
	BUG_ON(rank < 0 || rank >= group_size);
	BUG_ON(thread_count <= 0);
	BUG_ON(block_size % HUGE_PAGE_SIZE != 0);
	BUG_ON(block_size == 0);
	BUG_ON(send_ordering < 0 || send_ordering >= NR_SEND_POLICY);
	BUG_ON(compute_size % HUGE_PAGE_SIZE != 0);

	vector<uint64_t> send_to(group_size, 0);
	vector<uint64_t> receive_from(group_size, 0);
	istringstream send_to_stream(send_to_list);
	istringstream receive_from_stream(receive_from_list);
	string token;
	while (getline(send_to_stream, token, ',')) {
		istringstream token_stream(token);
		string rank_str, size_str;
		getline(token_stream, rank_str, ':');
		getline(token_stream, size_str, ':');
		int rank = stoi(rank_str);
		BUG_ON(rank < 0 || rank >= group_size);
		uint64_t size = stoll(size_str);
		send_to[rank] = size;
		BUG_ON(size % block_size != 0);
	}
	while (getline(receive_from_stream, token, ',')) {
		istringstream token_stream(token);
		string rank_str, size_str;
		getline(token_stream, rank_str, ':');
		getline(token_stream, size_str, ':');
		int rank = stoi(rank_str);
		BUG_ON(rank < 0 || rank >= group_size);
		uint64_t size = stoll(size_str);
		receive_from[rank] = size;
		BUG_ON(size % block_size != 0);
	}
	for (int i = 0; i < group_size; ++i) {
		printf("rank %d send to      rank %d: %lu B\n", rank, i, send_to[i]);
		printf("rank %d receive from rank %d: %lu B\n", rank, i, receive_from[i]);
	}

	int fd = open(cxl_dax_path, O_RDWR);
	BUG_ON(fd < 0);
	uint8_t *cxl_buf = (uint8_t *) mmap(NULL, CXL_MEM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	BUG_ON(cxl_buf == MAP_FAILED);
	close(fd);
	uint8_t *cxl_base = cxl_buf;

	uint8_t *ready_arr = cxl_buf;
	BUG_ON(ready_arr[rank * CACHE_LINE_SIZE] != 0);
	BUG_ON(group_size * CACHE_LINE_SIZE > HUGE_PAGE_SIZE);
	cxl_buf += HUGE_PAGE_SIZE;

	uint8_t *finished_arr = cxl_buf;
	BUG_ON(finished_arr[rank * CACHE_LINE_SIZE] != 0);
	BUG_ON(group_size * CACHE_LINE_SIZE > HUGE_PAGE_SIZE);
	cxl_buf += HUGE_PAGE_SIZE;

	// initialize memory channels
	struct msg_chan_out *group_chan_outs = (struct msg_chan_out *) malloc(group_size * sizeof(struct msg_chan_out));
	BUG_ON(group_chan_outs == NULL);
	struct msg_chan_in *group_chan_ins = (struct msg_chan_in *) malloc(group_size * sizeof(struct msg_chan_in));
	BUG_ON(group_chan_ins == NULL);
	BUG_ON(CHAN_SIZE * sizeof(struct lrpc_msg) > HUGE_PAGE_SIZE);
	for (int i = 0; i < group_size; ++i) {
		memset(&group_chan_outs[i], 0, sizeof(struct msg_chan_out));
		msg_init_out(&group_chan_outs[i], (struct lrpc_msg *) (cxl_buf + (i * group_size + rank) * 2 * HUGE_PAGE_SIZE),
		             CHAN_SIZE, (uint32_t *) (cxl_buf + (i * group_size + rank) * 2 * HUGE_PAGE_SIZE + HUGE_PAGE_SIZE));

		memset(&group_chan_ins[i], 0, sizeof(struct msg_chan_in));
		msg_init_in(&group_chan_ins[i], (struct lrpc_msg *) (cxl_buf + (rank * group_size + i) * 2 * HUGE_PAGE_SIZE),
		            CHAN_SIZE, (uint32_t *) (cxl_buf + (rank * group_size + i) * 2 * HUGE_PAGE_SIZE + HUGE_PAGE_SIZE));
		group_chan_ins[i].prefetch_len = 16;
	}
	cxl_buf += 2 * group_size * group_size * HUGE_PAGE_SIZE;

	// initialize buffer areas
	vector<uint8_t *> buf_areas(group_size);
	vector<uint64_t> buf_indices(group_size);
	for (int i = 0; i < group_size; ++i) {
		buf_areas[i] = cxl_buf;
		buf_indices[i] = 0;
		cxl_buf += BUF_SIZE;
	}

	// initialize per-thread lrpc channels
	vector<struct lrpc_chan_out> lrpc_chan_outs(thread_count);
	vector<struct lrpc_chan_in> lrpc_chan_ins(thread_count);
	uint8_t *lrpc_out_buf = (uint8_t *) aligned_alloc(PAGE_SIZE, thread_count * HUGE_PAGE_SIZE * 2);
	BUG_ON(lrpc_out_buf == NULL);
	uint8_t *lrpc_in_buf = (uint8_t *) aligned_alloc(PAGE_SIZE, thread_count * HUGE_PAGE_SIZE * 2);
	BUG_ON(lrpc_in_buf == NULL);
	for (int i = 0; i < thread_count; ++i) {
		memset(&lrpc_chan_outs[i], 0, sizeof(struct lrpc_chan_out));
		lrpc_init_out(&lrpc_chan_outs[i], (struct lrpc_msg *) (lrpc_out_buf + i * HUGE_PAGE_SIZE * 2),
		              CHAN_SIZE, (uint32_t *) (lrpc_out_buf + i * HUGE_PAGE_SIZE * 2 + HUGE_PAGE_SIZE));

		memset(&lrpc_chan_ins[i], 0, sizeof(struct lrpc_chan_in));
		lrpc_init_in(&lrpc_chan_ins[i], (struct lrpc_msg *) (lrpc_in_buf + i * HUGE_PAGE_SIZE * 2),
		             CHAN_SIZE, (uint32_t *) (lrpc_in_buf + i * HUGE_PAGE_SIZE * 2 + HUGE_PAGE_SIZE));
	}

	// initialize threads
	vector<thread> threads;
	for (int i = 0; i < thread_count; ++i) {
		threads.push_back(thread(send_recv_thread_fn, lrpc_out_buf + i * HUGE_PAGE_SIZE * 2, lrpc_in_buf + i * HUGE_PAGE_SIZE * 2));
	}
	// sleep(1);

	// initialize computer buffer
	uint8_t *compute_buf = (uint8_t *) aligned_alloc(HUGE_PAGE_SIZE, compute_size);
	BUG_ON(compute_buf == NULL);
	for (uint64_t i = 0; i < compute_size / sizeof(uint64_t); i += sizeof(uint64_t)) {
		compute_buf[i + 0] = '1';
		compute_buf[i + 1] = '2';
		compute_buf[i + 2] = '3';
		compute_buf[i + 3] = '4';
		compute_buf[i + 4] = '5';
		compute_buf[i + 5] = '6';
		compute_buf[i + 6] = '7';
		compute_buf[i + 7] = '\0';
	}

	// signal ready and synchronize
	printf("rank %d ready\n", rank);
	if (rank != 0) {
		BUG_ON(ready_arr[rank * CACHE_LINE_SIZE] != 0);
		ready_arr[rank * CACHE_LINE_SIZE] = 1;
		clflushopt(ready_arr + rank * CACHE_LINE_SIZE);

		while (ready_arr[0] == 0) {
			clflushopt(ready_arr);
			pause();
		}
	} else {
		for (int i = 1; i < group_size; ++i) {
			while (ready_arr[i * CACHE_LINE_SIZE] == 0) {
				clflushopt(ready_arr + i * CACHE_LINE_SIZE);
				pause();
			}
		}
		BUG_ON(ready_arr[0] != 0);
		ready_arr[0] = 1;
		clflushopt(ready_arr);
	}
	uint64_t start = __rdtsc();

	// computing
	printf("rank %d computing\n", rank);
	for (uint64_t i = 0; i < compute_size / sizeof(uint64_t); i += sizeof(uint64_t)) {
		volatile uint64_t val = stoll((char *) (compute_buf + i * sizeof(uint64_t)));
	}
	printf("rank %d done computing, time: %lu ms\n", rank, (uint64_t) ((__rdtsc() - start) / BASE_TSC / 1e6));

	// sending
	printf("rank %d sending\n", rank);
	vector<int> thread_to_rank(thread_count, -1);
	deque<int> overflow_send_to;  // depending on ordering
	int cur_thread = 0;
	int cur_rank = 0;
	while (has_pending_send(send_to)) {
		if (thread_to_rank[cur_thread] != -1) {
			// thread is busy, wait for it to finish
			uint64_t cmd;
			unsigned long payload;
			switch (send_ordering) {
			case SEND_ORDERED:
			{
				while (!lrpc_recv(&lrpc_chan_ins[cur_thread], &cmd, &payload)) {
					pause();
				}
				BUG_ON(cmd != LRPC_CMD_DONE);
				break;
			}
			case SEND_UNORDERED:
			case SEND_ONCE:
			{
				bool received = lrpc_recv(&lrpc_chan_ins[cur_thread], &cmd, &payload);
				if (!received) {
					cur_thread = (cur_thread + 1) % thread_count;
					continue;
				}
				BUG_ON(cmd != LRPC_CMD_DONE);
				break;
			}
			default:
				BUG_ON(true);
			}

			// send to the target rank
			int target_rank = thread_to_rank[cur_thread];
			switch (send_ordering) {
			case SEND_ORDERED:
			{
				while (!msg_send(&group_chan_outs[target_rank], MSG_CMD_SEND, rank)) {
					pause();
				}
				break;
			}
			case SEND_UNORDERED:
			{
				bool sent = msg_send(&group_chan_outs[target_rank], MSG_CMD_SEND, rank);
				if (!sent)
					overflow_send_to.push_back(target_rank);
				break;
			}
			case SEND_ONCE:
				overflow_send_to.push_back(target_rank);
				break;
			default:
				BUG_ON(true);
			}
		}

		// find the target rank with pending send
		while (send_to[cur_rank] == 0) {
			cur_rank = (cur_rank + 1) % group_size;
		}
		BUG_ON(send_to[cur_rank] == 0);

		uint64_t buf_index = buf_indices[rank];
		buf_indices[rank] = (buf_index + 1) % num_blocks;

		bool sent = lrpc_send(&lrpc_chan_outs[cur_thread], LRPC_CMD_WRITE, (unsigned long) buf_areas[rank] + buf_index * block_size);
		BUG_ON(!sent);
		thread_to_rank[cur_thread] = cur_rank;
		send_to[cur_rank] -= block_size;

		cur_thread = (cur_thread + 1) % thread_count;
		cur_rank = (cur_rank + 1) % group_size;
	}
	for (int i = 0; i < thread_count; ++i) {
		if (thread_to_rank[cur_thread] != -1) {
			uint64_t cmd;
			unsigned long payload;
			while (!lrpc_recv(&lrpc_chan_ins[cur_thread], &cmd, &payload)) {
				pause();
			}
			BUG_ON(cmd != LRPC_CMD_DONE);

			int target_rank = thread_to_rank[cur_thread];
			switch (send_ordering) {
			case SEND_ORDERED:
			{
				while (!msg_send(&group_chan_outs[target_rank], MSG_CMD_SEND, rank)) {
					pause();
				}
				break;
			}
			case SEND_UNORDERED:
			{
				bool sent = msg_send(&group_chan_outs[target_rank], MSG_CMD_SEND, rank);
				if (!sent)
					overflow_send_to.push_back(target_rank);
				break;
			}
			case SEND_ONCE:
				overflow_send_to.push_back(target_rank);
				break;
			default:
				BUG_ON(true);
			}
		}
		cur_thread = (cur_thread + 1) % thread_count;
	}
	printf("rank %d overflow_send_to size: %lu\n", rank, overflow_send_to.size());
	while (!overflow_send_to.empty()) {
		BUG_ON(send_ordering == SEND_ORDERED);
		uint64_t overflow_size = overflow_send_to.size();
		for (int i = 0; i < overflow_size; ++i) {
			int cur_rank = overflow_send_to.front();
			overflow_send_to.pop_front();

			bool sent = msg_send(&group_chan_outs[cur_rank], MSG_CMD_SEND, rank);
			if (!sent)
				overflow_send_to.push_back(cur_rank);
		}
		for (int i = 0; i < group_size; ++i) {
			msg_out_sync(&group_chan_outs[i]);
		}
		pause();
	}
	for (int i = 0; i < group_size; ++i) {
		msg_out_sync(&group_chan_outs[i]);
	}
	printf("rank %d done sending, time: %lu ms\n", rank, (uint64_t) ((__rdtsc() - start) / BASE_TSC / 1e6));

	// receiving
	printf("rank %d receiving\n", rank);
	vector<bool> thread_available(thread_count, true);
	cur_rank = 0;
	while (has_pending_receive(receive_from)) {
		for (int tid = 0; tid < thread_count; ++tid) {
			if (!has_pending_receive(receive_from))
				break;
			// monitor completed threads
			if (!thread_available[tid]) {
				uint64_t cmd;
				unsigned long payload;
				bool received = lrpc_recv(&lrpc_chan_ins[tid], &cmd, &payload);
				if (received) {
					BUG_ON(cmd != LRPC_CMD_DONE);
					thread_available[tid] = true;
				}
			}
			// skip busy threads
			if (!thread_available[tid])
				continue;

			// find a rank to receive from
			while (true) {
				if (receive_from[cur_rank] == 0) {
					cur_rank = (cur_rank + 1) % group_size;
					continue;
				}
				uint64_t cmd;
				unsigned long payload;
				bool received = msg_recv(&group_chan_ins[cur_rank], &cmd, &payload);
				if (received) {
					break;
				} else {
					cur_rank = (cur_rank + 1) % group_size;
				}
			}
			BUG_ON(receive_from[cur_rank] == 0);

			uint64_t buf_index = buf_indices[cur_rank];
			buf_indices[cur_rank] = (buf_index + 1) % num_blocks;

			bool sent = lrpc_send(&lrpc_chan_outs[tid], LRPC_CMD_READ, (unsigned long) buf_areas[cur_rank] + buf_index * block_size);
			BUG_ON(!sent);
			thread_available[tid] = false;
			receive_from[cur_rank] -= block_size;

			cur_rank = (cur_rank + 1) % group_size;
		}
	}
	for (int i = 0; i < thread_count; ++i) {
		if (!thread_available[i]) {
			uint64_t cmd;
			unsigned long payload;
			while (!lrpc_recv(&lrpc_chan_ins[i], &cmd, &payload)) {
				pause();
			}
			BUG_ON(cmd != LRPC_CMD_DONE);
		}
	}
	printf("rank %d done receiving, time: %lu ms\n", rank, (uint64_t) ((__rdtsc() - start) / BASE_TSC / 1e6));

	// signal finished and synchronize
	printf("rank %d finished\n", rank);
	if (rank != 0) {
		BUG_ON(finished_arr[rank * CACHE_LINE_SIZE] != 0);
		finished_arr[rank * CACHE_LINE_SIZE] = 1;
		clflushopt(finished_arr + rank * CACHE_LINE_SIZE);
	} else {
		for (int i = 1; i < group_size; ++i) {
			while (finished_arr[i * CACHE_LINE_SIZE] == 0) {
				clflushopt(finished_arr + i * CACHE_LINE_SIZE);
				pause();
			}
		}
		uint64_t end = __rdtsc();
		double duration_ns = (end - start) / BASE_TSC;
		printf("duration: %.2f ms\n", duration_ns / 1e6);
	}

	printf("rank %d exiting\n", rank);
	for (int i = 0; i < thread_count; ++i) {
		lrpc_send(&lrpc_chan_outs[i], LRPC_CMD_STOP, 0);
	}
	for (int i = 0; i < thread_count; ++i) {
		threads[i].join();
	}

	return 0;
}
