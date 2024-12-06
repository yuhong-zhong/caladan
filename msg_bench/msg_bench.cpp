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

#define CXL_MEM_SIZE (1ul << 30ul)
#define NUMA0_CORE (0)
#define NUMA1_CORE (52)
#define BASE_TSC (2.3l)

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

// void msg_out_sync(struct msg_chan_out *chan)
// {
// 	// while (chan->send_head > chan->clwb_send_head) {
// 	// 	clwb(&chan->tbl[chan->clwb_send_head & (chan->size - 1)]);
// 	// 	chan->clwb_send_head = MIN(chan->send_head, chan->clwb_send_head + CACHE_LINE_SIZE / sizeof(*chan->tbl));
// 	// }

// 	struct lrpc_msg *dst;
// 	dst = &chan->tbl[chan->send_head & (chan->size - 1)];
// 	clwb(dst);

// 	chan->send_tail = ACCESS_ONCE(*chan->recv_head_wb);
// 	if (chan->send_head - chan->send_tail >= chan->size / 2) {
// 		clflushopt(chan->recv_head_wb);
// 		_mm_mfence();
// 		chan->send_tail = ACCESS_ONCE(*chan->recv_head_wb);
// 	}
// }

bool msg_send(struct msg_chan_out *chan, uint64_t cmd,
	      unsigned long payload)
{
	struct lrpc_msg *dst;

	assert(!(cmd & LRPC_DONE_PARITY));

	if (unlikely(chan->send_head - chan->send_tail >= chan->size)) {
		chan->send_tail = ACCESS_ONCE(*chan->recv_head_wb);
		if (chan->send_head - chan->send_tail == chan->size) {
			clflushopt(chan->recv_head_wb);
			return false;
		}
	}

	dst = &chan->tbl[chan->send_head & (chan->size - 1)];
	cmd |= (chan->send_head & chan->size) ? 0 : LRPC_DONE_PARITY;
	dst->payload = payload;
	store_release(&dst->cmd, cmd);
	// only allow clwb after the message is written
	store_release(&chan->send_head, chan->send_head + 1);

	if (chan->send_head % (CACHE_LINE_SIZE / sizeof(*chan->tbl)) == 0)
		clwb(dst);

	return true;
}

struct msg_chan_in {
	struct lrpc_msg	*tbl;
	uint32_t 	*recv_head_wb;
	uint32_t	recv_head;
	uint32_t	size;
	uint32_t	new_recv_head;
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

// #define LRPC_IN_SYNC_BURST_SIZE 64
// static inline void msg_in_sync(struct msg_chan_in *chan)
// {
// 	int i;
// 	struct lrpc_msg *m;
// 	uint64_t cmd, parity;

// 	clwb(chan->recv_head_wb);

// 	chan->new_recv_head = MAX(ACCESS_ONCE(*chan->recv_head_wb), chan->new_recv_head);
// 	for (i = 0; i < LRPC_IN_SYNC_BURST_SIZE; ++i) {
// 		m = &chan->tbl[(chan->new_recv_head) & (chan->size - 1)];
// 		cmd = ACCESS_ONCE(m->cmd);
// 		parity = ((chan->new_recv_head) & chan->size) ?
// 			 0 : LRPC_DONE_PARITY;

// 		if ((cmd & LRPC_DONE_PARITY) != parity) {
// 			clflushopt(m);
// 			break;
// 		}
// 		chan->new_recv_head += 2;
// 	}
// }

bool msg_recv(struct msg_chan_in *chan, uint64_t *cmd_out,
	      unsigned long *payload_out)
{
	struct lrpc_msg *m = &chan->tbl[chan->recv_head & (chan->size - 1)];
	uint64_t parity = (chan->recv_head & chan->size) ?
			  0 : LRPC_DONE_PARITY;
	uint64_t cmd;

	cmd = load_acquire(&m->cmd);
	if ((cmd & LRPC_DONE_PARITY) != parity) {
		clflushopt(m);
		clflushopt(m + CACHE_LINE_SIZE / sizeof(*m));
		prefetch(m + 2 * CACHE_LINE_SIZE / sizeof(*m));
		prefetch(m + 3 * CACHE_LINE_SIZE / sizeof(*m));
		return false;
	}
	*cmd_out = cmd & LRPC_CMD_MASK;
	*payload_out = m->payload;
	chan->recv_head++;

	store_release(chan->recv_head_wb, chan->recv_head);

	if ((chan->recv_head % (chan->size / 8)) == 0)
		clwb(chan->recv_head_wb);

	return true;
}

void run_on_core(uint64_t core) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(core, &cpuset);
        int sched_result = sched_setaffinity(0, sizeof(cpuset), &cpuset);
        BUG_ON(sched_result != 0);
}

void consumer_thread_fn(uint8_t *cxl_numa1, uint64_t num_iterations) {
	run_on_core(NUMA1_CORE);

	struct msg_chan_in chan;
	memset(&chan, 0, sizeof(chan));
	uint32_t *recv_head_wb = (uint32_t *) cxl_numa1;
	cxl_numa1 += HUGE_PAGE_SIZE;
	msg_init_in(&chan, (struct lrpc_msg *) cxl_numa1, CHAN_SIZE, recv_head_wb);

	uint64_t cmd;
	unsigned long payload;
	for (uint64_t i = 0; i < num_iterations; i++) {
		while (!msg_recv(&chan, &cmd, &payload)) {
			// log_ratelimited("recv failed\n");
			pause();
		}
		BUG_ON(payload != i);
	}
}

int main(int argc, char *argv[]) {
	if (argc != 2) {
		fprintf(stderr, "Usage: %s <num_iterations>\n", argv[0]);
		exit(1);
	}
	uint64_t num_iterations = atoll(argv[1]);
	BUG_ON(num_iterations == 0);

	run_on_core(NUMA0_CORE);

	int fd = open("/dev/dax0.0", O_RDWR);
	BUG_ON(fd < 0);
	uint8_t *cxl_numa0 = (uint8_t *) mmap(NULL, CXL_MEM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	BUG_ON(cxl_numa0 == MAP_FAILED);
	close(fd);

	memset(cxl_numa0, 0, CXL_MEM_SIZE);
	batch_clflushopt(cxl_numa0, CXL_MEM_SIZE);
	_mm_mfence();

	fd = open("/dev/dax2.0", O_RDWR);
	BUG_ON(fd < 0);
	uint8_t *cxl_numa1 = (uint8_t *) mmap(NULL, CXL_MEM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	BUG_ON(cxl_numa1 == MAP_FAILED);
	close(fd);

	memset(cxl_numa1, 0, CXL_MEM_SIZE);
	batch_clflushopt(cxl_numa1, CXL_MEM_SIZE);
	_mm_mfence();

	printf("finished zeroing CXL memory\n");

	struct msg_chan_out chan_out;
	memset(&chan_out, 0, sizeof(chan_out));
	uint32_t *recv_head_wb = (uint32_t *) cxl_numa0;
	cxl_numa0 += HUGE_PAGE_SIZE;
	msg_init_out(&chan_out, (struct lrpc_msg *) cxl_numa0, CHAN_SIZE, recv_head_wb);

	thread consumer_thread(consumer_thread_fn, cxl_numa1, num_iterations);
	sleep(1);

	uint64_t start = __rdtsc();
	for (uint64_t i = 0; i < num_iterations; i++) {
		while (!msg_send(&chan_out, 0, i)) {
			// log_ratelimited("send failed\n");
			pause();
		}
	}
	uint64_t end = __rdtsc();
	double duration_ns = (end - start) / BASE_TSC;
	double throughput = num_iterations / (duration_ns / 1e9);

	printf("throughput: %.2f Mop/s (%.2f MB/s)\n", throughput / 1e6, throughput * sizeof(struct lrpc_msg) / (1 << 20));
	consumer_thread.join();

	return 0;
}
