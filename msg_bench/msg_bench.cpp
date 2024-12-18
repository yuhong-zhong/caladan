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

#define NUMA0_DAX "/dev/dax0.0"
#define NUMA1_DAX "/dev/dax2.0"

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

	if ((chan->recv_head % (CACHE_LINE_SIZE / sizeof(*m))) == 0) {
		for (int i = 1; i <= chan->prefetch_len; i++) {
			prefetch(&chan->tbl[(chan->recv_head + i * CACHE_LINE_SIZE / sizeof(*m)) & (chan->size - 1)]);
		}
	}

	cmd = load_acquire(&m->cmd);
	if ((cmd & LRPC_DONE_PARITY) != parity) {
		clflushopt(m);
		_mm_lfence();
		for (int i = 1; i <= chan->prefetch_len; i++)
			clflushopt(&chan->tbl[(chan->recv_head + i * CACHE_LINE_SIZE / sizeof(*m)) & (chan->size - 1)]);
		cmd = load_acquire(&m->cmd);
		chan->prefetch_len = (chan->prefetch_len <= 3) ? 1 : (chan->prefetch_len - 2);
		chan->hit_count = 0;
		if ((cmd & LRPC_DONE_PARITY) != parity) {
			clflushopt(m);
			return false;
		}
	}
	*cmd_out = cmd & LRPC_CMD_MASK;
	*payload_out = m->payload;
	chan->recv_head++;

	chan->hit_count += 1;
	if (chan->hit_count >= (chan->prefetch_len + 1) * (CACHE_LINE_SIZE / sizeof(*m))) {
		chan->prefetch_len = (chan->prefetch_len == PREFETCH_LEN) ? PREFETCH_LEN : (chan->prefetch_len + 1);
		chan->hit_count = 0;
	}

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

	for (int i = 1; i <= chan->prefetch_len; i++) {
		prefetch(&chan->tbl[(chan->recv_head + i * CACHE_LINE_SIZE / sizeof(*m)) & (chan->size - 1)]);
	}
	// prefetch(&chan->tbl[(chan->recv_head + PREFETCH_LEN * CACHE_LINE_SIZE / sizeof(*m)) & (chan->size - 1)]);

	cmd = load_acquire(&m->cmd);
	if ((cmd & LRPC_DONE_PARITY) != parity) {
		clflushopt(m);
		_mm_lfence();
		for (int i = 1; i <= chan->prefetch_len; i++)
			clflushopt(&chan->tbl[(chan->recv_head + i * CACHE_LINE_SIZE / sizeof(*m)) & (chan->size - 1)]);
		cmd = load_acquire(&m->cmd);
		chan->prefetch_len = (chan->prefetch_len <= 3) ? 1 : (chan->prefetch_len - 2);
		chan->hit_count = 0;
		if ((cmd & LRPC_DONE_PARITY) != parity) {
			clflushopt(m);
			return false;
		}
	}
	*cmd_out = cmd & LRPC_CMD_MASK;
	*payload_out = m->payload;
	chan->recv_head += LRPC_BATCH_SIZE;

	chan->hit_count += 1;
	if (chan->hit_count - 1 >= chan->prefetch_len) {
		chan->prefetch_len = (chan->prefetch_len == PREFETCH_LEN) ? PREFETCH_LEN : (chan->prefetch_len + 1);
		chan->hit_count = 0;
	}

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

#define LAT_SAMPLE_RATE (1000ul)

void consumer_thread_fn(uint8_t *cxl_numa1, uint64_t num_iterations) {
	run_on_core(NUMA1_CORE);

	struct msg_chan_in chan;
	memset(&chan, 0, sizeof(chan));
	uint32_t *recv_head_wb = (uint32_t *) cxl_numa1;
	cxl_numa1 += HUGE_PAGE_SIZE;
	msg_init_in(&chan, (struct lrpc_msg *) cxl_numa1, CHAN_SIZE, recv_head_wb);
	chan.prefetch_len = 1;

	const uint64_t num_samples = num_iterations / LAT_SAMPLE_RATE;
	uint64_t *latency_buf = (uint64_t *) aligned_alloc(PAGE_SIZE, num_samples * sizeof(uint64_t));
	BUG_ON(latency_buf == NULL);
	memset(latency_buf, 0, num_samples * sizeof(uint64_t));
	uint64_t lat_index = 0;

	uint64_t cmd;
	unsigned long payload;
	for (uint64_t i = 0; i < num_iterations; i++) {
		while (!msg_recv(&chan, &cmd, &payload)) {
			pause();
		}
		// while (!huge_msg_recv(&chan, &cmd, &payload)) {
		// 	pause();
		// }
		BUG_ON(cmd != i);
		if (i % LAT_SAMPLE_RATE == LAT_SAMPLE_RATE - 1) {
			uint64_t now = __rdtsc();
			latency_buf[lat_index++] = now - payload;
		}
	}

	sort(latency_buf, latency_buf + num_samples);
	printf("p50: %lu ns, p80: %lu ns, p90: %lu ns, p99: %lu ns\n",
	       (uint64_t) (latency_buf[(uint64_t) (num_samples / 2)] / BASE_TSC),
	       (uint64_t) (latency_buf[(uint64_t) (num_samples * 0.8)] / BASE_TSC),
	       (uint64_t) (latency_buf[(uint64_t) (num_samples * 0.9)] / BASE_TSC),
	       (uint64_t) (latency_buf[(uint64_t) (num_samples * 0.99)] / BASE_TSC));
	// for (uint64_t i = 0; i < num_samples; i++) {
	// 	printf("%lu\n", latency_buf[i]);
	// }
}

int main(int argc, char *argv[]) {
	if (argc != 3) {
		fprintf(stderr, "Usage: %s <num_iterations> <delay (ns)>\n", argv[0]);
		exit(1);
	}
	uint64_t num_iterations = atoll(argv[1]);
	BUG_ON(num_iterations == 0);
	uint64_t delay_ns = atoll(argv[2]);
	uint64_t delay_tsc = (uint64_t) delay_ns * BASE_TSC;

	run_on_core(NUMA0_CORE);

	int fd = open(NUMA0_DAX, O_RDWR);
	BUG_ON(fd < 0);
	uint8_t *cxl_numa0 = (uint8_t *) mmap(NULL, CXL_MEM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	BUG_ON(cxl_numa0 == MAP_FAILED);
	close(fd);

	memset(cxl_numa0, 0, CXL_MEM_SIZE);
	batch_clflushopt(cxl_numa0, CXL_MEM_SIZE);
	_mm_mfence();

	fd = open(NUMA1_DAX, O_RDWR);
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

	// // for (uint64_t i = 0; i < CHAN_SIZE; ++i) {
	// // 	bool sent = msg_send(&chan_out, 0, i);
	// // 	BUG_ON(!sent);
	// // }

	// // batch_clflushopt(cxl_numa0 - HUGE_PAGE_SIZE, CXL_MEM_SIZE);
	// // _mm_mfence();

	// run_on_core(NUMA1_CORE);

	// struct msg_chan_in chan_in;
	// memset(&chan_in, 0, sizeof(chan_in));
	// uint32_t *new_recv_head_wb = (uint32_t *) cxl_numa1;
	// cxl_numa1 += HUGE_PAGE_SIZE;
	// msg_init_in(&chan_in, (struct lrpc_msg *) cxl_numa1, CHAN_SIZE, new_recv_head_wb);

	// // batch_clflushopt(cxl_numa1 - HUGE_PAGE_SIZE, CXL_MEM_SIZE);
	// // _mm_mfence();

	// // volatile uint64_t tmp_cmd;
	// // for (uint64_t i = 0; i < CHAN_SIZE; ++i) {
	// // 	tmp_cmd = ACCESS_ONCE(chan_in.tbl[i].cmd);
	// // }

	// run_on_core(NUMA0_CORE);
	// for (uint64_t i = 0; i < CHAN_SIZE; ++i) {
	// 	bool sent = msg_send(&chan_out, 0, i);
	// 	BUG_ON(!sent);
	// }
	// run_on_core(NUMA1_CORE);

	// uint64_t start = __rdtsc();
	// for (uint64_t i = 0; i < CHAN_SIZE; ++i) {
	// 	uint64_t cmd;
	// 	unsigned long payload;
	// 	while (!msg_recv(&chan_in, &cmd, &payload)) {
	// 		// log_ratelimited("recv failed\n");
	// 		pause();
	// 	}
	// 	BUG_ON(payload != i);
	// }
	// uint64_t end = __rdtsc();
	// double duration_ns = (end - start) / BASE_TSC;
	// double throughput = CHAN_SIZE / (duration_ns / 1e9);
	// printf("throughput: %.2f Mop/s (%.2f MB/s)\n", throughput / 1e6, throughput * sizeof(struct lrpc_msg) / (1 << 20));

	thread consumer_thread(consumer_thread_fn, cxl_numa1, num_iterations);
	sleep(2);

	uint64_t start = __rdtsc();
	for (uint64_t i = 0; i < num_iterations; i++) {
		uint64_t now = __rdtsc();
		while (now < start + delay_tsc * i) {
			pause();
			now = __rdtsc();
		}
		while (!msg_send(&chan_out, i, now)) {
			pause();
		}
		// while (!huge_msg_send(&chan_out, i, now)) {
		// 	pause();
		// }
	}
	uint64_t end = __rdtsc();
	double duration_ns = (end - start) / BASE_TSC;
	double throughput = num_iterations / (duration_ns / 1e9);

	printf("throughput: %.2f Mop/s (%.2f MB/s)\n", throughput / 1e6, throughput * sizeof(struct lrpc_msg) / (1 << 20));
	// printf("throughput: %.2f Mop/s (%.2f MB/s)\n", throughput / 1e6, throughput * sizeof(struct batch_lrpc_msg) / (1 << 20));
	consumer_thread.join();

	return 0;
}
