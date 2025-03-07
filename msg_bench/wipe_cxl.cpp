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

#define CXL_DAX "/dev/dax0.0"

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


int main(int argc, char *argv[]) {
	if (argc != 2) {
		fprintf(stderr, "Usage: %s <CXL size (GB)>\n", argv[0]);
		exit(1);
	}
        uint64_t cxl_size = atoll(argv[1]) << 30ul;

	int fd = open(CXL_DAX, O_RDWR);
	BUG_ON(fd < 0);
	uint8_t *cxl_buf = (uint8_t *) mmap(NULL, cxl_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	BUG_ON(cxl_buf == MAP_FAILED);
	close(fd);

        memset(cxl_buf, 0, cxl_size);
        batch_clflushopt(cxl_buf, cxl_size);

	return 0;
}
