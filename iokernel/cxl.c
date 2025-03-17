#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <sys/mman.h>
#include <base/mempool.h>
#include <base/assert.h>
#include <base/log.h>
#include <iokernel/shm.h>
#include "defs.h"

#define LIKELY(x) (__builtin_expect((x), 1))
#define UNLIKELY(x) (__builtin_expect((x), 0))


const char *iok_cxl_path = NULL;
uint64_t iok_cxl_size = 0;

uint8_t *cxl_buf;

spinlock_t lock;
uint64_t early_allocated;

uint8_t *client_buf_base;
uint64_t client_buf_offset;
DEFINE_BITMAP(free_client_slots, MAX_NR_CXL_CLIENTS);

void *cxl_early_alloc(uint64_t size, uint64_t alignment, uint64_t *out_cxl_offset)
{
        uint64_t offset;
        void *ptr;

        RT_BUG_ON(alignment == 0);

        spin_lock(&lock);
        log_info("cxl: early allocation of memory, size = 0x%lx, alignment = 0x%lx, allocated = 0x%lx", size, alignment, early_allocated);
        early_allocated = ROUND_UP(early_allocated, alignment);
        ptr = cxl_buf + early_allocated;
        offset = early_allocated;
        early_allocated += size;
        early_allocated = ROUND_UP(early_allocated, alignment);
        RT_BUG_ON(early_allocated > iok_cxl_size);
        spin_unlock(&lock);

        if (out_cxl_offset != NULL)
                *out_cxl_offset = offset;
        return ptr;
}

void cxl_set_client_base(uint64_t offset)
{
        spin_lock(&lock);
        client_buf_base = cxl_buf + offset;
        client_buf_offset = offset;
        RT_BUG_ON(offset % PGSIZE_2MB != 0);
        RT_BUG_ON(offset >= early_allocated);
        spin_unlock(&lock);
}

void *cxl_alloc_client(uint64_t *out_cxl_offset)
{
        int i;

        log_info("cxl: allocating shared memory for a new client");
        RT_BUG_ON(unlikely(cfg.is_secondary));

        spin_lock(&lock);
        RT_BUG_ON(client_buf_base == NULL);

        i = bitmap_find_next_set(free_client_slots, MAX_NR_CXL_CLIENTS, 0);
        RT_BUG_ON(i == MAX_NR_CXL_CLIENTS);
        bitmap_clear(free_client_slots, i);

        spin_unlock(&lock);

        memset(client_buf_base + i * CXL_CLIENT_SIZE, 0, CXL_CLIENT_SIZE);
#ifdef NO_CACHE_COHERENCE
        batch_clflushopt(client_buf_base + i * CXL_CLIENT_SIZE, CXL_CLIENT_SIZE);
        _mm_mfence();
#endif

        if (out_cxl_offset != NULL)
                *out_cxl_offset = client_buf_offset + i * CXL_CLIENT_SIZE;
        return client_buf_base + i * CXL_CLIENT_SIZE;
}

void cxl_free_client(void *ptr)
{
        int i = (int) (((uint64_t) ptr - (uint64_t) client_buf_base) / CXL_CLIENT_SIZE);
        RT_BUG_ON(i < 0 || i >= MAX_NR_CXL_CLIENTS);
        RT_BUG_ON(unlikely(cfg.is_secondary));

        log_info("cxl: freeing shared memory for a client allocated at index %d", i);

        spin_lock(&lock);
        RT_BUG_ON(bitmap_test(free_client_slots, i));
        bitmap_set(free_client_slots, i);
        spin_unlock(&lock);
}

uint64_t virt_addr_to_phys_addr(uint64_t virtual_addr) {
        int pagemap_fd = open("/proc/self/pagemap", O_RDONLY);
	RT_BUG_ON(pagemap_fd < 0);

	uint64_t virtual_pfn = virtual_addr / PGSIZE_4KB;
	size_t offset = virtual_pfn * sizeof(uint64_t);

	/*
	 * /proc/self/pagemap doc:
	 * https://www.kernel.org/doc/Documentation/vm/pagemap.txt
	 */
	uint64_t page;
	int ret = pread(pagemap_fd, &page, 8, offset);
	if (ret != 8) {
		// Unknown error
		return 0;
	}
	if ((page & (1UL << 63UL)) == 0) {
		// Not present
		return 0;
	}
	if ((page & (1UL << 62UL)) != 0) {
		// Swapped
		return 0;
	}
	if ((page & 0x7fffffffffffffUL) == 0) {
		// Unmapped
		return 0;
	}

	uint64_t physical_addr = ((page & 0x7fffffffffffffUL) * PGSIZE_4KB)
	                         + (virtual_addr % PGSIZE_4KB);
	RT_BUG_ON((physical_addr % PGSIZE_4KB) != (virtual_addr % PGSIZE_4KB));
	return physical_addr;
}

void *cxl_get_client(uint64_t cxl_offset)
{
        RT_BUG_ON(cxl_offset % PGSIZE_2MB != 0);
        RT_BUG_ON(cxl_offset + CXL_CLIENT_SIZE >= iok_cxl_size);
        RT_BUG_ON(unlikely(!cfg.is_secondary));

        return cxl_buf + cxl_offset;
}

int cxl_init(void)
{
        int fd;

        RT_BUG_ON(iok_cxl_path == NULL);
        RT_BUG_ON(iok_cxl_size == 0);
        RT_BUG_ON(iok_cxl_size % PGSIZE_2MB != 0);
        RT_BUG_ON(iok_cxl_size < INGRESS_MBUF_SHM_SIZE);

        log_info("cxl: using CXL memory, path = %s, size = 0x%lx", iok_cxl_path, iok_cxl_size);

        fd = open(iok_cxl_path, O_RDWR);
        RT_BUG_ON(fd < 0);

        cxl_buf = (uint8_t *) mmap(NULL, iok_cxl_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        RT_BUG_ON(cxl_buf == MAP_FAILED);
        RT_BUG_ON((uint64_t) cxl_buf % PGSIZE_2MB != 0);
        close(fd);

        spin_lock_init(&lock);
        early_allocated = 0;
        client_buf_base = NULL;
        bitmap_init(free_client_slots, MAX_NR_CXL_CLIENTS, true);

        return 0;
}
