/*
 * thread.h - support for user-level threads
 */

#pragma once

#include <base/list.h>
#include <base/thread.h>
#include <base/types.h>
#include <base/compiler.h>
#include <runtime/preempt.h>
#include <iokernel/control.h>

struct thread;
typedef void (*thread_fn_t)(void *arg);
typedef struct thread thread_t;


/*
 * Low-level routines, these are helpful for bindings and synchronization
 * primitives.
 */

extern void thread_park_and_unlock_np(spinlock_t *l);
extern void thread_park_and_preempt_enable(void);
extern void thread_ready(thread_t *thread);
extern void thread_ready_head(thread_t *thread);
extern thread_t *thread_create(thread_fn_t fn, void *arg);
extern thread_t *thread_create_with_buf(thread_fn_t fn, void **buf, size_t len);
extern thread_t *thread_create_nostack(thread_fn_t fn, void *arg);
extern void thread_set_fsbase(thread_t *th, uint64_t fsbase);
extern void thread_free(thread_t *th);

DECLARE_PERTHREAD(thread_t *, __self);
DECLARE_PERTHREAD_ALIAS(thread_t * const, __self, __const_self);

DECLARE_PERTHREAD(unsigned int, kthread_idx);
DECLARE_PERTHREAD(uint64_t, runtime_fsbase);

static inline unsigned int get_current_affinity(void)
{
	return perthread_read(kthread_idx);
}

/**
 * thread_self - gets the currently running thread
 */
static inline thread_t *thread_self(void)
{
	return perthread_read_const_p(__const_self);
}

/*
 * Trap frame support
 */

/*
 * See the "System V Application Binary Interface" for a full explation of
 * calling and argument passing conventions.
 */

struct thread_tf {
    /* argument registers, can be clobbered by callee */
    uint64_t rdi; /* first argument */
    uint64_t rsi;
    uint64_t rdx;
    uint64_t rcx;
    uint64_t r8;
    uint64_t r9;
    uint64_t r10;
    uint64_t r11;

    /* callee-saved registers */
    uint64_t rbx;
    uint64_t rbp;
    uint64_t r12;
    uint64_t r13;
    uint64_t r14;
    uint64_t r15;

    /* special-purpose registers */
    uint64_t rax;   /* holds return value */
    uint64_t rip;   /* instruction pointer */
    uint64_t rsp;   /* stack pointer */
    uint64_t orig_rax;  /* second copy of rax, used for system call restarts */
    uint64_t fsbase; /* holds %fs */
};

#define ARG0(tf)        ((tf)->rdi)
#define ARG1(tf)        ((tf)->rsi)
#define ARG2(tf)        ((tf)->rdx)
#define ARG3(tf)        ((tf)->rcx)
#define ARG4(tf)        ((tf)->r8)
#define ARG5(tf)        ((tf)->r9)

/* format of the trap frame set up by uintr_asm_entry */
struct uintr_frame {
	struct thread_tf general_regs;
	unsigned char *xsave_area;
	unsigned long uirrv;
	unsigned long rip;
	unsigned long rflags;
	unsigned long rsp;
};

/*
 * Thread support
 */

struct stack;

struct thread {
    bool        main_thread:1;
    bool        has_fsbase:1;
    bool        thread_ready:1;
    bool        link_armed:1;
    bool        junction_thread;
    bool        thread_running;
    bool        in_syscall;
    /* modified by interrupt handler; should not be shared with other bitfields */
    bool        xsave_area_in_use:1;
    atomic8_t        interrupt_state;
    struct thread_tf    *entry_regs;
    unsigned long    junction_tstate_buf[22];
    struct stack        *stack;
    uint16_t        last_cpu;
    uint16_t        cur_kthread;
    uint64_t        ready_tsc;
    uint64_t        total_cycles;
    struct thread_tf    tf;
    struct list_node    link;
    struct list_node    interruptible_link;
#ifdef GC
    struct list_node    gc_link;
    unsigned int        onk;
#endif
};

extern uint64_t thread_get_total_cycles(thread_t *th);

/*
 * High-level routines, use this API most of the time.
 */

extern void thread_yield(void);
extern int thread_spawn(thread_fn_t fn, void *arg);
extern void thread_exit(void) __noreturn;
