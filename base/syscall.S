
#include <asm/unistd_64.h>

.section        .note.GNU-stack,"",@progbits
.text
.globl base_syscall_start
base_syscall_start:

.align 16
.globl syscall_write
.type syscall_write, @function
syscall_write:
	movl	$__NR_write, %eax
	syscall
	ret

.align 16
.globl syscall_mmap
syscall_mmap:
	movl    $__NR_mmap, %eax
	movq    %rcx, %r10
	syscall
	ret

.align 16
.globl syscall_mbind
syscall_mbind:
	movl    $__NR_mbind, %eax
	movq    %rcx, %r10
	syscall
	ret

.align 16
.globl syscall_ioctl
syscall_ioctl:
	movl    $__NR_ioctl, %eax
	syscall
	ret

.align 16
.globl syscall_rt_sigreturn
syscall_rt_sigreturn:
	movl    $__NR_rt_sigreturn, %eax
	syscall
	ret

.align 16
.globl syscall_madvise
syscall_madvise:
	movl    $__NR_madvise, %eax
	syscall
	ret

.align 16
.globl syscall_mprotect
syscall_mprotect:
	movl    $__NR_mprotect, %eax
	syscall
	ret

.align 16
.globl syscall_exit
syscall_exit:
	movl    $__NR_exit_group, %eax
	syscall

.globl base_syscall_end
base_syscall_end:
