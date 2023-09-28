/*
 * interruptible_wait.c - support for interrupting blocked threads
 */

#include <runtime/sync.h>
#include <runtime/interruptible_wait.h>

#include "defs.h"

// Junction overrides these symbols
void __weak deliver_signals_jmp_thread(thread_t *th) {}