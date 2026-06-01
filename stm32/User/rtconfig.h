#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

#include <stdarg.h>
#include <stddef.h>

#define RT_NAME_MAX                     8
#define RT_ALIGN_SIZE                   4
#define RT_THREAD_PRIORITY_MAX          32
#define RT_TICK_PER_SECOND              1000

#define RT_DEBUG
#define RT_USING_OVERFLOW_CHECK

#undef RT_USING_HOOK

#define IDLE_THREAD_STACK_SIZE          256

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_EVENT
#define RT_USING_MAILBOX
#define RT_USING_MESSAGEQUEUE

#undef RT_USING_HEAP
#undef RT_USING_SMALL_MEM
#undef RT_USING_MEMHEAP
#undef RT_USING_MEMPOOL

#undef RT_USING_TIMER_SOFT

#undef RT_USING_COMPONENTS_INIT

#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE       1024
#define RT_MAIN_THREAD_PRIORITY         10

#undef RT_USING_CONSOLE
#undef RT_USING_FINSH
#undef FINSH_USING_MSH
#undef FINSH_USING_SYMTAB
#undef FINSH_USING_DESCRIPTION

#endif