#define vPortSVCHandler sv_call_handler
#define xPortPendSVHandler pend_sv_handler
#define xPortSysTickHandler sys_tick_handler

#define configUSE_PREEMPTION 1
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configCPU_CLOCK_HZ ((unsigned long)72000000)
#define configTICK_RATE_HZ ((TickType_t)1000)
#define configMAX_PRIORITIES (5)
#define configMINIMAL_STACK_SIZE ((unsigned short)128)
#define configTOTAL_HEAP_SIZE ((size_t)(16 * 1024))
#define configMAX_TASK_NAME_LEN (16)
#define configUSE_TRACE_FACILITY 0
#define configUSE_16_BIT_TICKS 0
#define configIDLE_SHOULD_YIELD 1
#define configUSE_MUTEXES 0
#define configCHECK_FOR_STACK_OVERFLOW 0

#define configUSE_CO_ROUTINES 0
#define configMAX_CO_ROUTINE_PRIORITIES (2)

#define INCLUDE_vTaskPrioritySet 0
#define INCLUDE_uxTaskPriorityGet 0
#define INCLUDE_vTaskDelete 0
#define INCLUDE_vTaskCleanUpResources 0
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_vTaskDelayUntil 0
#define INCLUDE_vTaskDelay 1

/* This is the raw value as per the Cortex-M3 NVIC.  Values can be 255
(lowest) to 0 (1?) (highest). */
#define configKERNEL_INTERRUPT_PRIORITY 255
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
/* equivalent to 0xb0, or priority 11. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 191
