# MIROS.24: A Minimal Real-Time Operating System for ARM Cortex-M

This document provides an in-depth overview of the MIROS.24 project, a minimal real-time operating system (RTOS) tailored for ARM Cortex-M microcontrollers. The system is designed to facilitate understanding of RTOS fundamentals, including thread management, context switching, and scheduling.
GitHub

# Core Source Files Overview
main.c
Serves as the entry point of the application. It initializes the board support package (BSP), sets up threads, and starts the scheduler.

bsp.c / bsp.h
Implements the Board Support Package, providing hardware abstraction. It includes functions for initializing hardware components, configuring system clocks, and handling low-level operations.

miros.c / miros.h
Contains the core RTOS functionalities, including thread management, context switching, and scheduling. It defines the thread control blocks (TCBs), manages the ready queue, and implements the scheduler and context switch mechanisms.

system_TM4C123GH6PM.c
Provides system-level initialization specific to the TM4C123GH6PM microcontroller. It sets up system clocks and configures the system tick timer.

qassert.h
Defines a lightweight assertion mechanism for debugging purposes. It allows for runtime checks and aids in identifying logical errors during development.

# RTOS Architecture and Flow
🧵 Thread Definition and the OSThread Structure
In MIROS.24, the OSThread struct serves as the Thread Control Block (TCB). At present, it is a minimal structure defined in miros.h:

`
typedef struct {
    void *sp;  // Stack pointer to the top of the thread's stack
} OSThread;
`

This struct contains only a single member: a pointer to the stack (sp), which is used by the scheduler and context switching logic to manage thread execution. This minimalist TCB can be extended in the future to include more information such as:

- Thread priority
- Thread state (e.g., running, ready, blocked)
- Thread ID
- CPU usage statistics

Each thread is initialized using the function OSThread_start() defined in miros.c. The signature is:

`
void OSThread_start(
    OSThread *me,
    OSThreadHandler threadHandler,
    void *stkSto,
    uint32_t stkSize
);
`

The arguments are:

- me: A pointer to the OSThread struct (TCB) to initialize
- hreadHandler: The thread's entry function (a function pointer)
- stkSto: A pointer to the base of the statically allocated stack array
- stkSize: The number of uint32_t elements in the stack array

Inside OSThread_start(), the RTOS initializes the thread’s stack manually, setting up an initial stack frame according to the ARM Cortex-M calling convention and exception return behavior. This setup mimics the context that the CPU would expect to see when resuming from an interrupt, ensuring the thread will start executing the correct entry function when context is first switched to it.

The stack pointer (sp) member of the OSThread struct is then updated to point to the newly prepared top-of-stack, making it ready for the scheduler.

Threads are initialized and added to the OS_thread[] array, which holds pointers to all active threads. This array facilitates round-robin scheduling by maintaining the order of thread execution.

Scheduling Mechanism
MIROS.24 employs a round-robin scheduling policy. The scheduler cycles through the OS_thread[] array, selecting the next thread to run. Each thread is given a fixed time slice, after which the scheduler switches to the next thread in the array.
GeeksforGeeks

Context Switching
Context switching is triggered by the SysTick interrupt, which sets the PendSV exception. The PendSV handler performs the context switch by:

- Saving the current thread's context (registers R4-R11) onto its stack.
- Updating the current thread's stack pointer in its TCB.
- Selecting the next thread to run from the OS_thread[] array.
- Restoring the next thread's context from its stack.
- Updating the process stack pointer (PSP) to the next thread's stack.

This mechanism ensures that each thread resumes execution from where it left off.

Interrupts Utilized
SysTick: Generates periodic interrupts to trigger the scheduler.

PendSV: Handles context switching between threads.

# Detailed Mechanisms
Automated Context Switching via PendSV
The PendSV exception is set by writing to the Interrupt Control and State Register (ICSR). When PendSV is triggered:

- The current thread's context is saved.
- The scheduler selects the next thread.
- The next thread's context is restored.

This approach ensures that context switching occurs at the lowest priority, preventing interference with higher-priority interrupts.
Reddit

Round-Robin Scheduling with OS_thread[]
The OS_thread[] array holds pointers to all active threads. The scheduler maintains an index to track the current thread. Upon each scheduling event:

- The index is incremented modulo the number of threads.
- The thread at the new index is selected to run.

This simple mechanism ensures equal CPU time distribution among threads.

Register Saving and Restoration
During a context switch:

Saved Registers: R4-R11 are saved manually onto the current thread's stack.

Automatically Saved: Upon exception entry, the processor automatically saves R0-R3, R12, LR, PC, and xPSR.

Restoration: The saved registers are restored from the next thread's stack before resuming execution.

This process preserves the execution context of each thread across switches.

# Comparison with FreeRTOS
Features Not Included in MIROS.24:

Priority-Based Scheduling: MIROS.24 uses round-robin scheduling without thread priorities.

Dynamic Memory Management: No support for dynamic allocation of thread stacks or resources.

Inter-Thread Communication: Lacks mechanisms like queues, semaphores, or mutexes.

Tickless Idle Mode: Does not support low-power tickless operation.

Software Timers: No support for software-based timers or delayed task execution.

MIROS.24 focuses on simplicity and educational clarity, making it suitable for learning purposes but not for complex, real-world applications.

# Conclusion
MIROS.24 provides a foundational understanding of RTOS concepts, including thread management, context switching, and scheduling. Its minimalist design makes it an excellent educational tool for those seeking to grasp the inner workings of real-time operating systems on ARM Cortex-M microcontrollers.
