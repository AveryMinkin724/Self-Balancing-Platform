/*************************************
* MInimal Real-time Operating System (MIROS)
* version 0.24 (matching lesson 24)
*
* This software is a teaching aid to illustrate the concepts underlying
* a Real-Time Operating System (RTOS). The main goal of the software is
* simplicity and clear presentation of the concepts, but without dealing
* with various corner cases, portability, or error handling. For these
* reasons, the software is generally NOT intended or recommended for use
* in commercial applications.
*
* Copyright (C) 2018 Miro Samek. All Rights Reserved.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <https://www.gnu.org/licenses/>.
*
* Contact Information:
* https://www.state-machine.com
****************************************************************************/

#include <stdint.h> // C99 standard integers
#include "miros.h" 
#include "qassert.h"
#include "TM4C123GH6PM.h"

Q_DEFINE_THIS_FILE

OSThread * volatile OS_curr; /* pointer to the current thread */
OSThread * volatile OS_next; /* pointer to the next thread to run */ // make pointers themselves (not just the value they are pointig to) volatile because they will be used inside interrupts

OSThread *OS_thread[32 + 1]; /* array of threads started so ... */
uint8_t OS_threadNum; /* number of threads started so far */ // assigned to BSS section and implicitly initialized to 0
uint8_t OS_currIdx; /* current thread index for round robin */

void OS_init(void) {
    /* set the PendSV interrupt priority to the lowest level 0xFF */
    *(uint32_t volatile *)0xE000ED20 |= (0xFFU << 16); //this will show up as E0 because there are only 3 bits
		// using raw register address (rather than CMSIS name) so it can be used with multiple ARM Cortex cores (M0, M3, M4, or M7)
}

void OS_sched(void) {
		/* OS_next = ... */
		++OS_currIdx;
		if (OS_currIdx == OS_threadNum) {
				OS_currIdx = 0U;
		}
		OS_next = OS_thread[OS_currIdx];
		
		if (OS_next != OS_curr) {
				*(uint32_t volatile *)0xE000ED04 = (1U <<28); //calls pend_SVhandler, OS_curr then points to OS_next
		}
}

void OS_run(void) {
    /* callback to configure and start interrupts */
		OS_onStartup(); //callback function means its defined in application: in this case

    __asm volatile ("cpsid i");
    OS_sched(); // usually should just be called at end of interrupt, but this should return to next context and not back to OS_run so it should be finer. Q_ERROR is failsafe for this
    __asm volatile ("cpsie i");

    /* the following code should never execute */
    Q_ERROR();
}

void OSThread_start(
		OSThread *me, //pointer to the Thread Control Block
		OSThreadHandler threadHandler, // pointer-to-function to the thread handler
		void *stkSto, uint32_t stkSize) // memory & size of private thread stack
{
		/* Establish initial sp from which to build the stack frame */
		uint32_t *sp = (uint32_t *)((((uint32_t)stkSto + stkSize) / 8) *8); // stack grows hi to low, so stkSto + stkSize points to "bottom" (highest memory) of stack. ... / 8) * 8) is to align at 8 byte boundary
		uint32_t *stk_limit;
	
		*(--sp) = (1 << 24); /* xPSR */ //de-reference the pointer to the first/bottom (highest address) of the stack (decremented sp of 1 beyond top) THUMB state must be set
		*(--sp) = (uint32_t)threadHandler; /* PC */ //Return address from interrupt, set to address of the thread function 
    *(--sp) = 0x0000000EU; /* LR (14) */
		*(--sp) = 0x0000000CU; /* R12 */
		*(--sp) = 0x00000003U; /* R3 */
		*(--sp) = 0x00000002U; /* R2 */
		*(--sp) = 0x00000001U; /* R1 */
		*(--sp) = 0x00000000U; /* R0 */
		/* additionally, fake registers R4-R11 */
		*(--sp) = 0x0000000BU; /* R11 */
		*(--sp) = 0x0000000AU; /* R10 */
		*(--sp) = 0x00000009U; /* R9 */
		*(--sp) = 0x00000008U; /* R8 */	
		*(--sp) = 0x00000007U; /* R7 */
		*(--sp) = 0x00000006U; /* R6 */
		*(--sp) = 0x00000005U; /* R5 */	
		*(--sp) = 0x00000004U; /* R4 */
		
		/* save the top of the stack in the thread's attribute */
		me->sp = sp;
		
		/* round up the "bottom?" (lowest address, not really bottom of stack thats highest, but also not top, thats where sp is) of the stack to the 8-byte boundary */
		stk_limit = (uint32_t *)(((((uint32_t)stkSto - 1U) /8) + 1U) * 8);
		/* pre-fill the unused part of the stack with 0xDEADBEEF */
		for (sp = sp - 1U; sp >= stk_limit; --sp) {
			*sp = 0xDEADBEEFU;
		}
		
		Q_ASSERT(OS_threadNum < Q_DIM(OS_thread)); //assert evaluates expression, and when it is false calls a special callback function "Q_onAssert" in bsp.c
		
		/* register the thread with the OS */
		OS_thread[OS_threadNum] = me;
		++OS_threadNum;
		
}

/* inline assembly syntax for Compiler 6 (ARMCLANG) */
__attribute__ ((naked))
void PendSV_Handler(void) {
__asm volatile (
    /* __disable_irq(); */
    "  CPSID         I                 \n"

    /* if (OS_curr != (OSThread *)0) { */
    "  LDR           r1,=OS_curr       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  CMP           r1,#0             \n"
    "  BEQ           PendSV_restore    \n"

    /*     push registers r4-r11 on the stack */
#if (__ARM_ARCH == 6)               // if ARMv6-M...
    "  SUB           sp,sp,#(8*4)     \n" // make room for 8 registers r4-r11
    "  MOV           r0,sp            \n" // r0 := temporary stack pointer
    "  STMIA         r0!,{r4-r7}      \n" // save the low registers
    "  MOV           r4,r8            \n" // move the high registers to low registers...
    "  MOV           r5,r9            \n"
    "  MOV           r6,r10           \n"
    "  MOV           r7,r11           \n"
    "  STMIA         r0!,{r4-r7}      \n" // save the high registers
#else                               // ARMv7-M or higher
    "  PUSH          {r4-r11}          \n"
#endif                              // ARMv7-M or higher

    /*     OS_curr->sp = sp; */
    "  LDR           r1,=OS_curr       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  MOV           r0,sp             \n"
    "  STR           r0,[r1,#0x00]     \n"
    /* } */

    "PendSV_restore:                   \n"
    /* sp = OS_next->sp; */
    "  LDR           r1,=OS_next       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  LDR           r0,[r1,#0x00]     \n"
    "  MOV           sp,r0             \n"

    /* OS_curr = OS_next; */
    "  LDR           r1,=OS_next       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  LDR           r2,=OS_curr       \n"
    "  STR           r1,[r2,#0x00]     \n"

    /* pop registers r4-r11 */
#if (__ARM_ARCH == 6)               // if ARMv6-M...
    "  MOV           r0,sp             \n" // r0 := top of stack
    "  MOV           r2,r0             \n"
    "  ADDS          r2,r2,#(4*4)      \n" // point r2 to the 4 high registers r7-r11
    "  LDMIA         r2!,{r4-r7}       \n" // pop the 4 high registers into low registers
    "  MOV           r8,r4             \n" // move low registers into high registers
    "  MOV           r9,r5             \n"
    "  MOV           r10,r6            \n"
    "  MOV           r11,r7            \n"
    "  LDMIA         r0!,{r4-r7}       \n" // pop the low registers
    "  ADD           sp,sp,#(8*4)      \n" // remove 8 registers from the stack
#else                               // ARMv7-M or higher
    "  POP           {r4-r11}          \n"
#endif                              // ARMv7-M or higher

    /* __enable_irq(); */
    "  CPSIE         I                 \n"

    /* return to the next thread */
    "  BX            lr                \n"
    );
}
