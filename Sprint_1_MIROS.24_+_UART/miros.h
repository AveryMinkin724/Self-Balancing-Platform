/*************************************
* MInimal Real-time Operating System (MIROS)
* version 0.23 (matching lesson 23)
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
#include <stdint.h>

#ifndef MIROS_H
#define MIROS_H

/* Thread Control Block Struct */
typedef struct {
		void *sp; /* stack pointer */ 
		/* ... other attributes associated with a thread */
} OSThread;

typedef void (*OSThreadHandler)(); //Pointer to a function taking no arguments (for now) and returning void

void OS_init(void);

/* transfer control to the RTOS to run the threads */
void OS_run(void);

/* callback to configure and start interrupts */
void OS_onStartup(void);
	
/* this function must ber called with interrupts DISABLED */
void OS_sched(void);
	
/* funct to fabricate register context on each thread's stack */
void OSThread_start(
		OSThread *me, //pointer to the Thread Control Block
		OSThreadHandler threadHandler, // pointer-to-function to the thread handler
		void *stkSto, uint32_t stkSize); // memory & size of private thread stack

#endif /* MIROS */