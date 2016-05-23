/* ///////////////////////////////////////////////////////////////////// */
/*  File   : app_cpu1.c                                                  */
/*  Author : Chun-Jen Tsai                                               */
/*  Date   : 05/1/2016                                                   */
/* --------------------------------------------------------------------- */
/*  This program demonstrates how to use the two ARM cores of Zynq 7020  */
/*  in Asymmetric Multi-Processor (AMP) mode. In this example, Cortex A9 */
/*  core 0 runs a FreeRTOS application while Cortex A9 core 1 runs a     */
/*  bare-metal application without any OS.                               */
/* ///////////////////////////////////////////////////////////////////// */

#include <stdio.h>
#include "xparameters.h"
#include "xscugic.h"
#include "xil_mmu.h"
#include "sleep.h"

#define SHARED_TICK  (*(volatile unsigned long *)(0xFFFF0000))
#define UNSAFE_MUTEX (*(volatile unsigned long *)(0xFFFF0004))
#define SW_TRIGGER   (*(volatile unsigned long *)(0xFFFF0008))



extern u32 MMUTable;

volatile int *mutex = (int *) (XPAR_IRQ_GEN_0_BASEADDR+4);
void lock_mutex(thread_id) {
        do {
            *mutex = thread_id;
        } while (*mutex != thread_id) /* busy waiting */;
}

void unlock_mutex(thread_id) {
        *mutex = thread_id; /* set mutex to 0 */
}

void show_message(char *msg)
{
	/* Note that this critical section is not protected properly.  It only  */
	/* works if the execution frequency from all cores is not too high.     */
    lock_mutex(32);
    printf("%s\n", msg); fflush(0);
    unlock_mutex(32);
}
volatile int *reg_0 = (int *) XPAR_IRQ_GEN_0_BASEADDR;
int main()
{
    /* Disable cache on OCM, S=b1 TEX=b100 AP=b11, Domain=b1111, C=b0, B=b0 */
	Xil_SetTlbAttributes(0xFFFF0000, 0x14de2);

    show_message("CPU1: Waiting for CPU 0 to start...");
    SW_TRIGGER = 0;
    while (SW_TRIGGER == 0) /* wait for trigger signal from CPU 0 */;
    SW_TRIGGER = 0;
    //int count=0;
    *reg_0=0;
     while (1)
     {  
        if(*reg_0<6)
        {
            *reg_0++;
            sleep(1);
        }
	    /**reg_0=0;
		for(i=0;i<5;i++)
		{
		sleep(5);
			*reg_0++;
		}*/
    	//sleep(1); /* delay one second */
    	show_message("CPU1: Hello, World CPU 1.");
     }

    return 0;
}
