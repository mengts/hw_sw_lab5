/* ///////////////////////////////////////////////////////////////////// */
/*  File   : app_cpu0.c                                                  */
/*  Author : Chun-Jen Tsai                                               */
/*  Date   : 05/1/2016                                                   */
/* --------------------------------------------------------------------- */
/*  This program demonstrates how to use the two ARM cores of Zynq 7020  */
/*  in Asymmetric Multi-Processor (AMP) mode. In this example, Cortex A9 */
/*  core 0 runs a FreeRTOS application while Cortex A9 core 1 runs a     */
/*  bare-metal application without any OS.                               */
/* ///////////////////////////////////////////////////////////////////// */

/* Standard include files */
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* FreeRTOS include files */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Xilinx include files */
#include "xparameters.h"
#include "xscutimer.h"
#include "xscuwdt.h"
#include "xscugic.h"
#include "xgpiops.h"

#include "xil_io.h"
#include "xil_mmu.h"
#include "xil_exception.h"
#include "xpseudo_asm.h"

/* LED control declarations */
void vSetLED(BaseType_t xValue);
void vToggleLED(void);

#define partstDIRECTION_OUTPUT	( 1 )
#define partstOUTPUT_ENABLED	( 1 )
#define partstLED_OUTPUT		( 7 )  /* 7 for ZedBoard, 10 for ZC702, 12 for MicroZed */
static XGpioPs xGpio;

#define MSEC_PER_TICK (1000/configTICK_RATE_HZ)

/* Prototypes for the FreeRTOS call-back functions defined in user files. */
void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
void vApplicationTickHook(void);

/* This function initializes the Zynq hardware. */
static void prvSetupHardware(void);
static void prvInstallISR(void (*isr)(void *), int irq_id, void *addr);
void vUserISR(void *data);
void vSetUserISR_PlusePeriod(int msec);
void vStopUserISR(void);

/* The private watchdog is used as the timer that generates run time stats.
   This frequency means it will overflow quite quickly. */
void vTaskGetRunTimeStats( char *pcWriteBuffer );
XScuWdt xWatchDogInstance;
char pcWriteBuffer[1024];

/* The interrupt controller is initialized in this file. */
XScuGic xInterruptController;

#define CPU1STARTADR 0xfffffff0
#define SHARED_TICK  (*(volatile unsigned long *)(0xFFFF0000))
#define UNSAFE_MUTEX (*(volatile unsigned long *)(0xFFFF0004))
#define SW_TRIGGER   (*(volatile unsigned long *)(0xFFFF0008))

int count=0;

/* User function prototypes. */
void  led_ctrl(void *pvParameters);
void  cpu0_task(void *pvParameters);

int main()
{
	int app_done = 0;

    /* Configure the hardware ready to run findface. */
    prvSetupHardware();

    /* Put the findface task in the taks queue. */
    xTaskCreate(cpu0_task,               /* pointer to the task function    */
    		(char *) "cpu0_task",        /* textural name of the task       */
    		configMINIMAL_STACK_SIZE,    /* stack size of the task in bytes */
    		(void *) &app_done,          /* pointer to the task parameter   */
    		tskIDLE_PRIORITY + 1,        /* priority of the task            */
    		NULL);                       /* pointer for returned task ID    */

    /* Put the LED control task in the task queue. */
    xTaskCreate(led_ctrl, (char *) "led_ctrl",
    		configMINIMAL_STACK_SIZE, (void *) &app_done, tskIDLE_PRIORITY + 1, NULL);

    /* Start running the tasks. If all is well, the scheduler will run     */
    /* forever and the function vTaskStartScheduler() will never return.   */
    vTaskStartScheduler();

    /* If the program reaches here, then there was probably insufficient   */
    /* FreeRTOS heap memory for the idle and/or timer tasks to be created. */
    return -1;  /* Return an error code to nobody! */
}
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
    lock_mutex(10);
    printf("%s\n", msg); fflush(0);
    unlock_mutex(10);
}

void cpu0_task(void *pvParameters)
{
    int *pDone = (int *) pvParameters;
    //unsigned int counter;

    /* Trigger CPU 1 to start running. */
    SW_TRIGGER = 1;

    /* Trigger the periodic user interrupts every 100 msecs. */
    show_message("CPU0: start initiating 10Hz interrupts.");
    //vSetUserISR_PlusePeriod(100);
    //counter = 0;

    while (1)
    {
    	vTaskDelay(50); /* delay half a second */
    	show_message("CPU0: Hello, World CPU 0.%d",count);
    	/*if (counter++ == 5)
    	{*/
    		/* Turn off user interrupts. */
    	/*    show_message("CPU0: No more user IRQ for CPU 0.");
    		vStopUserISR();
    	}*/
    }

    /* set app_done flag */
    *pDone = 1;

    /* The thread has ended, we must delete this task from the task queue. */
    vTaskDelete(NULL);
}

/* ----------------------------------------------------------------- */
/*  The following function initializes the ZedBoard and installs     */
/*  the interrupt table and ISR for the FreeRTOS OS kernel.          */
/* ----------------------------------------------------------------- */
static void prvSetupHardware(void)
{
    BaseType_t xStatus;
    XScuGic_Config *pxGICConfig;
    XGpioPs_Config *pxConfigPtr;

    //Disable cache on OCM
    Xil_SetTlbAttributes(0xFFFF0000,0x14de2);           // S=b1 TEX=b100 AP=b11, Domain=b1111, C=b0, B=b0
    UNSAFE_MUTEX = 0;

    /* Ensure no interrupts execute while the scheduler is in an inconsistent
       state.  Interrupts are enabled when the scheduler is started. */
    portDISABLE_INTERRUPTS();

    /* Obtain the configuration of the GIC. */
    pxGICConfig = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);

    /* Sanity check the FreeRTOSConfig.h settings matches the hardware. */
    configASSERT(pxGICConfig);
    configASSERT(pxGICConfig->CpuBaseAddress ==
        (configINTERRUPT_CONTROLLER_BASE_ADDRESS +
            configINTERRUPT_CONTROLLER_CPU_INTERFACE_OFFSET));
    configASSERT(pxGICConfig->DistBaseAddress ==
        configINTERRUPT_CONTROLLER_BASE_ADDRESS);

    /* Install a default handler for each GIC interrupt. */
    xStatus =
        XScuGic_CfgInitialize(&xInterruptController, pxGICConfig,
        pxGICConfig->CpuBaseAddress);
    configASSERT(xStatus == XST_SUCCESS);
    (void) xStatus; /* Stop compiler warning if configASSERT() is undefined. */

    /* Install an interrupt ISR for a PL user logic. */
    prvInstallISR(vUserISR, XPAR_FABRIC_IRQ_GEN_0_IRQ_INTR, (void *) XPAR_IRQ_GEN_0_BASEADDR);

    /* Initialise the LED port through GPIO driver. */
    pxConfigPtr = XGpioPs_LookupConfig(XPAR_XGPIOPS_0_DEVICE_ID);
    xStatus = XGpioPs_CfgInitialize(&xGpio, pxConfigPtr, pxConfigPtr->BaseAddr);
    configASSERT(xStatus == XST_SUCCESS);
    (void) xStatus; /* Stop compiler warning if configASSERT() is undefined. */

    /* Enable outputs and set low (initially turn-off the LED). */
    XGpioPs_SetDirectionPin(&xGpio, partstLED_OUTPUT, partstDIRECTION_OUTPUT);
    XGpioPs_SetOutputEnablePin(&xGpio, partstLED_OUTPUT, partstOUTPUT_ENABLED);
    XGpioPs_WritePin(&xGpio, partstLED_OUTPUT, 0x0);

    /* The Xilinx projects use a BSP that do not allow the start up code
       to be altered easily.  Therefore the vector table used by FreeRTOS
       is defined in FreeRTOS_asm_vectors.S, which is part of this project.
       Switch to use the FreeRTOS vector table. */
    vPortInstallFreeRTOSVectorTable();
}

/* ---------------------------------------------------------------------- */
/*  The following function is a thread that toggles LED on the ZedBoard.  */
/* --------------=------------------------------------------------------- */
void led_ctrl(void *pvParameters)
{
    TickType_t xNextWakeTime;
    int *pDone = (int *) pvParameters;

    xNextWakeTime = xTaskGetTickCount();

    while (! *pDone) /* app_done is false */
    {
        /* Wake up this task every second. */
        vTaskDelayUntil( &xNextWakeTime, configTICK_RATE_HZ);

        /*  Toggle the LED. */
        vToggleLED();
    }

    /* The thread has ended, we must delete this task from the task queue. */
    vTaskDelete(NULL);
}

/* ----------------------------------------------------------------- */
/*  The following functions are user-defined call-back routines      */
/*  for FreeRTOS. These functions are invoked by the FreeRTOS kernel */
/*  when things goes wrong.                                          */
/*                                                                   */
/*  Here, we only provide empty templates for these call-back funcs. */
/* ----------------------------------------------------------------- */

/* ----------------------------------------------------------------- */
void vApplicationMallocFailedHook(void)
{
    /*
       Called if a call to pvPortMalloc() fails because there is
       insufficient free memory available in the FreeRTOS heap.
       The size of the FreeRTOS heap is set by the configTOTAL_HEAP_SIZE
       configuration constant in FreeRTOSConfig.h.
     */
	xil_printf("STACK OVERFLOW!\n");
    taskDISABLE_INTERRUPTS();
    for (;;);
}

/* ----------------------------------------------------------------- */
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void) pcTaskName;
    (void) pxTask;

    /* Run time stack overflow checking is performed if
       configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.
       This hook function is called if a stack overflow is detected.
     */
    taskDISABLE_INTERRUPTS();
    for (;;);
}

/* ----------------------------------------------------------------- */
void vApplicationIdleHook(void)
{
    /* This is just a trivial example of an idle hook.
       It is called on each cycle of the idle task.  It must
       *NOT* block the processor.
     */
}

/* ----------------------------------------------------------------- */
void vApplicationTickHook(void)
{
	/* This function is called from the timer ISR. You can put
	   periodic maintenance operations here. However, the function
	   must be very short, not use much stack, and not call any
	   kernel API functions that don't end in "FromISR" or "FROM_ISR"
	 */
    SHARED_TICK++;
}

/* ----------------------------------------------------------------- */
void vAssertCalled(const char *pcFile, unsigned long ulLine)
{
    volatile unsigned long ul = 0;

    (void) pcFile;
    (void) ulLine;

    taskENTER_CRITICAL();
    {
        /* Set ul to a non-zero value using the debugger to step
           out of this function.
         */
        while (ul == 0)
        {
            portNOP();
        }
    }
    taskEXIT_CRITICAL();
}

/* ----------------------------------------------------------------- */
void vInitialiseTimerForRunTimeStats( void )
{
    XScuWdt_Config *pxWatchDogInstance;
    uint32_t ulValue;
    const uint32_t ulMaxDivisor = 0xff, ulDivisorShift = 0x08;

    pxWatchDogInstance = XScuWdt_LookupConfig( XPAR_SCUWDT_0_DEVICE_ID );
    XScuWdt_CfgInitialize( &xWatchDogInstance, pxWatchDogInstance, pxWatchDogInstance->BaseAddr );

    ulValue = XScuWdt_GetControlReg( &xWatchDogInstance );
    ulValue |= ulMaxDivisor << ulDivisorShift;
    XScuWdt_SetControlReg( &xWatchDogInstance, ulValue );

    XScuWdt_LoadWdt( &xWatchDogInstance, UINT_MAX );
    XScuWdt_SetTimerMode( &xWatchDogInstance );
    XScuWdt_Start( &xWatchDogInstance );
}

/* ----------------------------------------------------------------- */
/*  LED light control functions.                                     */
/* ----------------------------------------------------------------- */
void vSetLED(BaseType_t xValue)
{
    XGpioPs_WritePin(&xGpio, partstLED_OUTPUT, xValue);
}

void vToggleLED()
{
    BaseType_t xLEDState;

    xLEDState = XGpioPs_ReadPin(&xGpio, partstLED_OUTPUT);
    XGpioPs_WritePin(&xGpio, partstLED_OUTPUT, !xLEDState);
}

/* -------------------------------------------------------------------------- */
/*  Begin of the user-defined interrupt service routines.                     */
/* -------------------------------------------------------------------------- */

/* Define the address of the ICD Interrupt Clear Pending Register for irq_id */
#define ICDICPR (XPAR_PS7_SCUGIC_0_DIST_BASEADDR + XSCUGIC_PENDING_CLR_OFFSET)

//int ScuGicInterrupt_Init(void (*isr_ptr)(void), int irq_id);
volatile int *timer_base_count = (int *) XPAR_IRQ_GEN_0_BASEADDR;

void vUserISR(void *data)
{
	int irq_id = XPAR_FABRIC_IRQ_GEN_0_IRQ_INTR;//91

	/* Main content of the ISR */
	//show_message("CPU0: Got IRQ!");

    count++;
	/* Clear the pending register bit of the interrupt (write-one-to-clear) */
	*(int*)XSCUGIC_ENABLE_DISABLE_OFFSET_CALC(ICDICPR, irq_id) = 1<<(irq_id%32);
}

/*void vSetUserISR_PlusePeriod(int msec)
{
    *(u32*)(XPAR_IRQ_GEN_0_BASEADDR) = 100000*msec;//timeer_base_count
}

void vStopUserISR(void)
{
    *(u32*)(XPAR_IRQ_GEN_0_BASEADDR) = 0;
}*/

static void prvInstallISR(void (*isr)(void *), int irq_id, void *IO_BASEADDR)
{
    BaseType_t xStatus;
    int CPUID;

    /* Install the user interrupt handling routine. */
    xStatus = XScuGic_Connect(&xInterruptController, irq_id,
              (Xil_ExceptionHandler) isr, (void *) IO_BASEADDR);
	configASSERT(xStatus == XST_SUCCESS);
    (void) xStatus; /* Stop compiler warning if configASSERT() is undefined. */

	/* Set the interrupt to low-priority rising-edge triggered. */
	XScuGic_SetPriorityTriggerType(&xInterruptController, irq_id,
			portLOWEST_USABLE_INTERRUPT_PRIORITY << portPRIORITY_SHIFT, 3);

	/* Enable the interrupt device in the interrupt controller. */
	XScuGic_Enable(&xInterruptController, irq_id);

	/* Route the interrupt to CPU 0 */
	CPUID = XSCUGIC_SPI_CPU0_MASK;
    CPUID |= CPUID << 8;
    CPUID |= CPUID << 16;

    XScuGic_DistWriteReg(&xInterruptController,
    		XSCUGIC_SPI_TARGET_OFFSET_CALC(irq_id), CPUID);
}

/* -------------------------------------------------------------------------- */
/*  End of the user-defined interrupt service routines.                       */
/* -------------------------------------------------------------------------- */
