/*
 *  Do not modify this file; it is automatically 
 *  generated and any modifications will be overwritten.
 *
 * @(#) xdc-D28
 */

#include <xdc/std.h>

#include <ti/sysbios/knl/Swi.h>
extern const ti_sysbios_knl_Swi_Handle SWI_RobotControl;

#include <ti/sysbios/knl/Clock.h>
extern const ti_sysbios_knl_Clock_Handle CLK_addtime;

#include <ti/sysbios/knl/Clock.h>
extern const ti_sysbios_knl_Clock_Handle CLK_startMcBSP;

#include <ti/sysbios/hal/Hwi.h>
extern const ti_sysbios_hal_Hwi_Handle HWI8_EDMA3_0;

#include <ti/sysbios/knl/Swi.h>
extern const ti_sysbios_knl_Swi_Handle SWI_LCD;

#include <ti/sysbios/knl/Swi.h>
extern const ti_sysbios_knl_Swi_Handle SWI_vision;

#include <ti/sysbios/hal/Hwi.h>
extern const ti_sysbios_hal_Hwi_Handle HWI4_VPIF95;

#include <ti/sysbios/hal/Hwi.h>
extern const ti_sysbios_hal_Hwi_Handle HWI7_PRU6;

#include <ti/sysbios/hal/Hwi.h>
extern const ti_sysbios_hal_Hwi_Handle HWI5_EDMA3_1;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle TASK_ComWithLinux;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle TASK_LADAR;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle task2;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle task3;

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle SEM_startAstar;

extern int xdc_runtime_Startup__EXECFXN__C;

extern int xdc_runtime_Startup__RESETFXN__C;

