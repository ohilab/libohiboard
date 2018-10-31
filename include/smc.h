/******************************************************************************
 * Copyright (C) 2017 A. C. Open Hardware Ideas Lab
 * 
 * Authors:
 *  Matteo Pirro
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 ******************************************************************************/

/**
 * @file libohiboard/include/adc.h
 * @author Matteo Pirro
 * @brief SMC definitions and prototypes.
 */

#ifdef LIBOHIBOARD_SMC

#ifndef __SMC_H
#define __SMC_H

#include "platforms.h"
#include "errors.h"
#include "types.h"

typedef enum _Smc_PowerModeStatus
{
	SMC_POWERMODESTATUS_RUN  = 0b00000001,
	SMC_POWERMODESTATUS_STOP = 0b00000010,
	SMC_POWERMODESTATUS_VLPR = 0b00000100,
	SMC_POWERMODESTATUS_VLPW = 0b00001000,
	SMC_POWERMODESTATUS_VLPS = 0b00010000,
	SMC_POWERMODESTATUS_LLS  = 0b00100000,
	SMC_POWERMODESTATUS_VLLS = 0b01000000,
} Smc_PowerModeStatus;

typedef struct _Smc_AllowedStatus
{
	uint8_t AVLP;
	uint8_t ALLS;
	uint8_t AVLLS;
} Smc_AllowedStatus;

typedef enum _Smc_Status
{
	SMC_STATUS_RUN,
	SMC_STATUS_STOP,
	SMC_STATUS_WAIT,
	SMC_STATUS_VLPR,
	SMC_STATUS_VLPW,
	SMC_STATUS_VLPS,
	SMC_STATUS_VLLS,
	SMC_STATUS_LLS,
} Smc_Status;

typedef struct _Smc_Config
{
	Smc_AllowedStatus enabledStatus;

} Smc_Config;

typedef struct Smc_Device* Smc_DevideHandle;

extern Smc_DevideHandle OB_SMC0;


/**
 *
 */
System_Errors Smc_init (Smc_DevideHandle dev, Smc_Config *config);

/**
 *
 */
void Smc_interrupt (Smc_DevideHandle dev);

/**
 * This function return the current status of SMC.
 */
Smc_Status Smc_getActualStatus (Smc_DevideHandle dev);

/**
 *
 */
Smc_PowerModeStatus Smc_getPowerMode (Smc_DevideHandle dev);


/**
 * Configures the ARM system control register for WAIT(sleep)mode
 * and then executes the WFI instruction to enter the mode.
 *
 * Parameters:
 * none
 *
 */
void Smc_sleep (Smc_DevideHandle dev);

/**
 * Configures the ARM system control register for STOP
 * (deepsleep) mode and then executes the WFI instruction
 * to enter the mode.
 *
 * Parameters:
 * none
 *
 */
void Smc_deepsleep (Smc_DevideHandle dev);

/**
 * WAIT mode entry routine. Puts the processor into Wait mode.
 * In this mode the core clock is disabled (no code executing),
 * but bus clocks are enabled (peripheral modules are
 * operational)
 *
 * Mode transitions:
 * RUN to WAIT
 * VLPR to VLPW
 *
 * This function can be used to enter normal wait mode or VLPW
 * mode. If you are executing in normal run mode when calling
 * this function, then you will enter normal wait mode.
 * If you are in VLPR mode when calling this function,
 * then you will enter VLPW mode instead.
 *
 * NOTE: Some modules include a programmable option to disable
 * them in wait mode. If those modules are programmed to disable
 * in wait mode, they will not be able to generate interrupts to
 * wake the core.
 *
 * WAIT mode is exited using any enabled interrupt or RESET,
 * so no exit_wait routine is needed.
 * For Kinetis K:
 * If in VLPW mode, the statue of the SMC_PMCTRL[LPWUI] bit
 * determines if the processor exits to VLPR (LPWUI cleared)
 * or normal run mode (LPWUI set). The enable_lpwui()
 * and disable_lpwui()functions can be used to set this bit
 * to the desired option prior to calling enter_wait().
 * For Kinetis L:
 * LPWUI does not exist.
 * Exits with an interrupt from VLPW will always be back to VLPR.
 * Exits from an interrupt from Wait will always be back to Run.
 *
 * Parameters:
 * none
 */
System_Errors Smc_run2wait (Smc_DevideHandle dev);
System_Errors Smc_vlpr2vlpw (Smc_DevideHandle dev);

/**
 * STOP mode entry routines.
 * if in Run mode puts the processor into normal stop mode.
 * If in VLPR mode puts the processor into VLPS mode.
 * In this mode core, bus and peripheral clocks are disabled.
 *
 * Mode transitions:
 * RUN to STOP
 * VLPR to VLPS
 *
 * This function can be used to enter normal stop mode.
 * If you are executing in normal run mode when calling this
 * function and AVLP = 0, then you will enter normal stop mode.
 * If AVLP = 1 with previous write to PMPROT
 * then you will enter VLPS mode instead.
 *
 * STOP mode is exited using any enabled interrupt or RESET,
 * so no exit_stop routine is needed.
 *
 * Kinetis K:
 * when VLPS is entered directly from RUN mode,
 * exit to VLPR is disabled by hardware and the system will
 * always exit back to RUN.
 *
 * If however VLPS mode is entered from VLPR the state of
 * the LPWUI bit determines the state the MCU will return
 * to upon exit from VLPS.If LPWUI is 1 and an interrupt
 * occurs you will exit to normal run mode instead of VLPR.
 * If LPWUI is 0 and an interrupt occurs you will exit to VLPR.
 *
 * For Kinetis L:
 * when VLPS is entered from run an interrupt will exit to run.
 * When VLPS is entered from VLPR an interrupt will exit to VLPS
 *
 * Parameters:
 * none
 */
System_Errors Smc_run2stop (Smc_DevideHandle dev);
System_Errors Smc_vlpr2vlps (Smc_DevideHandle dev);
System_Errors Smc_run2vlps (Smc_DevideHandle dev);

/**
 * VLPR mode entry routine.Puts the processor into Very Low Power
 * Run Mode. In this mode, all clocks are enabled,
 * but the core, bus, and peripheral clocks are limited
 * to 2 or 4 MHz or less.
 * The flash clock is limited to 1MHz or less.
 *
 * Mode transitions:
 * RUN to VLPR
 *
 * For Kinetis K:
 * While in VLPR, VLPW or VLPS the exit to VLPR is determined by
 * the value passed in from the calling program.
 * LPWUI is static during VLPR mode and
 * should not be written to while in VLPR mode.
 *
 * For Kinetis L:
 * LPWUI does not exist. the parameter pass is a don't care
 * Exits with an interrupt from VLPW will always be back to VLPR.
 * Exits from an interrupt from Wait will always be back to Run.
 *
 * Parameters:
 * lpwui_value - The input determines what is written to the
 * LPWUI bit in the PMCTRL register
 * Clear LPWUI and interrupts keep you in VLPR
 * Set LPWUI and interrupts return you to Run mode
 * Return value : PMSTAT value or error code
 * PMSTAT = 000_0100 Current power mode is VLPR
 * ERROR Code = 0x14 - already in VLPR mode
 * = 0x24 - REGONS never clears
 * indicating stop regulation
 */
System_Errors Smc_run2vlpr (Smc_DevideHandle dev);
System_Errors Smc_vlpr2run (Smc_DevideHandle dev);



/**
 * LLS mode entry routine. Puts the processor into LLS mode from
 * normal Run mode or VLPR.
 *
 * Mode transitions:
 * RUN to LLS
 * VLPR to LLS
 *
 * Wake-up from LLS mode is controlled by the LLWU module. Most
 * modules cannot issue a wake-up interrupt in LLS mode, so make
 * sure to set up the desired wake-up sources in the LLWU before
 * calling this function.
 *
 * Parameters:
 * none
 */
System_Errors Smc_run2lls (Smc_DevideHandle dev);


///* LLS mode entry routine. Puts the processor into LLS mode from
//* normal Run mode or VLPR.
//*
//* Mode transitions:
//* RUN to LLS
//* VLPR to LLS
//*
//* Wake-up from LLS mode is controlled by the LLWU module. Most
//* modules cannot issue a wake-up interrupt in LLS mode, so make
//* sure to set up the desired wake-up sources in the LLWU before
//* calling this function.
//*
//* Parameters:
//* none
//*/
//void Smc_enter_lls(Smc_DevideHandle dev);
//
//
///* LLS mode entry routine. Puts the processor into LLS mode from
//* normal Run mode or VLPR.
//*
//* Mode transitions:
//* RUN to LLS2
//* VLPR to LLS2
//*
//* NOTE: LLS2 mode will always exit to Run mode even if you were
//* in VLPR mode before entering LLS2.
//*
//* Wake-up from LLS2 mode is controlled by the LLWU module. Most
//* modules cannot issue a wake-up interrupt in LLS mode, so make
//* sure to set up the desired wake-up sources in the LLWU before
//* calling this function.
//*
//* Parameters:
//* none
//*/
//void Smc_enter_lls2(Smc_DevideHandle dev);
//
//
///* LLS3 mode entry routine. Puts the processor into LLS3 mode from
//* normal Run mode or VLPR.
//*
//* Mode transitions:
//* RUN to LLS3
//* VLPR to LLS3
//*
//* Wake-up from LLS3 mode is controlled by the LLWU module. Most
//* modules cannot issue a wake-up interrupt in LLS3 mode, so make
//* sure to set up the desired wake-up sources in the LLWU before
//* calling this function.
//*
//* Parameters:
//* none
//*/
//void enter_lls3(Smc_DevideHandle dev);
//
///* VLLS3 mode entry routine. Puts the processor into
//* VLLS3 mode from normal Run mode or VLPR.
//*
//* Mode transitions:
//* RUN to VLLS3
//* VLPR to VLLS3
//*
//* NOTE: VLLSx modes will always exit to Run mode even if you were
//* in VLPR mode before entering VLLSx.
//*
//* Wake-up from VLLSx mode is controlled by the LLWU module. Most
//* modules cannot issue a wake-up interrupt in VLLSx mode, so make
//* sure to setup the desired wake-up sources in the LLWU before
//* calling this function.
//*
//* Parameters:
//* none
//*/
//void Smc_enter_vlls3(Smc_DevideHandle dev);
//
///* VLLS2 mode entry routine. Puts the processor into
//* VLLS2 mode from normal Run mode or VLPR.
//*
//* Mode transitions:
//* RUN to VLLS2
//* VLPR to VLLS2
//*
//* NOTE: VLLSx modes will always exit to Run mode even
//* if you were in VLPR mode before entering VLLSx.
//*
//* Wake-up from VLLSx mode is controlled by the LLWU module. Most
//* modules cannot issue a wake-up interrupt in VLLSx mode, so make
//* sure to setup the desired wake-up sources in the LLWU before
//* calling this function.
//*
//* Parameters:
//* none
//*/
//void Smc_enter_vlls2(Smc_DevideHandle dev);
//
//
///* VLLS1 mode entry routine. Puts the processor into
//* VLLS1 mode from normal Run mode or VLPR.
//*
//* Mode transitions:
//* RUN to VLLS1
//* VLPR to VLLS1
//*
//* NOTE:VLLSx modes will always exit to Run mode even if you were
//* in VLPR mode before entering VLLSx.
//*
//* Wake-up from VLLSx mode is controlled by the LLWU module. Most
//* modules cannot issue a wake-up interrupt in VLLSx mode, so make
//* sure to setup the desired wake-up sources in the LLWU before
//* calling this function.
//*
//* Parameters:
//* none
//*/
//void Smc_enter_vlls1(Smc_DevideHandle dev);
//
//
///* VLLS0 mode entry routine. Puts the processor into
//* VLLS0 mode from normal run mode or VLPR.
//*
//* Mode transitions:
//* RUN to VLLS0
//* VLPR to VLLS0
//*
//* NOTE: VLLSx modes will always exit to RUN mode even if you were
//* in VLPR mode before entering VLLSx.
//*
//* Wake-up from VLLSx mode is controlled by the LLWU module. Most
//* modules cannot issue a wake-up interrupt in VLLSx mode, so make
//* sure to setup the desired wake-up sources in the LLWU before
//* calling this function.
//*
//* Parameters:
//* PORPO_value - 0 POR detect circuit is enabled in VLLS0
//* 1 POR detect circuit is disabled in VLLS0
//*/
//void Smc_enter_vlls0(Smc_DevideHandle dev, unsigned char PORPO_value );


#endif /* __SMC_H_ */


#endif /* LIBOHIBOARD_SMC */
