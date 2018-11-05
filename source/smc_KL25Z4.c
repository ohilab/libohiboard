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
 * @file libohiboard/source/smc.c
 * @author Matteo Pirro
 * @brief SMC implementations for KL25Z4.
 */
 
#ifdef LIBOHIBOARD_SMC

#if defined (LIBOHIBOARD_KL15Z4) || \
    defined (LIBOHIBOARD_KL25Z4)

// https://github.com/bingdo/FRDM-KL25Z-WIZ550io/blob/master/system/src/kl25-sc/smc.c

// Entered only from Run mode
// The Clock Monitor(s) in MCG must be disabled
// The slow IRC clock must not be enabled
// Select either BLPI mode derived from Fast IRC or BLPE mode from external source
// Set SIM_ClkDIV so that core clock < 4 MHz, Bus < 4 MHz, flash clock < 1 MHz
// AVLP bit in PMPROT register must = 1
// Run Mode Selection (RUNM) bits must be set to 'b10
// Optional wait for PMC Regulator to change to Stop Regulation Mode (REGONS) = 0
// [MC2] SMC_PMSTAT will be set to 'b000_0100

#include "smc.h"

typedef struct Smc_Device {

    SMC_MemMapPtr regMap;
    PMC_MemMapPtr regMapPMC;

    Smc_AllowedStatus enabledStatus;
    Smc_Status actualStatus;
    Smc_Status lastStatus;

    uint8_t devInitialized;   /**< Indicate that device was been initialized. */
} Smc_Device;

static Smc_Device smc0 = {
        .regMap    = SMC_BASE_PTR,
        .regMapPMC = PMC_BASE_PTR,
};

Smc_DevideHandle OB_SMC0 = &smc0;

System_Errors Smc_init (Smc_DevideHandle dev, Smc_Config *config)
{
    uint8_t enabledStatus = 0;

    dev->enabledStatus = config->enabledStatus;

    if (dev->enabledStatus.AVLP)
        enabledStatus |= SMC_PMPROT_AVLP_MASK;

    if (dev->enabledStatus.ALLS)
        enabledStatus |= SMC_PMPROT_ALLS_MASK;

    if (dev->enabledStatus.ALLS)
        enabledStatus |= SMC_PMPROT_AVLLS_MASK;

    SMC_PMPROT = enabledStatus;

    dev->actualStatus = SMC_STATUS_RUN;

    return ERRORS_NO_ERROR;
}

void Smc_interrupt (Smc_DevideHandle dev)
{
    switch (dev->actualStatus)
    {
    case SMC_STATUS_WAIT:
        dev->actualStatus = SMC_STATUS_RUN;
        dev->lastStatus   = SMC_STATUS_WAIT;
        break;
    case SMC_STATUS_STOP:
        dev->actualStatus = SMC_STATUS_RUN;
        dev->lastStatus   = SMC_STATUS_STOP;
        break;
    case SMC_STATUS_VLPW:
        dev->actualStatus = SMC_STATUS_VLPR;
        dev->lastStatus   = SMC_STATUS_VLPW;
        break;
    case SMC_STATUS_VLPS:
        dev->actualStatus = dev->lastStatus;
        dev->lastStatus   = SMC_STATUS_VLPS;
        break;
    case SMC_STATUS_LLS:
        dev->actualStatus = dev->lastStatus;
        dev->lastStatus = SMC_STATUS_LLS;
        break;
    }
}

Smc_Status Smc_getActualStatus (Smc_DevideHandle dev)
{
    return dev->actualStatus;
}

Smc_PowerModeStatus Smc_getPowerMode (Smc_DevideHandle dev)
{
    return SMC_PMSTAT_REG(dev->regMap);
}

void Smc_sleep (Smc_DevideHandle dev)
{
    // Clear the SLEEPDEEP bit to make sure we go into WAIT (sleep)
    // mode instead of deep sleep.
    // This define is into core_cm0plus.h files
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    // WFI instruction will start entry into WAIT mode
    __asm("WFI");
}

void Smc_deepsleep (Smc_DevideHandle dev)
{
    // Set the SLEEPDEEP bit to enable deep sleep mode (STOP)
    // This define is into core_cm0plus.h files
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // WFI instruction will start entry into STOP mode
    __asm("WFI");
}

System_Errors Smc_run2wait (Smc_DevideHandle dev)
{
    if (dev->actualStatus == SMC_STATUS_RUN)
    {
        dev->lastStatus = SMC_STATUS_RUN;
        dev->actualStatus = SMC_STATUS_WAIT;
        Smc_sleep(dev);

        return ERRORS_SMC_NO_ERROR;
    }
    else
    {
        return ERRORS_SMC_STATUS_NOT_ALLOWED;
    }
}

System_Errors Smc_vlpr2vlpw (Smc_DevideHandle dev)
{
    if (dev->actualStatus == SMC_STATUS_VLPR)
    {
        dev->lastStatus = SMC_STATUS_VLPR;
        dev->actualStatus = SMC_STATUS_VLPW;
        Smc_sleep(dev);

        return ERRORS_SMC_NO_ERROR;
    }
    else
    {
        return ERRORS_SMC_STATUS_NOT_ALLOWED;
    }
}

System_Errors Smc_run2stop (Smc_DevideHandle dev)
{
    if (dev->actualStatus == SMC_STATUS_RUN)
    {
        dev->lastStatus = SMC_STATUS_RUN;
        dev->actualStatus = SMC_STATUS_STOP;
        volatile unsigned int dummyread;
        // The PMPROT register may have already been written by init code.
        // Set the STOPM field to 0b000 for normal STOP mode
        // For Kinetis L: if trying to enter Stop from VLPR user
        // forced to VLPS low power mode
        SMC_PMCTRL_REG(dev->regMap) &= ~SMC_PMCTRL_STOPM_MASK;
        SMC_PMCTRL_REG(dev->regMap) |= SMC_PMCTRL_STOPM(0);
        // wait for write to complete to SMC before stopping core
        dummyread = SMC_PMCTRL_REG(dev->regMap);
        Smc_deepsleep(dev);

        return ERRORS_SMC_NO_ERROR;
    }
    else
    {
        return ERRORS_SMC_STATUS_NOT_ALLOWED;
    }
}

System_Errors Smc_vlpr2vlps (Smc_DevideHandle dev)
{
    if (dev->actualStatus == SMC_STATUS_VLPR)
    {
        if (dev->enabledStatus.AVLP == 1)
        {
            dev->lastStatus = SMC_STATUS_VLPR;
            dev->actualStatus = SMC_STATUS_VLPS;
            volatile unsigned int dummyread;
            // The PMPROT register may have already been written by init code.
            // Set the STOPM field to 0b000 for normal STOP mode
            // For Kinetis L: if trying to enter Stop from VLPR user
            // forced to VLPS low power mode
            SMC_PMCTRL_REG(dev->regMap) &= ~SMC_PMCTRL_STOPM_MASK;
            SMC_PMCTRL_REG(dev->regMap) |= SMC_PMCTRL_STOPM(0x2);
            // wait for write to complete to SMC before stopping core
            dummyread = SMC_PMCTRL_REG(dev->regMap);
            Smc_deepsleep(dev);
        }
        else
        {
            return ERRORS_SMC_STATUS_NOT_ENABLED;
        }
    }
    else
    {
        return ERRORS_SMC_STATUS_NOT_ALLOWED;
    }
}

System_Errors Smc_run2vlps (Smc_DevideHandle dev)
{
    if (dev->actualStatus == SMC_STATUS_RUN)
    {
        if (dev->enabledStatus.AVLP == 1)
        {
            dev->lastStatus = SMC_STATUS_RUN;
            dev->actualStatus = SMC_STATUS_VLPS;
            volatile unsigned int dummyread;
            // The PMPROT register may have already been written by init code.
            // Set the STOPM field to 0b000 for normal STOP mode
            // For Kinetis L: if trying to enter Stop from VLPR user
            // forced to VLPS low power mode */
            SMC_PMCTRL_REG(dev->regMap) &= ~SMC_PMCTRL_STOPM_MASK;
            SMC_PMCTRL_REG(dev->regMap) |= SMC_PMCTRL_STOPM(0x2);
            /*wait for write to complete to SMC before stopping core */
            dummyread = SMC_PMCTRL_REG(dev->regMap);
            Smc_deepsleep(dev);
        }
        else
        {
            return ERRORS_SMC_STATUS_NOT_ENABLED;
        }
    }
    else
    {
        return ERRORS_SMC_STATUS_NOT_ALLOWED;
    }
}

System_Errors Smc_run2vlpr (Smc_DevideHandle dev)
{
    if (dev->actualStatus == SMC_STATUS_RUN)
    {
        if (dev->enabledStatus.AVLP == 1)
        {
            dev->lastStatus = SMC_STATUS_RUN;
            dev->actualStatus = SMC_STATUS_VLPR;

            // Am I already in VLPR?
            if ((SMC_PMSTAT_REG(dev->regMap) & SMC_PMSTAT_PMSTAT_MASK) == 0x04)
            {
                return ERRORS_SMC_STATUS_ALREADY_SET;
            }

            // Set the (for MC1)LPLLSM or (for MC2)STOPM field
            // to 0b010 for VLPS mode -
            // and RUNM bits to 0b010 for VLPR mode
            // Need to set state of LPWUI bit
            SMC_PMCTRL_REG(dev->regMap) &= ~SMC_PMCTRL_RUNM_MASK;
            SMC_PMCTRL_REG(dev->regMap) |= SMC_PMCTRL_RUNM(0x2);
            for (int i = 0 ; i < 10000 ; i++)
            {
                // Check that the value of REGONS bit is not 0.
                // When it is a zero, you can stop checking
                if ((PMC_REGSC_REG(dev->regMapPMC) & PMC_REGSC_REGONS_MASK) == 0x04)
                {
                    // 0 Regulator is in stop regulation or in transition to/from it
                    // 1 MCU is in Run regulation mode
                }
                else break;
            }
            if ((PMC_REGSC_REG(dev->regMapPMC) & PMC_REGSC_REGONS_MASK) == 0x04)
            {
                return ERRORS_SMC_ERROR;
            }
        }
        else
        {
            return ERRORS_SMC_STATUS_NOT_ENABLED;
        }
    }
    else
    {
        return ERRORS_SMC_STATUS_NOT_ALLOWED;
    }
}

System_Errors Smc_vlpr2run (Smc_DevideHandle dev)
{
    if (dev->actualStatus == SMC_STATUS_VLPR)
    {
        dev->lastStatus = SMC_STATUS_VLPR;
        dev->actualStatus = SMC_STATUS_RUN;
        volatile unsigned int dummyread;
        SMC_PMCTRL_REG(dev->regMap) &= ~SMC_PMCTRL_RUNM_MASK;
        SMC_PMCTRL_REG(dev->regMap) |= SMC_PMCTRL_RUNM(0x0);
        dummyread = SMC_PMCTRL_REG(dev->regMap);

        return ERRORS_SMC_NO_ERROR;
    }
    else
    {
        return ERRORS_SMC_STATUS_NOT_ALLOWED;
    }
}

System_Errors Smc_run2lls (Smc_DevideHandle dev)
{
    if (dev->actualStatus == SMC_STATUS_RUN)
    {
        dev->lastStatus = SMC_STATUS_RUN;
        dev->actualStatus = SMC_STATUS_LLS;
        volatile unsigned int dummyread;
        SMC_PMCTRL_REG(dev->regMap) &= ~SMC_PMCTRL_STOPM_MASK;
        SMC_PMCTRL_REG(dev->regMap) |= SMC_PMCTRL_STOPM(0x3);
        dummyread = SMC_PMCTRL_REG(dev->regMap);
        Smc_deepsleep(dev);

        return ERRORS_SMC_NO_ERROR;
    }
    else
    {
        return ERRORS_SMC_STATUS_NOT_ALLOWED;
    }
}


//void Smc_enter_lls(Smc_DevideHandle dev)
//{
//	volatile unsigned int dummyread;
//	/* Write to PMPROT to allow LLS power modes this write-once
//	bit allows the MCU to enter the LLS low power mode*/
//	SMC_PMPROT = SMC_PMPROT_ALLS_MASK;
//	/* Set the (for MC1) LPLLSM or
//	(for MC2)STOPM field to 0b011 for LLS mode
//	Retains LPWUI and RUNM values */
//	SMC_PMCTRL &= ~SMC_PMCTRL_STOPM_MASK ;
//	SMC_PMCTRL |= SMC_PMCTRL_STOPM(0x3) ;
//	/* set LLSM = 0b00 in SMC_VLLSCTRL (for MC4) */
//	//SMC_VLLSCTRL = SMC_VLLSCTRL_LLSM(0);
//	/*wait for write to complete to SMC before stopping core */
//	dummyread = SMC_PMCTRL;
//	/* Now execute the stop instruction to go into LLS */
//	Smc_deepsleep(dev);
//}
//
//
//void Smc_enter_lls2(Smc_DevideHandle dev)
//{
//	volatile unsigned int dummyread;
//	/* Write to PMPROT to allow LLS power modes this write-once
//	bit allows the MCU to enter the LLS low power mode*/
//	SMC_PMPROT = SMC_PMPROT_ALLS_MASK;
//	/* Set the (for MC1) LPLLSM or
//	(for MC2)STOPM field to 0b011 for LLS3 mode
//	Retains LPWUI and RUNM values */
//	SMC_PMCTRL &= ~SMC_PMCTRL_STOPM_MASK ;
//	SMC_PMCTRL |= SMC_PMCTRL_STOPM(0x3) ;
//	/* set LLSM = 0b10 in SMC_VLLSCTRL (for MC4) */
//	//SMC_VLLSCTRL = SMC_VLLSCTRL_LLSM(2);
//	/*wait for write to complete to SMC before stopping core */
//	dummyread = SMC_PMCTRL;
//	/* Now execute the stop instruction to go into LLS */
//	Smc_deepsleep(dev);
//}
//
//void enter_lls3(Smc_DevideHandle dev)
//{
//	volatile unsigned int dummyread;
//	/* Write to PMPROT to allow LLS power modes this write-once
//	bit allows the MCU to enter the LLS low power mode*/
//	SMC_PMPROT = SMC_PMPROT_ALLS_MASK;
//	/* Set the (for MC1) LPLLSM or
//	(for MC2)STOPM field to 0b011 for LLS3 mode
//	Retains LPWUI and RUNM values */
//	SMC_PMCTRL &= ~SMC_PMCTRL_STOPM_MASK ;
//	SMC_PMCTRL |= SMC_PMCTRL_STOPM(0x3) ;
//	/* set LLSM = 0b11 in SMC_VLLSCTRL (for MC4) */
//	//SMC_VLLSCTRL = SMC_VLLSCTRL_LLSM(3);
//	/*wait for write to complete to SMC before stopping core */
//	dummyread = SMC_PMCTRL;
//	/* Now execute the stop instruction to go into LLS */
//	Smc_deepsleep(dev);
//}
//
//
//void Smc_enter_vlls3(Smc_DevideHandle dev)
//{
//	volatile unsigned int dummyread;
//	/* Write to PMPROT to allow VLLS3 power modes */
//	SMC_PMPROT = SMC_PMPROT_AVLLS_MASK;
//	/* Set the VLLSM field to 0b100 for VLLSx(for MC1)
//	or STOPM field to 0b100 for VLLSx (for MC2)
//	- Retain state of LPWUI and RUNM */
//	SMC_PMCTRL &= ~SMC_PMCTRL_STOPM_MASK ;
//	SMC_PMCTRL |= SMC_PMCTRL_STOPM(0x4) ;
//	// MC_PMCTRL &= ~MC_PMCTRL_VLLSM_MASK ; //(for MC1)
//	// MC_PMCTRL |= MC_PMCTRL_VLLSM(0x4) ; //(for MC1)
//	/* set VLLSM = 0b11 in SMC_VLLSCTRL (for MC2) */
//	//SMC_VLLSCTRL = SMC_VLLSCTRL_VLLSM(3);
//	SMC_STOPCTRL = SMC_STOPCTRL_VLLSM(3);
//	/*wait for write to complete to SMC before stopping core */
//	//dummyread = SMC_VLLSCTRL;
//	dummyread = SMC_STOPCTRL;
//	/* Now execute the stop instruction to go into VLLS3 */
//	Smc_deepsleep(dev);
//}
//
//void Smc_enter_vlls2(Smc_DevideHandle dev)
//{
//	volatile unsigned int dummyread;
//	/* Write to PMPROT to allow VLLS2 power modes */
//	SMC_PMPROT = SMC_PMPROT_AVLLS_MASK;
//	/* Set the VLLSM field to 0b100 for VLLSx(for MC1)
//	or STOPM field to 0b100 for VLLSx (for MC2)
//	- Retain state of LPWUI and RUNM */
//	SMC_PMCTRL &= ~SMC_PMCTRL_STOPM_MASK ;
//	SMC_PMCTRL |= SMC_PMCTRL_STOPM(0x4) ;
//	// MC_PMCTRL &= ~MC_PMCTRL_VLLSM_MASK ;
//	// MC_PMCTRL |= MC_PMCTRL_VLLSM(0x4) ;
//	/* set VLLSM = 0b10 in SMC_VLLSCTRL (for MC2) */
//	//SMC_VLLSCTRL = SMC_VLLSCTRL_VLLSM(2);
//	SMC_STOPCTRL = SMC_STOPCTRL_VLLSM(2);
//	/*wait for write to complete to SMC before stopping core */
//	//dummyread = SMC_VLLSCTRL;
//	dummyread = SMC_STOPCTRL;
//	/* Now execute the stop instruction to go into VLLS2 */
//	Smc_deepsleep(dev);
//}
//
//void Smc_enter_vlls1(Smc_DevideHandle dev)
//{
//	volatile unsigned int dummyread;
//	/* Write to PMPROT to allow all possible power modes */
//	SMC_PMPROT = SMC_PMPROT_AVLLS_MASK;
//	/* Set the VLLSM field to 0b100 for VLLSx(for MC1)
//	or STOPM field to 0b100 for VLLSx (for MC2)
//	- Retain state of LPWUI and RUNM */
//	SMC_PMCTRL &= ~SMC_PMCTRL_STOPM_MASK ;
//	SMC_PMCTRL |= SMC_PMCTRL_STOPM(0x4) ;
//	// MC_PMCTRL &= ~MC_PMCTRL_VLLSM_MASK ;
//	// MC_PMCTRL |= MC_PMCTRL_VLLSM(0x4) ;
//	/* set VLLSM = 0b01 in SMC_VLLSCTRL (for MC2) */
//	//SMC_VLLSCTRL = SMC_VLLSCTRL_VLLSM(1);
//	SMC_STOPCTRL = SMC_STOPCTRL_VLLSM(1);
//	/*wait for write to complete to SMC before stopping core */
//	//dummyread = SMC_VLLSCTRL;
//	dummyread = SMC_STOPCTRL;
//	/* Now execute the stop instruction to go into VLLS1 */
//	Smc_deepsleep(dev);
//}
//
//
//void Smc_enter_vlls0(Smc_DevideHandle dev, unsigned char PORPO_value )
//{
//	volatile unsigned int dummyread;
//	/* Write to PMPROT to allow all possible power modes */
//	SMC_PMPROT = SMC_PMPROT_AVLLS_MASK;
//	/* Set the STOPM field to 0b100 for VLLS0 mode */
//	SMC_PMCTRL &= ~SMC_PMCTRL_STOPM_MASK;
//	SMC_PMCTRL |= SMC_PMCTRL_STOPM(0x4);
//	/* set VLLSM = 0b00 */
//	//SMC_VLLSCTRL = (PORPO_value <<SMC_VLLSCTRL_PORPO_SHIFT)	| SMC_VLLSCTRL_VLLSM(3);
//	SMC_STOPCTRL &= ~SMC_STOPCTRL_VLLSM_MASK;
//	SMC_STOPCTRL = SMC_STOPCTRL_VLLSM(0) | SMC_STOPCTRL_PORPO_MASK;
//	/*wait for write to complete to SMC before stopping core */
//	//dummyread = SMC_VLLSCTRL;
//	dummyread = SMC_STOPCTRL;
//	Smc_deepsleep(dev);
//}

#endif // LIBOHIBOARD_KL15Z4 || LIBOHIBOARD_KL25Z4

#endif
