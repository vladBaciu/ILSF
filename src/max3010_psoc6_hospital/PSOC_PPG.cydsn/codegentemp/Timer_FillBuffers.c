/*******************************************************************************
* File Name: Timer_FillBuffers.c
* Version 1.0
*
* Description:
*  This file provides the source code to the API for the Timer_FillBuffers
*  component
*
********************************************************************************
* Copyright 2016-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "Timer_FillBuffers.h"

/** Indicates whether or not the Timer_FillBuffers has been initialized. 
*  The variable is initialized to 0 and set to 1 the first time 
*  Timer_FillBuffers_Start() is called. This allows the Component to 
*  restart without reinitialization after the first call to 
*  the Timer_FillBuffers_Start() routine.
*/
uint8_t Timer_FillBuffers_initVar = 0U;

/** The instance-specific configuration structure. This should be used in the 
*  associated Timer_FillBuffers_Init() function.
*/ 
cy_stc_tcpwm_counter_config_t const Timer_FillBuffers_config =
{
        .period = 8000UL,
        .clockPrescaler = 0UL,
        .runMode = 0UL,
        .countDirection = 0UL,
        .compareOrCapture = 2UL,
        .compare0 = 16384UL,
        .compare1 = 16384UL,
        .enableCompareSwap = false,
        .interruptSources = 1UL,
        .captureInputMode = 3UL,
        .captureInput = CY_TCPWM_INPUT_CREATOR,
        .reloadInputMode = 3UL,
        .reloadInput = CY_TCPWM_INPUT_CREATOR,
        .startInputMode = 3UL,
        .startInput = CY_TCPWM_INPUT_CREATOR,
        .stopInputMode = 3UL,
        .stopInput = CY_TCPWM_INPUT_CREATOR,
        .countInputMode = 3UL,
        .countInput = CY_TCPWM_INPUT_CREATOR,
};


/*******************************************************************************
* Function Name: Timer_FillBuffers_Start
****************************************************************************//**
*
*  Calls the Timer_FillBuffers_Init() when called the first time and enables 
*  the Timer_FillBuffers. For subsequent calls the configuration is left 
*  unchanged and the component is just enabled.
*
* \globalvars
*  \ref Timer_FillBuffers_initVar
*
*******************************************************************************/
void Timer_FillBuffers_Start(void)
{
    if (0U == Timer_FillBuffers_initVar)
    {
        (void)Cy_TCPWM_Counter_Init(Timer_FillBuffers_HW, Timer_FillBuffers_CNT_NUM, &Timer_FillBuffers_config); 

        Timer_FillBuffers_initVar = 1U;
    }

    Cy_TCPWM_Enable_Multiple(Timer_FillBuffers_HW, Timer_FillBuffers_CNT_MASK);
    
    #if (Timer_FillBuffers_INPUT_DISABLED == 7UL)
        Cy_TCPWM_TriggerStart(Timer_FillBuffers_HW, Timer_FillBuffers_CNT_MASK);
    #endif /* (Timer_FillBuffers_INPUT_DISABLED == 7UL) */    
}


/* [] END OF FILE */
