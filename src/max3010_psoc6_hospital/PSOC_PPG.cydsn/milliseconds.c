/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "milliseconds.h"

#define TIMER_PERIOD_NSEC               1000UL

volatile uint32 gMilliseconds;

void MILLIS_InterruptHandler(void)
{
    /* Clear the terminal count interrupt */
    Cy_TCPWM_ClearInterrupt(Timer_HW, Timer_CNT_NUM, CY_TCPWM_INT_ON_TC);
    
    gMilliseconds++;
}

void MILLIS_AssignISR(void)
{
    Cy_SysInt_Init(&isr_1ms_cfg, MILLIS_InterruptHandler);
    NVIC_ClearPendingIRQ(isr_1ms_cfg.intrSrc);/* Clears the interrupt */
    NVIC_EnableIRQ(isr_1ms_cfg.intrSrc); /* Enable the core interrupt */
}

void MILLIS_InitAndStartTimer(void)
{
     /* Start the TCPWM component in timer/counter mode. The return value of the
     * function indicates whether the arguments are valid or not. It is not used
     * here for simplicity. */
    (void)Cy_TCPWM_Counter_Init(Timer_HW, Timer_CNT_NUM, &Timer_config);
    Cy_TCPWM_Enable_Multiple(Timer_HW, Timer_CNT_MASK); /* Enable the counter instance */
    
    /* Set the timer period in milliseconds. To count N cycles, period should be
     * set to N-1. */
    Cy_TCPWM_Counter_SetPeriod(Timer_HW, Timer_CNT_NUM, TIMER_PERIOD_NSEC - 1);
    
    /* Trigger a software reload on the counter instance. This is required when 
     * no other hardware input signal is connected to the component to act as
     * a trigger source. */
    Cy_TCPWM_TriggerReloadOrIndex(Timer_HW, Timer_CNT_MASK);     
}

void MILLIS_DisableTimer(void)
{
    Cy_TCPWM_Disable_Multiple(Timer_HW, Timer_CNT_MASK);
}

uint32_t MILLIS_GetValue(void)
{
    return gMilliseconds;
}
/* [] END OF FILE */
