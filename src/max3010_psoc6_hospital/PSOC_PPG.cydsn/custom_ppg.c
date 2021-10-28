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
#include "TLC5925.h"
#include <stdio.h>
/* @brief Just a redefinition */
#define TRUE                      true
#define FALSE                     false

#define TIMER_1_PERIOD_NSEC               1000UL
#define TIMER_FILLBUFFERS_PERIOD_NSEC     8000UL
#define BUFFER_LENGTH                    (400U)

#define ADC_CHANNEL_0_INV_AMP               0

volatile static bool bToggleFunctions = FALSE;
volatile static uint16_t bufferRed[BUFFER_LENGTH];
volatile static uint16_t bufferIR[BUFFER_LENGTH];
volatile static uint16_t bufferHead;

void (*pEnableFunctions[2])() = {TLC5925_enableRed, TLC5925_enableIR};

void CUSTOM_PPG_InterruptHandler_Spo2(void)
{
    /* Clear the terminal count interrupt */
    Cy_TCPWM_ClearInterrupt(Timer_1_HW, Timer_1_CNT_NUM, CY_TCPWM_INT_ON_TC);
    (*pEnableFunctions[bToggleFunctions])();
    
    bToggleFunctions ^= bToggleFunctions;
}

void CUSTOM_PPG_AssignISR_Spo2(void)
{
    Cy_SysInt_Init(&spo2_isr_cfg, CUSTOM_PPG_InterruptHandler_Spo2);
    NVIC_ClearPendingIRQ(spo2_isr_cfg.intrSrc);/* Clears the interrupt */
    NVIC_EnableIRQ(spo2_isr_cfg.intrSrc); /* Enable the core interrupt */
}

void CUSTOM_PPG_InitAndStartTimer_Spo2(void)
{
     /* Start the TCPWM component in timer/counter mode. The return value of the
     * function indicates whether the arguments are valid or not. It is not used
     * here for simplicity. */
    (void)Cy_TCPWM_Counter_Init(Timer_1_HW, Timer_1_CNT_NUM, &Timer_1_config);
    Cy_TCPWM_Enable_Multiple(Timer_1_HW, Timer_1_CNT_MASK); /* Enable the counter instance */
    
    /* Set the timer period in milliseconds. To count N cycles, period should be
     * set to N-1. */
    Cy_TCPWM_Counter_SetPeriod(Timer_1_HW, Timer_1_CNT_NUM, TIMER_1_PERIOD_NSEC - 1);
    
    /* Trigger a software reload on the counter instance. This is required when 
     * no other hardware input signal is connected to the component to act as
     * a trigger source. */
    Cy_TCPWM_TriggerReloadOrIndex(Timer_1_HW, Timer_1_CNT_MASK);     
}


void ADC_INVERTING_AMP_ISR_Callback(void)
{
    if(BUFFER_LENGTH > bufferHead)
    {
        TLC5925_enableRed();
        bufferRed[bufferHead++] =  ADC_INVERTING_AMP_GetResult16(ADC_CHANNEL_0_INV_AMP);
        TLC5925_enableIR();
        bufferRed[bufferHead++] =  ADC_INVERTING_AMP_GetResult16(ADC_CHANNEL_0_INV_AMP);
    }
}

void printadcval(void)
{
    if(BUFFER_LENGTH == bufferHead)
    {
        for(uint16_t i = 0; i< BUFFER_LENGTH; i++)
            printf("%lu \r\n", bufferRed[i]); 
    }
 }
//void CUSTOM_PPG_AssignISR_FillBuff(void)
//{
//    Cy_SysInt_Init(&fillBuff_isr_cfg, CUSTOM_PPG_InterruptHandler_FillBuff);
//    NVIC_ClearPendingIRQ(fillBuff_isr_cfg.intrSrc);/* Clears the interrupt */
//    NVIC_EnableIRQ(fillBuff_isr_cfg.intrSrc); /* Enable the core interrupt */
//}
//
//void CUSTOM_PPG_InitAndStartTimer_FillBuff(void)
//{
//     /* Start the TCPWM component in timer/counter mode. The return value of the
//     * function indicates whether the arguments are valid or not. It is not used
//     * here for simplicity. */
//    (void)Cy_TCPWM_Counter_Init(Timer_FillBuffers_HW, Timer_FillBuffers_CNT_NUM, &Timer_FillBuffers_config);
//    Cy_TCPWM_Enable_Multiple(Timer_FillBuffers_HW, Timer_FillBuffers_CNT_MASK); /* Enable the counter instance */
//    
//    /* Set the timer period in milliseconds. To count N cycles, period should be
//     * set to N-1. */
//    Cy_TCPWM_Counter_SetPeriod(Timer_FillBuffers_HW, Timer_FillBuffers_CNT_NUM, TIMER_FILLBUFFERS_PERIOD_NSEC - 1);
//    
//    /* Trigger a software reload on the counter instance. This is required when 
//     * no other hardware input signal is connected to the component to act as
//     * a trigger source. */
//    Cy_TCPWM_TriggerReloadOrIndex(Timer_FillBuffers_HW, Timer_FillBuffers_CNT_MASK);     
//}
//
//void CUSTOM_PPG_FillBuffers(void)
//{
//   
//    
//    
//    TLC5925_enableIR();
//    
//}

/* [] END OF FILE */
