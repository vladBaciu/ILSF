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
#ifndef CUSTOM_PPG_H_
#define CUSTOM_PPG_H_
    
#include <stdint.h>
    

void CUSTOM_PPG_AssignISR_Spo2(void);
void CUSTOM_PPG_InterruptHandler_Spo2(void);
void CUSTOM_PPG_InitAndStartTimer_Spo2(void);
void CUSTOM_PPG_AssignISR_FillBuff(void);
void CUSTOM_PPG_InitAndStartTimer_FillBuff(void);
void CUSTOM_PPG_InterruptHandler_FillBuff(void);
void printadcval(void);
#endif /* CUSTOM_PPG_H_ */    
/* [] END OF FILE */
