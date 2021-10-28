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
#ifndef TLC5925_H_
#define TLC5925_H_
    
#include <stdint.h>
    

void TLC5925_ShiftOut(uint16_t ulBitOrder, uint16_t ulVal);   
void TLC5925_enableRed(void);
void TLC5925_enableGreen(void);
void TLC5925_enableIR(void);
void TLC5925_SetCurrent_mA(uint16_t val);

#endif /* TLC5925_H_ */    
/* [] END OF FILE */
