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
#ifndef AD5273_H_
#define AD5273_H_
    
#include <stdint.h>

uint8_t AD5273_ReadData(void);
uint8_t AD5273_WriteData(uint8_t val);
void AD5273_SetWiper(uint32_t resVal);

#endif /* AD5273_H_ */
/* [] END OF FILE */
