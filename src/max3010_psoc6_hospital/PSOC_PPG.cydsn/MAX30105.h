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
#include <stdint.h>
#include <stdbool.h>

uint32_t MAX30105_ReadRegister8(uint8_t *rxBuffer, uint8_t reg);
uint32_t MAX30105_WriteRegister8(uint8_t txBuffer, uint8_t reg);
uint32_t MAX30105_WriteWithBitMask(uint8_t reg, uint8_t mask, uint8_t val);
uint8_t MAX30105_GetWritePointer(void);
uint8_t MAX30105_GetReadPointer(void);
bool MAX30105_SafeCheck(uint8_t maxTimeToCheck);
uint16_t MAX30105_Check(void);
uint32_t MAX30105_GetRed(void);
uint32_t MAX30105_GetIR(void);
uint32_t MAX30105_GetGreen(void);
uint32_t MAX30105_GetFIFORed(void);
uint32_t MAX30105_GetFIFOIR(void);
uint32_t MAX30105_GetFIFOGreen(void);
uint8_t MAX30105_Available(void);
void MAX30105_NextSample(void);
void MAX30105_SetFIFOAverage(uint8_t numberOfSamples);
void MAX30105_EnableFIFORollover(void);
void MAX30105_SetLEDMode(uint8_t mode);
void MAX30105_SetADCRange(uint8_t adcRange);
void MAX30105_SetSampleRate(uint8_t sampleRate);
void MAX30105_SetPulseWidth(uint8_t pulseWidth);
void MAX30105_SetPulseAmplitudeRed(uint8_t amplitude);
void MAX30105_SetPulseAmplitudeIR(uint8_t amplitude);
void MAX30105_SetPulseAmplitudeGreen(uint8_t amplitude);
void MAX30105_SetPulseAmplitudeProximity(uint8_t amplitude);
void MAX30105_EnableSlot(uint8_t slotNumber, uint8_t device);
void MAX30105_ClearFIFO(void);
void MAX30105_Setup(uint8_t  powerLevel, uint8_t sampleAverage, uint8_t ledMode, uint32_t sampleRate, uint32_t pulseWidth, uint32_t adcRange);
void MAX30105_SoftReset(void);

/* [] END OF FILE */
