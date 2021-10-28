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
#include "AD5273BRJZ1.h"
#include "milliseconds.h"
#include "I2C_MAX30105.h"
#include <math.h>

#define AD5273_AD_PIN          (0x00)
#define AD5273_BASE_ADDRESS    (0x2CU) 
#define AD5273_ADDRESS         (AD5273_BASE_ADDRESS | AD5273_AD_PIN) // 010110 MSB


/* Command valid status */
#define TRANSFER_CMPLT        (0x00UL)
#define TRANSFER_ERROR        (0xFFUL)

#define I2C_TIMEOUT                         (100UL)
#define I2C_AD5273_HW                       I2C_MAX30105_HW
#define I2C_AD5273_INSTRUCTION_BYTE         (0x00)

#define AD5273_E0_STATUS_MASK   (1 << 6)
#define AD5273_E1_STATUS_MASK   (1 << 7)
#define AD5273_RDAC_MASK        ~(0xC0)

#define R_W_VALUE               (60U)
#define R_AB_VALUE              (10000UL)

static void HandleError(void);

uint8_t AD5273_ReadData(void)
{
    uint32_t  errorStatus = TRANSFER_ERROR;
    uint8_t rdacVal = 0U;
    errorStatus = I2C_MAX30105_MasterSendStart(AD5273_ADDRESS, CY_SCB_I2C_READ_XFER, I2C_TIMEOUT);
    
    if(CY_SCB_I2C_SUCCESS != errorStatus)
    {
       goto exit;
    }
    
    errorStatus = I2C_MAX30105_MasterReadByte(CY_SCB_I2C_NAK, &rdacVal, I2C_TIMEOUT);
    
    /* Check status of transaction */
    if ((errorStatus == CY_SCB_I2C_SUCCESS)           ||
        (errorStatus == CY_SCB_I2C_MASTER_MANUAL_NAK) ||
        (errorStatus == CY_SCB_I2C_MASTER_MANUAL_ADDR_NAK))
    {
        /* Send Stop condition on the bus */
        if (I2C_MAX30105_MasterSendStop(I2C_TIMEOUT) == CY_SCB_I2C_SUCCESS)
        {
           errorStatus = TRANSFER_CMPLT;
        }
    }
    
    exit:
    
        if(TRANSFER_CMPLT != errorStatus)
        {
            errorStatus = TRANSFER_ERROR;
        }
    
        return rdacVal;
}

uint8_t AD5273_WriteData(uint8_t val)
{
    uint32_t  errorStatus = TRANSFER_ERROR;
    
    if(val > 64) goto exit;
    
    errorStatus = I2C_MAX30105_MasterSendStart(AD5273_ADDRESS, CY_SCB_I2C_WRITE_XFER, I2C_TIMEOUT);
    
    if(CY_SCB_I2C_SUCCESS != errorStatus)
    {
       goto exit;
    }
    
    errorStatus = I2C_MAX30105_MasterWriteByte(I2C_AD5273_INSTRUCTION_BYTE, I2C_TIMEOUT);
    
    if(CY_SCB_I2C_SUCCESS != errorStatus)
    {
       goto exit;
    }
    
    errorStatus = I2C_MAX30105_MasterWriteByte(val & AD5273_RDAC_MASK, I2C_TIMEOUT);
    
    /* Check status of transaction */
    if ((errorStatus == CY_SCB_I2C_SUCCESS)           ||
        (errorStatus == CY_SCB_I2C_MASTER_MANUAL_NAK) ||
        (errorStatus == CY_SCB_I2C_MASTER_MANUAL_ADDR_NAK))
    {
        /* Send Stop condition on the bus */
        if (I2C_MAX30105_MasterSendStop(I2C_TIMEOUT) == CY_SCB_I2C_SUCCESS)
        {
           errorStatus = TRANSFER_CMPLT;
        }
    }
    
    
  exit:
    
    if(TRANSFER_CMPLT != errorStatus)
    {
       errorStatus = TRANSFER_ERROR;
    }
    
    return errorStatus;
}

void AD5273_SetWiper(uint32_t resVal)
{
    uint8_t result = 0U;
    uint8_t rdacVal = 0U;
    
    if(0x00 != resVal)
    {
      /* R_WB(D) = (D * R_AB)/63 + R_W 
        rdacVal is D in this equation*/
      rdacVal = (uint8_t)ceil(((resVal - R_W_VALUE)/ (float)R_AB_VALUE) * 63.0);
    }
    
    AD5273_WriteData(rdacVal);
    
    result = AD5273_ReadData();
    
    if(((AD5273_E1_STATUS_MASK | AD5273_E0_STATUS_MASK) & result) != 0x00)
    {
        HandleError();
    }
}


/*******************************************************************************
* Function Name: HandleError
****************************************************************************//**
*
* This function processes unrecoverable errors such as any component 
* initialization errors etc. In case of such error the system will 
* stay in the infinite loop of this function.
*
*
* \note
* * If error ocuurs interrupts are disabled.
*
*******************************************************************************/
static void HandleError(void)
{   
     /* Disable all interrupts. */
    __disable_irq();
    
    /* Infinite loop. */
    while(1u) {}
}
/* [] END OF FILE */
