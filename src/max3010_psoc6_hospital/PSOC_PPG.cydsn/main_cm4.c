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

#include "MAX30105.h"
#include "milliseconds.h"
#include "spo2_algorithm.h"

#include "string.h"
#include <stdio.h>
#include <stdlib.h>

/******************************************************* MACROS ******************************************************************/
/* @brief Just a redefinition */
#define TRUE                      true
#define FALSE                     false

/* @brief Channel indexes */
#define RED_CHANNEL   (0U)
#define GREEN_CHANNEL (1U)
#define IR_CHANNEL    (2U)


/* @brief Some config macros */
#define NO_FINGER_THRESHOLD                   (50000UL)

#define READ_FROM_FIFO_BUFFER                 TRUE
#define FIFO_NUMBER_OF_SAMPLES               (100U)
#define FIFO_NUMBER_OF_OVERLAPPING_SAMPLES   (75U)

#define USE_CUSTOM_SENSOR_CFG     TRUE

#define BPM_AVERAGE_RATE          (6U)
#define SPO2_AVERAGE_RATE         (8U)

/**************************************************** GLOBAL VARIABLES **********************************************************/

/* @brief Stores the number of samples aquired in the input buffer */
uint32_t samplesTaken = 0UL;
/* @brief Channel buffers */
uint16_t FIFO_Buffer[3 * FIFO_NUMBER_OF_SAMPLES];
uint32_t channelsValues[3];

/* @brief Stores the SpO2 value */
int32_t spo2;
/* @brief Stores the status of SpO2 value */ 
int8_t validSPO2; 
/* @brief Stores the HR value */
int32_t heartRate;
/* @brief Las HR rate value */ 
int32_t gLastHeartRate;
/* @brief Stores the status of HR value */ 
int8_t validHeartRate; 

/* @brief HR buffer and index used for averaging */
uint8_t gBeatAvg;
uint8_t gBpmBuff[BPM_AVERAGE_RATE];
uint8_t gBpmCurrentIndex = 0U;

/* @brief Temporary variables */
float gTempSpO2Avg;
uint32_t gTempSpO2AvgInt1;
uint32_t gTempSpO2AvgInt2;
uint32_t gTempbeatAvg;

/* @brief SpO2 buffer and index used for averaging */
float gSpO2Buff[BPM_AVERAGE_RATE];
uint8_t gSpO2CurrentIndex = 0U;

/* @brief Stores the status of the measurement */
bool gTissueDetected = FALSE;

/* @brief Pointer to a serial frame newly created*/
uint8_t *gpFrame = NULL;
/**************************************************** GLOBAL STRUCTS AND DEFINITIONS ********************************************/
/* Options: 0=Off to 255=50mA */
#define CFG_LED_BRIGHTNESS   (60)
/* Options: 1, 2, 4, 8, 16, */
#define CFG_SAMPLE_AVERAGE   (8)
/* Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green */
#define CFG_LED_MODE         (2)
/* Options: 50, 100, 200, 400, 800, 1000, 1600, 3200 */
#define CFG_SAMPLE_RATE      (200)
/* Options: 69, 118, 215, 411 */
#define CFG_PULSE_WIDTH      (411)
/* Options: 2048, 4096, 8192, 16384 */
#define CFG_ADC_RANGE        (16384)


/* @brief Frame flags. Same as in the Python application */
#define FRAME_START               (0xDA)
#define FRAME_TERMINATOR_1        (0xEA)
#define FRAME_TERMINATOR_2        (0xDC)
#define SERIAL_FRAME_LENGTH_MAX   (300U)
/* @brief Sensor config structure */
typedef struct 
{
  uint8_t ledBrightness;
  uint8_t sampleAverage;
  uint8_t ledMode;
  int sampleRate;
  int pulseWidth;
  int adcRange;
} sensorConfig_t;

/* @brief Enum with frame types */
typedef enum 
{ 
    CHANNEL_DATA = 0x7C, 
    PARAMS = 0x83, 
    DEBUG_FRAME = 0xF2
} frameType_t;

/* @brief Store params for the debug frame. Not supported yet. */
typedef struct
{
  uint8_t dummy;
}debugType_t;

/* @brief Store params for differend kind of serial frames */
typedef struct
{
  frameType_t frameType;
  union
  {
    uint8_t hr_spo2[3];
    uint8_t wavelength;
    debugType_t debugParam;
  }params;
  bool tissueDetected;
}frameParams_t;

frameParams_t frameParam;


static void HandleError(void);

/**************************************************** CUSTOM FUNCTIONS **********************************************************/
/* Function: createSerialFrame
 * Param: void *inputData - a generic pointer to an input buffer
 *        uint8_t noOfBytes - number of bytes in the input buffer to be included in the frame
 *        frameParams_t *serialFrameStruct - contains params such as hr, sp02, etc
 * Description: Create the type of frame according to the frameType parameter. DEBUG frame not implemented.
 * Return type: void
 */
uint8_t* createSerialFrame(void *inputData, uint16_t noOfBytes, frameParams_t *serialFrameStruct)
{
  static uint8_t serialFrame[SERIAL_FRAME_LENGTH_MAX + 5];
  if( SERIAL_FRAME_LENGTH_MAX + 5 < noOfBytes)
    serialFrameStruct->frameType = 0xFF;
    
  serialFrame[0] = FRAME_START;
  
  switch(serialFrameStruct->frameType)
  {
    case CHANNEL_DATA:
        serialFrame[1] = CHANNEL_DATA;
        serialFrame[2] = serialFrameStruct->tissueDetected;
        serialFrame[3] = serialFrameStruct->params.wavelength;
        memcpy(&serialFrame[4], inputData, noOfBytes);
        serialFrame[noOfBytes + 4] = FRAME_TERMINATOR_1;
        serialFrame[noOfBytes + 5] = FRAME_TERMINATOR_2;
        break;
        
    case PARAMS:
        serialFrame[1] = PARAMS;
        serialFrame[2] = serialFrameStruct->tissueDetected;
        serialFrame[3] = serialFrameStruct->params.hr_spo2[0];
        serialFrame[4] = serialFrameStruct->params.hr_spo2[1];
        serialFrame[5] = serialFrameStruct->params.hr_spo2[2];
        serialFrame[6] = FRAME_TERMINATOR_1;
        serialFrame[7] = FRAME_TERMINATOR_2;
        break;
        
    case DEBUG_FRAME:
        break;
        
    default:
        serialFrame[1] = 0xDE;
        serialFrame[2] = 0xAD;
        serialFrame[3] = 0xDE;
        serialFrame[4] = 0xAD;
        serialFrame[5] = FRAME_TERMINATOR_1;
        serialFrame[6] = FRAME_TERMINATOR_2;
        break;
  }

  return serialFrame;
}

/* Function: sendFrame
 * Param: uint8_t *pFrame - pointer to a buffer frame 
 * Description: Sends the buffer frame untill the end terminators are detected
 * Return type: void
 */
void sendFrame(uint8_t *pFrame)
{ 
  bool terminator_1 = FALSE;
  bool endOfFrame = FALSE;
  
  while(!endOfFrame)
  {

    
    printf("%02x", *pFrame);
    
    if(FRAME_TERMINATOR_1 == *pFrame)
    {
      terminator_1 = TRUE;
    }

    if((TRUE == terminator_1) && (FRAME_TERMINATOR_2 == *pFrame))
    {
      endOfFrame = TRUE;
    }
   
    pFrame++;

  }
  printf("\n");
}




/* Function: readChannel
 * Param: uint8_t channel - channel to be read
 * Description: Read channel (not from FIFO buffer)
 * Return type: uint32_t
 */
uint32_t readChannel(uint8_t channel)
{
  uint32_t channelValue;
  if(RED_CHANNEL == channel)
  {
    channelValue = MAX30105_GetRed();
  }
  else if(GREEN_CHANNEL == channel)
  {
    channelValue = MAX30105_GetGreen();
  }
  else if(IR_CHANNEL == channel)
  {
    channelValue = MAX30105_GetIR();
  }
  else
  {
    channelValue = 0;
  }
  
  return channelValue;
}

/* Function: readChannels
 * Param: uint32_t *channelsBuffer - output buffer where the channels are stored
 *        bool readFIFO - read from FIFO buffer or not
 * Description: Read IR, RED and GREEN channels
 * Return type: bool
 */
bool readChannels(uint16_t *channelsBuffer, bool readFIFO)
{
  bool bIsTissuePresent = TRUE;
  if(READ_FROM_FIFO_BUFFER != readFIFO)
  {
    *(channelsBuffer + RED_CHANNEL) = (uint16_t) MAX30105_GetRed();
    *(channelsBuffer + GREEN_CHANNEL) = (uint16_t) MAX30105_GetGreen();
    *(channelsBuffer + IR_CHANNEL) = (uint16_t) MAX30105_GetIR();
    if (*(channelsBuffer + IR_CHANNEL) < NO_FINGER_THRESHOLD)
    {
      bIsTissuePresent = FALSE;
    }
  }
  else
  {
    MAX30105_Check();
 
    while(MAX30105_Available())
      {
        *(channelsBuffer + ((RED_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + samplesTaken)) = (uint16_t) MAX30105_GetFIFORed();
        *(channelsBuffer + ((GREEN_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + samplesTaken)) = (uint16_t) MAX30105_GetFIFOGreen();
        *(channelsBuffer + ((IR_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + samplesTaken)) = (uint16_t) MAX30105_GetFIFOIR();
        if (*(channelsBuffer + ((IR_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + samplesTaken)) < NO_FINGER_THRESHOLD)
        {
            bIsTissuePresent = FALSE;
        }
        MAX30105_NextSample();

        samplesTaken++;
        
        if (FIFO_NUMBER_OF_SAMPLES == samplesTaken)
        {
          break;
        }
      }
  }
  return bIsTissuePresent;
}

int main(void)
{
    MILLIS_AssignISR();
    
    
    __enable_irq(); /* Enable global interrupts. */
    
     
   
    MILLIS_InitAndStartTimer();
    
    
    /* UART initialization status */
    cy_en_scb_uart_status_t uart_status ;
    
     /* Initialize UART operation. Config and Context structure is copied from Generated source. */
    uart_status  = Cy_SCB_UART_Init(Uart_Printf_HW, &Uart_Printf_config, &Uart_Printf_context);
    if(uart_status != CY_SCB_UART_SUCCESS)
    {
        HandleError();
    }	
    
    Cy_SCB_UART_Enable(Uart_Printf_HW);
    

    MAX30105_Setup(CFG_LED_BRIGHTNESS, CFG_SAMPLE_AVERAGE, CFG_LED_MODE, CFG_SAMPLE_RATE, CFG_PULSE_WIDTH, CFG_ADC_RANGE);
    
    
    gTissueDetected = readChannels(&FIFO_Buffer[0], TRUE);


    while(1)
    {
      
      for (uint8_t i = FIFO_NUMBER_OF_SAMPLES - FIFO_NUMBER_OF_OVERLAPPING_SAMPLES; i < FIFO_NUMBER_OF_SAMPLES; i++)
      {

        FIFO_Buffer[((RED_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + (i - (FIFO_NUMBER_OF_SAMPLES - FIFO_NUMBER_OF_OVERLAPPING_SAMPLES)))] = FIFO_Buffer[((RED_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + i)];
        FIFO_Buffer[((IR_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + (i - (FIFO_NUMBER_OF_SAMPLES - FIFO_NUMBER_OF_OVERLAPPING_SAMPLES)))] = FIFO_Buffer[((IR_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + i)];
  
      }
      samplesTaken = FIFO_NUMBER_OF_OVERLAPPING_SAMPLES;
      while(FIFO_NUMBER_OF_SAMPLES != samplesTaken)
        gTissueDetected = readChannels(&FIFO_Buffer[0], TRUE);

      
      
      if(TRUE == validHeartRate)
      {

        if((uint8_t)abs(gLastHeartRate - heartRate) < 10UL || (0UL == gLastHeartRate))
        {
          gBpmBuff[gBpmCurrentIndex++] = heartRate;
          gLastHeartRate = heartRate;
          gBpmCurrentIndex %= BPM_AVERAGE_RATE;
        }
        else
        {
          gLastHeartRate = heartRate - 10;
        }
  
       
        gTempbeatAvg = 0;
        for (uint8_t x = 0 ; x < BPM_AVERAGE_RATE ; x++)
          gTempbeatAvg += gBpmBuff[x];
        gTempbeatAvg /= BPM_AVERAGE_RATE;
      }

     if(TRUE == validSPO2)
     {
         gSpO2Buff[gSpO2CurrentIndex++] = spo2;
         gSpO2CurrentIndex %= 4;

         gTempSpO2Avg = 0;
         for (uint8_t x = 0 ; x < 4; x++)
           gTempSpO2Avg += gSpO2Buff[x];
         gTempSpO2Avg /= 4;
     }
    /*
    if(TRUE == gTissueDetected)
    {
        printf("HR: %lu \r\n", gTempbeatAvg); 
        printf("Saturation: %u.%u \r\n", (uint8_t) gTempSpO2Avg, (uint8_t)(100* (gTempSpO2Avg - (uint8_t) gTempSpO2Avg)));
    }
    */
;
    
    memset(&frameParam, 0x00, sizeof(frameParam));

     frameParam.frameType = CHANNEL_DATA;
     frameParam.params.wavelength = IR_CHANNEL;
     frameParam.tissueDetected = gTissueDetected;
     gpFrame = createSerialFrame(&FIFO_Buffer[(IR_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + FIFO_NUMBER_OF_OVERLAPPING_SAMPLES], (FIFO_NUMBER_OF_SAMPLES - FIFO_NUMBER_OF_OVERLAPPING_SAMPLES) * 2, &frameParam);
      
     sendFrame(gpFrame);
  
  
 
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


/*******************************************************************************
* When printf function is called it is redirected to the following functions
* depending on compiler used.
*******************************************************************************/
#if defined (__GNUC__)
/*******************************************************************************
* Function Name: _write
********************************************************************************
* Summary: 
* NewLib C library is used to retarget printf to _write. printf is redirected to 
* this function when GCC compiler is used to print data to terminal using UART. 
*
* \param file
* This variable is not used.
* \param *ptr
* Pointer to the data which will be transfered through UART.
* \param len
* Length of the data to be transfered through UART.
*
* \return
* returns the number of characters transferred using UART.
* \ref int
*******************************************************************************/
   int _write(int file, char *ptr, int len)
    {
        int nChars = 0;

        /* Suppress the compiler warning about an unused variable. */
        if (0 != file)
        {
        }
                
        nChars = Cy_SCB_UART_PutArray(Uart_Printf_HW, ptr, len);
           
        return (nChars);
    }
#elif defined(__ARMCC_VERSION)
    
/*******************************************************************************
* Function Name: fputc
********************************************************************************
* Summary: 
* printf is redirected to this function when MDK compiler is used to print data
* to terminal using UART.
*
* \param ch
* Character to be printed through UART.
*
* \param *file
* pointer to a FILE object that identifies the stream where the character is to be
* written.
*
* \return
* This function returns the character that is written in case of successful
* write operation else in case of error EOF is returned.
* \ref int
*******************************************************************************/
    struct __FILE
    {
        int handle;
    };
    
    enum
    {
        STDIN_HANDLE,
        STDOUT_HANDLE,
        STDERR_HANDLE
    };
    
    FILE __stdin = {STDIN_HANDLE};
    FILE __stdout = {STDOUT_HANDLE};
    FILE __stderr = {STDERR_HANDLE};
    
    int fputc(int ch, FILE *file)
    {
        int ret = EOF;
        switch(file->handle)
        {
            case STDOUT_HANDLE:
                while (0UL == Cy_SCB_UART_Put(Uart_Printf_HW, ch))
                {
                }
                ret = ch;
            break;
                
            case STDERR_HANDLE:
                ret = ch;
            break;
                
            default:
                file = file;
            break;
        }
        return(ret);
    }
    

#endif /* (__GNUC__) */

/* [] END OF FILE */
