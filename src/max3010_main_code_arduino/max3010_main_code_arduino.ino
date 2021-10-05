#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;


#define debug Serial

/******************************************************* MACROS ******************************************************************/
#define TRUE                      true
#define FALSE                     false

#define NO_FINGER_THRESHOLD       (50000UL)

#define RED_CHANNEL   (0U)
#define GREEN_CHANNEL (1U)
#define IR_CHANNEL    (2U)


#define READ_FROM_FIFO_BUFFER     TRUE
#define FIFO_NUMBER_OF_SAMPLES               (100U)
#define FIFO_NUMBER_OF_OVERLAPPING_SAMPLES   (75U)


#define USE_CUSTOM_SENSOR_CFG     TRUE

#define BPM_AVERAGE_RATE          (6U)
#define SPO2_AVERAGE_RATE         (8U)

#define SERIAL_FRAME_LENGTH_MAX   (300U)

#define FRAME_START               (0xDA)
#define FRAME_TERMINATOR_1        (0xEA)
#define FRAME_TERMINATOR_2        (0xDC)
/**************************************************** GLOBAL VARIABLES **********************************************************/

uint32_t samplesTaken = 0UL;

uint32_t FIFO_Buffer[3 * FIFO_NUMBER_OF_SAMPLES];
uint32_t channelsValues[3];
uint8_t gBeatAvg;

int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
uint32_t gLastHeartRate;
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

uint32_t gTempbeatAvg;
uint8_t gBpmBuff[BPM_AVERAGE_RATE];
uint8_t gBpmCurrentIndex = 0U;

float gTempSpO2Avg;
uint32_t gTempSpO2AvgInt1;
uint32_t gTempSpO2AvgInt2;

float gSpO2Buff[BPM_AVERAGE_RATE];
uint8_t gSpO2CurrentIndex = 0U;

bool gTissueDetected = FALSE;

uint8_t *gpFrame = NULL;
/**************************************************** GLOBAL STRUCTS AND DEFINITIONS ********************************************/
/* Options: 0=Off to 255=50mA */
#define CFG_LED_BRIGHTNESS   (60)
/* Options: 1, 2, 4, 8, 16, */
#define CFG_SAMPLE_AVERAGE   (4)
/* Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green */
#define CFG_LED_MODE         (2)
/* Options: 50, 100, 200, 400, 800, 1000, 1600, 3200 */
#define CFG_SAMPLE_RATE      (100)
/* Options: 69, 118, 215, 411 */
#define CFG_PULSE_WIDTH      (411)
/* Options: 2048, 4096, 8192, 16384 */
#define CFG_ADC_RANGE        (16384)

typedef struct 
{
  byte ledBrightness;
  byte sampleAverage;
  byte ledMode;
  int sampleRate;
  int pulseWidth;
  int adcRange;
} sensorConfig_t;

typedef enum 
{ 
    CHANNEL_DATA = 0x7C, 
    PARAMS = 0x83, 
    DEBUG = 0xF2
} frameType_t;

typedef struct
{
  uint8_t dummy;
}debugType_t;
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
/**************************************************** CUSTOM FUNCTIONS **********************************************************/
uint32_t readChannel(uint8_t channel)
{
  uint32_t channelValue;
  if(RED_CHANNEL == channel)
  {
    channelValue = particleSensor.getRed();
  }
  else if(GREEN_CHANNEL == channel)
  {
    channelValue = particleSensor.getGreen();
  }
  else if(IR_CHANNEL == channel)
  {
    channelValue = particleSensor.getIR();
  }
  else
  {
    channelValue = 0;
  }
  
  return channelValue;
}

bool readChannels(uint32_t *channelsBuffer, bool readFIFO)
{
  bool bIsTissuePresent = TRUE;
  if(READ_FROM_FIFO_BUFFER != readFIFO)
  {
    *(channelsBuffer + RED_CHANNEL) = particleSensor.getRed();
    *(channelsBuffer + GREEN_CHANNEL) = particleSensor.getGreen();
    *(channelsBuffer + IR_CHANNEL) = particleSensor.getIR();
    if (*(channelsBuffer + IR_CHANNEL) < NO_FINGER_THRESHOLD)
    {
      bIsTissuePresent = FALSE;
    }
  }
  else
  {
    particleSensor.check();
 
    while(particleSensor.available())
      {
       
        *(channelsBuffer + ((RED_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + samplesTaken)) = particleSensor.getFIFORed();
        *(channelsBuffer + ((GREEN_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + samplesTaken)) = particleSensor.getFIFOGreen();
        *(channelsBuffer + ((IR_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + samplesTaken)) = particleSensor.getFIFOIR();
        if (*(channelsBuffer + ((IR_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + samplesTaken)) < NO_FINGER_THRESHOLD)
        {
            bIsTissuePresent = FALSE;
        }
        particleSensor.nextSample();

        samplesTaken++;
        
        if (FIFO_NUMBER_OF_SAMPLES == samplesTaken)
        {
          break;
        }
      }
  }
  return bIsTissuePresent;
}

uint8_t computeBPM(uint32_t sample)
{
  long delta;
  float beatsPerMinute;
  uint32_t tempbeatAvg;
  uint8_t beatAvg;
  static byte rates[BPM_AVERAGE_RATE]; 
  static byte rateSpot = 0;
  static long lastBeat = 0;

  if(checkForBeat(sample) == true)
  {
    //We sensed a beat!
    delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 40)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= BPM_AVERAGE_RATE; //Wrap variable

      //Take average of readings
      tempbeatAvg = 0;
      for (byte x = 0 ; x < BPM_AVERAGE_RATE ; x++)
        tempbeatAvg += rates[x];
      tempbeatAvg /= BPM_AVERAGE_RATE;
    }
    beatAvg = (uint8_t) tempbeatAvg;
  }

  return beatAvg;
}

uint8_t* createSerialFrame(void *inputData, uint8_t noOfBytes, frameParams_t *serialFrameStruct)
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
    case DEBUG:
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

void sendFrame(uint8_t *pFrame)
{ 
  bool terminator_1 = FALSE;
  bool endOfFrame = FALSE;
  char tempBuff[7];
  while(!endOfFrame)
  {

    if (*pFrame < 0x10) {Serial.print("0");}
    debug.print(*pFrame, HEX);
    
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
}

void setTimers(void)
{
 /* Load the Compare or Capture register with the timeout value*/
 TCB2.CCMP = 0x7fff;

 /* Enable TCB and set CLK_PER divider to 1 (No Prescaling) */
 TCB2.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;

 /* Enable Capture or Timeout interrupt */
 TCB2.INTCTRL = TCB_CAPT_bm;


  
 sei();
}
/**************************************************** ARDUINO STANDARD FUNCTIONS **********************************************************/

void setup()
{
  debug.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);


#if (USE_CUSTOM_SENSOR_CFG == TRUE)
  sensorConfig_t max30105Config;
#endif  

  // Initialize sensor
  if (particleSensor.begin() == false)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    while (1);
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
#if (USE_CUSTOM_SENSOR_CFG == TRUE)
  max30105Config.ledBrightness = CFG_LED_BRIGHTNESS;
  max30105Config.sampleAverage = CFG_SAMPLE_AVERAGE;
  max30105Config.ledMode = CFG_LED_MODE; 
  max30105Config.sampleRate = CFG_SAMPLE_RATE; 
  max30105Config.pulseWidth = CFG_PULSE_WIDTH; 
  max30105Config.adcRange = CFG_ADC_RANGE;
  particleSensor.setup(max30105Config.ledBrightness, max30105Config.sampleAverage, 
                       max30105Config.ledMode, max30105Config.sampleRate, max30105Config.pulseWidth, max30105Config.adcRange); 
#else
  particleSensor.setup();
#endif

  //setTimers();

}


void loop()
{
  
  
   gTissueDetected = readChannels((uint32_t *)&FIFO_Buffer[0], TRUE);
   if(samplesTaken == FIFO_NUMBER_OF_SAMPLES)
   {
      maxim_heart_rate_and_oxygen_saturation(&FIFO_Buffer[IR_CHANNEL * FIFO_NUMBER_OF_SAMPLES], FIFO_NUMBER_OF_SAMPLES, &FIFO_Buffer[RED_CHANNEL * FIFO_NUMBER_OF_SAMPLES], &spo2, &validSPO2, &heartRate, &validHeartRate);
   }

   while(1)
   {
      for (byte i = FIFO_NUMBER_OF_SAMPLES - FIFO_NUMBER_OF_OVERLAPPING_SAMPLES; i < FIFO_NUMBER_OF_SAMPLES; i++)
      {

        FIFO_Buffer[((RED_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + (i - (FIFO_NUMBER_OF_SAMPLES - FIFO_NUMBER_OF_OVERLAPPING_SAMPLES)))] = FIFO_Buffer[((RED_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + i)];
        FIFO_Buffer[((IR_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + (i - (FIFO_NUMBER_OF_SAMPLES - FIFO_NUMBER_OF_OVERLAPPING_SAMPLES)))] = FIFO_Buffer[((IR_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + i)];
  
      }
      samplesTaken = FIFO_NUMBER_OF_OVERLAPPING_SAMPLES;
      while(FIFO_NUMBER_OF_SAMPLES != samplesTaken)
        gTissueDetected = readChannels((uint32_t *)&FIFO_Buffer[0], TRUE);

      maxim_heart_rate_and_oxygen_saturation(&FIFO_Buffer[IR_CHANNEL * FIFO_NUMBER_OF_SAMPLES], FIFO_NUMBER_OF_SAMPLES, &FIFO_Buffer[RED_CHANNEL * FIFO_NUMBER_OF_SAMPLES], &spo2, &validSPO2, &heartRate, &validHeartRate);
      
      if(TRUE == validHeartRate)
      {

        if(abs(gLastHeartRate - heartRate) < 10UL || (0UL == gLastHeartRate))
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
        for (byte x = 0 ; x < BPM_AVERAGE_RATE ; x++)
          gTempbeatAvg += gBpmBuff[x];
        gTempbeatAvg /= BPM_AVERAGE_RATE;
      }

      if(TRUE == validSPO2)
      {
          gSpO2Buff[gSpO2CurrentIndex++] = spo2;
          gSpO2CurrentIndex %= 4;

          gTempSpO2Avg = 0;
          for (byte x = 0 ; x < 4; x++)
            gTempSpO2Avg += gSpO2Buff[x];
          gTempSpO2Avg /= 4;
      }

      memset(&frameParam, 0x00, sizeof(frameParam));  
      frameParam.frameType = PARAMS;
      frameParam.params.hr_spo2[0] = gTempbeatAvg;
      gTempSpO2AvgInt1 = gTempSpO2Avg;
      gTempSpO2AvgInt2 = (gTempSpO2Avg - gTempSpO2AvgInt1) * 100;
      frameParam.params.hr_spo2[1] = gTempSpO2AvgInt1;
      frameParam.params.hr_spo2[2] = gTempSpO2AvgInt2;
      frameParam.tissueDetected = gTissueDetected;
      
      gpFrame =  createSerialFrame(NULL, 2, &frameParam);

      sendFrame(gpFrame);
      debug.print('\n');


      memset(&frameParam, 0x00, sizeof(frameParam));

      frameParam.frameType = CHANNEL_DATA;
      frameParam.params.wavelength = IR_CHANNEL;
      frameParam.tissueDetected = gTissueDetected;
      gpFrame = createSerialFrame(&FIFO_Buffer[(IR_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + FIFO_NUMBER_OF_OVERLAPPING_SAMPLES], (FIFO_NUMBER_OF_SAMPLES - FIFO_NUMBER_OF_OVERLAPPING_SAMPLES) * 4, &frameParam);
      
      sendFrame(gpFrame);
      //debug.print('\n');

      memset(&frameParam, 0x00, sizeof(frameParam));

      frameParam.frameType = CHANNEL_DATA;
      frameParam.params.wavelength = RED_CHANNEL;
      frameParam.tissueDetected = gTissueDetected;
      gpFrame = createSerialFrame(&FIFO_Buffer[(RED_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + FIFO_NUMBER_OF_OVERLAPPING_SAMPLES], (FIFO_NUMBER_OF_SAMPLES - FIFO_NUMBER_OF_OVERLAPPING_SAMPLES) * 4, &frameParam);

     sendFrame(gpFrame);
     debug.print('\n');
   // debug.println("Frame");
  //  for (byte i = FIFO_NUMBER_OF_OVERLAPPING_SAMPLES; i < FIFO_NUMBER_OF_SAMPLES; i++)
 //   {
  //     debug.print(FIFO_Buffer[((IR_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + i)]);
  //     debug.print('\n');
  //  }
   }
}

ISR(TCB2_INT_vect)
{
 
  TCB2.INTFLAGS = TCB_CAPT_bm;
 
}
