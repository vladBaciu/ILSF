#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include "PID.h"
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
#define SHOW_DEBUG_DATA           FALSE
#define USE_PID_CONTROLLER        FALSE


#define BPM_AVERAGE_RATE          (6U)
#define SPO2_AVERAGE_RATE         (8U)

#define SERIAL_FRAME_LENGTH_MAX   (300U)

#define FRAME_START               (0xDA)
#define FRAME_TERMINATOR_1        (0xEA)
#define FRAME_TERMINATOR_2        (0xDC)

#define PI_KI_VALUE               (0.0027)
#define PI_KI_MAX_ERROR           (27000)
#define PI_KP_VALUE               (0.30)

#define PI_MIN_OUTPUT_VALUE      (0)
#define PI_MAX_OUTPUT_VALUE      (200)


#define ANALOG_PIN                (17U)
#define ADC_REF_VOLTAGE           (3.3)
#define ADC_MAX_VALUE             (1024)

#define PWM_CONTROL_PIN           (9U)

#define TICK_SECOND      100       

#define NO_OF_ELEMENTS(arr)        (sizeof(arr)/sizeof(arr[0]))
/**************************************************** GLOBAL VARIABLES **********************************************************/

uint8_t hr_lookup_table[] = {50,55,60,65,70,74,78,83,86,90,94,98,104,108,110,120};
float voltage_lookup_table[] = { 0.87, 0.91, 1.06, 1.18, 1.38, 1.53, 1.65, 1.77, 1.924, 2.095, 2.27, 2.43, 2.53, 2.66, 2.8, 3.1};


volatile float voltageValue = 0;
volatile uint8_t lookup_index = 0;
volatile float v_in, duty_cycle_out, v_set = 100;
volatile int error = 0;
volatile float integral = 0;
char inputBuffer[4];

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

struct pid_controller pidctrl;
pid_t pid = 0;

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
uint8_t searchLookUpIndex(uint8_t bmpValue)
{
  uint8_t index = 0;
  
  if(bmpValue <= hr_lookup_table[0])
  {
    index = 0;
  
  }
  else if(bmpValue >= hr_lookup_table[sizeof(hr_lookup_table)])
  {
    index = NO_OF_ELEMENTS(hr_lookup_table) - 1;
  
  }
  else
  {
      index = -1;
  }
  for(uint8_t i = 0; i < NO_OF_ELEMENTS(hr_lookup_table) - 2; i++)
  { 

  
      if((bmpValue >= hr_lookup_table[i]) && (bmpValue <= hr_lookup_table[i+1]))
      {
        index = i;
        break;
      }
    
  }
  return index;
}
float getControlVoltage(uint8_t lookup_index, uint8_t bpm)
{
  float v1 = voltage_lookup_table[lookup_index];
  float v2 = voltage_lookup_table[lookup_index + 1];
  uint8_t hr_1 = hr_lookup_table[lookup_index];
  uint8_t hr_2 = hr_lookup_table[lookup_index + 1];
  // y = y1 + ((x – x1) / (x2 – x1)) * (y2 – y1),
  return v1 + ((bpm - hr_1)/(float)(hr_2 - hr_1)) * (v2 - v1);

}

void setTimers(void)
{
     TCB0.CTRLB = TCB_CNTMODE_INT_gc;    // Setting to periodic interrupt mode
     TCB0.INTCTRL = TCB_CAPT_bm;      // enable interrupt
     TCB0.CCMP = 0xffff;            // setting
     TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;  // timer prescaler of 2, Enable timer TCB2
 sei();
}

 void clk_init(void) {
     CPU_CCP = CCP_IOREG_gc;  // write IOREG to CCP to unlock MCLKCTRLB for 4 clock cycles
     CLKCTRL_MCLKCTRLB = 0; // set CLK_PER to 20MHz
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
  clk_init();
  setTimers();
  pinMode(PWM_CONTROL_PIN, OUTPUT);

#if (TRUE == USE_PID_CONTROLLER)
  pid = pid_create(&pidctrl, &v_in, &duty_cycle_out, &v_set, 1.4, 0.004, 0);
 
  pid_limits(pid, 0, 200);

  pid_auto(pid);
#endif
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
      if(FALSE == gTissueDetected)
      {
        gTempbeatAvg = 80; 
      }
      
#if (SHOW_DEBUG_DATA == TRUE)      
       debug.print("Error: ");
       debug.print(error);
       debug.print('\n');
       debug.print("BPM: ");
       debug.print(gTempbeatAvg);
       debug.print('\n');
       debug.print("V_SET: ");
       debug.print(v_set);
       debug.print('\n');
       debug.print("Duty out: ");
       debug.print(duty_cycle_out);
       debug.print('\n');
       debug.print("V_IN:");
       debug.print(v_in);
       debug.print('\n');
#endif       
   /*
         if (Serial.available() > 0) 
         {

          Serial.readBytesUntil('\n', inputBuffer, 4);
          gTempbeatAvg = atoi(inputBuffer);
          Serial.print("BPM value: ");
          Serial.println(gTempbeatAvg);
        
          memset(inputBuffer,0x00,4);
  `     }
  */
   }
   
}

ISR(TCB0_INT_vect)
{
 TCB0.INTFLAGS = TCB_CAPT_bm;
  
  v_in = analogRead(ANALOG_PIN) * (ADC_REF_VOLTAGE / (ADC_MAX_VALUE - 1));
  lookup_index = searchLookUpIndex(gTempbeatAvg);
  v_set = getControlVoltage(lookup_index,gTempbeatAvg);
  error = (v_set - v_in) * 100;

#if(FALSE == USE_PID_CONTROLLER)
  duty_cycle_out = (int) ((PI_KP_VALUE * error) + (PI_KI_VALUE * integral));
  
  integral += error;
  if ((integral) > PI_KI_MAX_ERROR) 
  {
       integral = PI_KI_MAX_ERROR;
  }
  if ((integral) < -PI_KI_MAX_ERROR) 
  {
       integral = -PI_KI_MAX_ERROR;
  }
  if (duty_cycle_out >= PI_MAX_OUTPUT_VALUE) 
  {
       duty_cycle_out = PI_MAX_OUTPUT_VALUE;
  }
  if (duty_cycle_out < PI_MIN_OUTPUT_VALUE) 
  {
       duty_cycle_out = PI_MIN_OUTPUT_VALUE;
  }
  if(gTempbeatAvg == 0)
  {
    analogWrite(PWM_CONTROL_PIN, 0 );
  }
  else
  {
    analogWrite(PWM_CONTROL_PIN, duty_cycle_out );
  }
#else
  pid_compute(pid);
  analogWrite(PWM_CONTROL_PIN, duty_cycle_out );
#endif
}
