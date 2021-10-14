#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"


/******************************************************* MACROS ******************************************************************/
/* @brief Just a redefinition */
#define TRUE                      true
#define FALSE                     false

/* @brief Channel indexes */
#define RED_CHANNEL   (0U)
#define GREEN_CHANNEL (1U)
#define IR_CHANNEL    (2U)

/* @brief Some config macros */ 
#define PING_PONG_TOWER_VER                      2U

#define SERIAL_FRAME_LENGTH_MAX               (300U)
#define USE_CUSTOM_SENSOR_CFG                  TRUE
#define SHOW_DEBUG_DATA                        TRUE
#define USE_PID_CONTROLLER                     FALSE
#define SET_MANUAL_BEAT_AVG                    TRUE 
#define READ_FROM_FIFO_BUFFER                  TRUE
#define FIFO_NUMBER_OF_SAMPLES                (100U)
#define FIFO_NUMBER_OF_OVERLAPPING_SAMPLES    (75U)
#define NO_FINGER_THRESHOLD       (50000UL)
#define BPM_AVERAGE_RATE          (6U)
#define SPO2_AVERAGE_RATE         (8U)
#define debug                     Serial

#if(PING_PONG_TOWER_VER == 1U)

#define PI_KI_VALUE               (0.0027)
#define PI_KI_MAX_ERROR           (27000)
#define PI_KP_VALUE               (0.34)

#elif(PING_PONG_TOWER_VER == 2U)

#define PI_KI_VALUE               (0.0027)
#define PI_KI_MAX_ERROR           (27000)
#define PI_KP_VALUE               (0.35)

#else

#define PI_KI_VALUE               (0.0)
#define PI_KI_MAX_ERROR           (0)
#define PI_KP_VALUE               (0.0)
#endif
#define PI_MIN_OUTPUT_VALUE       (0U)
#define PI_MAX_OUTPUT_VALUE       (200U)

#define ANALOG_PIN                (17U)
#define ADC_REF_VOLTAGE           (4.8)
#define ADC_MAX_VALUE             (1024.0)
#define PWM_CONTROL_PIN           (9U)

/* @brief Frame flags. Same as in the Python application */
#define FRAME_START               (0xDA)
#define FRAME_TERMINATOR_1        (0xEA)
#define FRAME_TERMINATOR_2        (0xDC)


#define SWITCH_DOWN_DIR       0U
#define SWITCH_UP_DIR         1U
/* @brief Get number of elements from an array */
#define NO_OF_ELEMENTS(arr)        (sizeof(arr)/sizeof(arr[0]))
/**************************************************** GLOBAL VARIABLES **********************************************************/
/* @brief Look up tables for HR and voltage */
#if(PING_PONG_TOWER_VER == 1U)
uint8_t hr_lookup_table[] =    {50  ,   60,   70,   80,   90,  100, 110, 120};
float voltage_lookup_table[] = {1.15, 1.43, 1.70, 2.09, 2.42, 2.7, 3.0, 3.3};
#elif(PING_PONG_TOWER_VER == 2U)
uint8_t hr_lookup_table[] =    {50  ,   60,   70,   80,   90,  100, 110, 120};
float voltage_lookup_table[] = {1.20, 1.49, 1.84, 2.13, 2.49, 2.73, 3.2, 3.3};
#else
#error Lookup table not defined for this ping pong tower
#endif
/* @brief Volatile variables used for PI controller inside ISR */
volatile float voltageValue = 0;
volatile uint8_t lookup_index = 0;
volatile float v_in, duty_cycle_out, v_set = 100;
volatile float error = 0;
volatile float integral = 0;

#if (SET_MANUAL_BEAT_AVG == TRUE) 
/* @brief Input manual buffer */ 
char inputBuffer[4];
#endif
/* @brief Stores the number of samples aquired in the input buffer */
uint32_t samplesTaken = 0UL;
/* @brief Count seconds  */
uint32_t secCounter = 0UL;
/* @brief BPM change  */
uint8_t gSwitchDirection = SWITCH_UP_DIR;
/* @brief Channel buffers */
uint32_t FIFO_Buffer[3 * FIFO_NUMBER_OF_SAMPLES];
uint32_t channelsValues[3];
uint8_t gBeatAvg;

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
uint32_t gTempbeatAvg;
uint8_t gBpmBuff[BPM_AVERAGE_RATE];
uint8_t gBpmCurrentIndex = 0U;

/* @brief Temporary variables */
float gTempSpO2Avg;
uint32_t gTempSpO2AvgInt1;
uint32_t gTempSpO2AvgInt2;

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
#define CFG_SAMPLE_AVERAGE   (4)
/* Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green */
#define CFG_LED_MODE         (2)
/* Options: 50, 100, 200, 400, 800, 1000, 1600, 3200 */
#define CFG_SAMPLE_RATE      (100)
/* Options: 69, 118, 215, 411 */
#define CFG_PULSE_WIDTH      (411)
/* Options: 2048, 4096, 8192, 16384 */
#define CFG_ADC_RANGE        (16384)

/* @brief Sensor config structure */
typedef struct 
{
  byte ledBrightness;
  byte sampleAverage;
  byte ledMode;
  int sampleRate;
  int pulseWidth;
  int adcRange;
} sensorConfig_t;

/* @brief Enum with frame types */
typedef enum 
{ 
    CHANNEL_DATA = 0x7C, 
    PARAMS = 0x83, 
    DEBUG = 0xF2
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

#if (USE_PID_CONTROLLER == TRUE)
struct pid_controller pidctrl;
pid_t pid = 0;
#endif

MAX30105 particleSensor;

/**************************************************** CUSTOM FUNCTIONS **********************************************************/
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

/* Function: readChannels
 * Param: uint32_t *channelsBuffer - output buffer where the channels are stored
 *        bool readFIFO - read from FIFO buffer or not
 * Description: Read IR, RED and GREEN channels
 * Return type: bool
 */
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

/* Function: computeBPM
 * Param: uint32_t sample - the last sample from one of the buffers (red/ir/green) - preferablly IR.
 * Description: Compute BPM. Runs for each aquired sample in order to detect the beat. Function not used 
 *              for the moment since it requires that all the samples to be treated - performs poorly if 
 *              samples are lost or skiped.
 * Return type: uint8_t
 */
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
    /* A beat was detected */
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

/* Function: createSerialFrame
 * Param: void *inputData - a generic pointer to an input buffer
 *        uint8_t noOfBytes - number of bytes in the input buffer to be included in the frame
 *        frameParams_t *serialFrameStruct - contains params such as hr, sp02, etc
 * Description: Create the type of frame according to the frameType parameter. DEBUG frame not implemented.
 * Return type: void
 */
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

/* Function: sendFrame
 * Param: uint8_t *pFrame - pointer to a buffer frame 
 * Description: Sends the buffer frame untill the end terminators are detected
 * Return type: void
 */
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

/* Function: searchLookUpIndex
 * Param: uint8_t bmpValue - search bmp interval in the lookup table
 * Description: gets the index (interval) where the bpmValue corresponds in the
 *              lookup table
 * Return type: uint8_t 
 */
uint8_t searchLookUpIndex(uint8_t bmpValue)
{
  uint8_t index = 0;
  
  if(bmpValue <= hr_lookup_table[0])
  {
    index = 0;
  }
  else if(bmpValue >= hr_lookup_table[sizeof(hr_lookup_table)-1])
  {
    index = NO_OF_ELEMENTS(hr_lookup_table) - 1;
  }
  else
  {
    for(uint8_t i = 1; i < NO_OF_ELEMENTS(hr_lookup_table) - 1; i++)
    { 
      if((bmpValue >= hr_lookup_table[i]) && (bmpValue <= hr_lookup_table[i+1]))
      {
        index = i;
        break;
      }
    }
  }
  
  
  return index;
}

/* Function: getControlVoltage
 * Param: uint8_t lookup_index - reference index for the bpm interval (i, and i++)
 *        uint8_t bpm - bpm to be converted
 * Description: converts a bpm value in control voltage
 * Return type: float
 */
float getControlVoltage(uint8_t lookup_index, uint8_t bpm)
{
  if(lookup_index == 0U)
  {
     return voltage_lookup_table[0];
  }
  else if(lookup_index == NO_OF_ELEMENTS(hr_lookup_table) - 1)
  {
     return voltage_lookup_table[NO_OF_ELEMENTS(hr_lookup_table) - 1];
  }
  else
  {
    float v1 = voltage_lookup_table[lookup_index];
    float v2 = voltage_lookup_table[lookup_index + 1];
    uint8_t hr_1 = hr_lookup_table[lookup_index];
    uint8_t hr_2 = hr_lookup_table[lookup_index + 1];
    // y = y1 + ((x – x1) / (x2 – x1)) * (y2 – y1),
    return v1 + ((bpm - hr_1)/(float)(hr_2 - hr_1)) * (v2 - v1);
  }
}

/* Function: setTimers
 * Param: void
 * Description: Activate TCB0 timer unit in compare mode. Used for PI controller updates.
 * Return type: void
 */
void setTimers(void)
{
     TCB0.CTRLB = TCB_CNTMODE_INT_gc;    // Setting to periodic interrupt mode
     TCB0.INTCTRL = TCB_CAPT_bm;      // enable interrupt
     TCB0.CCMP = 0xffff;            // setting
     TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;  // timer prescaler of 2, Enable timer TCB2
     sei();
}

/* Function: clk_init
 * Param: void
 * Description: Init clock to 20 MHz
 * Return type: void
 */
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
/* USE CUSTOM CONFIG FOR MAX SENSOR */
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

/* NOT USED NOW, NEEDS TUNING */
#if (TRUE == USE_PID_CONTROLLER)
  pid = pid_create(&pidctrl, &v_in, &duty_cycle_out, &v_set, 1.4, 0.004, 0);
 
  pid_limits(pid, 0, 200);

  pid_auto(pid);
#endif
}


void loop()
{
   gTempbeatAvg = 80;
   
#if (SET_MANUAL_BEAT_AVG == FALSE)  
   gTissueDetected = readChannels((uint32_t *)&FIFO_Buffer[0], TRUE);
   if(samplesTaken == FIFO_NUMBER_OF_SAMPLES)
   {
      maxim_heart_rate_and_oxygen_saturation(&FIFO_Buffer[IR_CHANNEL * FIFO_NUMBER_OF_SAMPLES], FIFO_NUMBER_OF_SAMPLES, &FIFO_Buffer[RED_CHANNEL * FIFO_NUMBER_OF_SAMPLES], &spo2, &validSPO2, &heartRate, &validHeartRate);
   }

   for (byte x = 0 ; x < BPM_AVERAGE_RATE ; x++)
   {
      gBpmBuff[x] = 80;
   }
#endif
   while(1)
   {
    
#if (SET_MANUAL_BEAT_AVG == FALSE)     
      for (byte i = FIFO_NUMBER_OF_SAMPLES - FIFO_NUMBER_OF_OVERLAPPING_SAMPLES; i < FIFO_NUMBER_OF_SAMPLES; i++)
      {

        FIFO_Buffer[((RED_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + (i - (FIFO_NUMBER_OF_SAMPLES - FIFO_NUMBER_OF_OVERLAPPING_SAMPLES)))] = FIFO_Buffer[((RED_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + i)];
        FIFO_Buffer[((IR_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + (i - (FIFO_NUMBER_OF_SAMPLES - FIFO_NUMBER_OF_OVERLAPPING_SAMPLES)))] = FIFO_Buffer[((IR_CHANNEL * FIFO_NUMBER_OF_SAMPLES) + i)];
  
      }
      samplesTaken = FIFO_NUMBER_OF_OVERLAPPING_SAMPLES;
      while(FIFO_NUMBER_OF_SAMPLES != samplesTaken)
        gTissueDetected = readChannels((uint32_t *)&FIFO_Buffer[0], TRUE);

      maxim_heart_rate_and_oxygen_saturation(&FIFO_Buffer[IR_CHANNEL * FIFO_NUMBER_OF_SAMPLES], FIFO_NUMBER_OF_SAMPLES, &FIFO_Buffer[RED_CHANNEL * FIFO_NUMBER_OF_SAMPLES], &spo2, &validSPO2, &heartRate, &validHeartRate);
      
      if((TRUE == validHeartRate) && (heartRate > 40))
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
#endif
#if (SET_MANUAL_BEAT_AVG == TRUE)
    /* 
     *  if (Serial.available() > 0) 
     {
          Serial.readBytesUntil('\n', inputBuffer, 4);
          gTempbeatAvg = atoi(inputBuffer);
          Serial.print("BPM value: ");
          Serial.println(gTempbeatAvg);
        
          memset(inputBuffer,0x00,4);
     }
    */
    if(secCounter == 6)
    {
      if(gTempbeatAvg == 120)
      {
        gSwitchDirection = SWITCH_DOWN_DIR;
      }
      else if(gTempbeatAvg == 50)
      {
        gSwitchDirection = SWITCH_UP_DIR;
      }
      else
      {
        gSwitchDirection = gSwitchDirection;
      }

      if(gSwitchDirection == SWITCH_UP_DIR)
      {
        gTempbeatAvg = gTempbeatAvg + 10;
      }
      else
      {
        gTempbeatAvg = gTempbeatAvg - 10;
      }
      
      
      secCounter = 0UL;
    }
#endif      
#if (SHOW_DEBUG_DATA == TRUE)      
       debug.print("Error: ");
       debug.print(error);
       debug.write(13);
       debug.write(10);
       debug.print("BPM: ");
       debug.print(gTempbeatAvg);
       debug.write(13);
       debug.write(10);
       debug.print("V_SET: ");
       debug.print(v_set);
       debug.write(13);
       debug.write(10);
       debug.print("Duty out: ");
       debug.print(duty_cycle_out);
       debug.write(13);
       debug.write(10);
       debug.print("V_IN:");
       debug.print(v_in);
       debug.write(13);
       debug.write(10);
#endif 
   secCounter++;
   }
   
}

/* Used to update the PI controller output */
ISR(TCB0_INT_vect)
{
 TCB0.INTFLAGS = TCB_CAPT_bm;
  
  v_in = analogRead(ANALOG_PIN) * (ADC_REF_VOLTAGE / (float)(ADC_MAX_VALUE - 1.0));
  lookup_index = searchLookUpIndex((uint8_t) gTempbeatAvg);
  v_set = getControlVoltage(lookup_index,gTempbeatAvg);
  error = (v_set - v_in) * 100.0;

#if(USE_PID_CONTROLLER == FALSE)
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
