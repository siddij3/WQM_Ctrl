//Libraries
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h> // for BlueTooth
#include "Adafruit_ADS1015.h"

//Project files
#include "WQM_PotStat_Shield.h"

/* IO
   Name Pin Function
   WQM:
   D4   4   O:Led (WQM_LED)
   D5   5   O:Free Cl Switch Enable (WQM_ClSwEn)
   D11  11  I:WQM Board present (WQM_BrdPresent)

   Main/Shared:
   D13  13  O:Led (MB_Led)
   A0   14  O:Ext. Led (if present)
   SDA  18  I2C SDA, comms to ADCs/DAC
   SCL  19  I2C SCL, comms to ADCs/DAC
   D0   0   RX1, Serial RX, HW Serial
   D1   1   TX1, Serial TX, HW Serial
   D2   2   RX2, Serial RX, SW Serial
   D3   3   TX2, Serial TX, SW Serial
*/

/*
 * GLOBAL VARIABLES
 */

//Board Vars
boolean WQM_Present = false; //WQM Shield Present
//boolean PS_Present = false; //PotStat Shield Present

//unsigned long tScratch2 = 0;

// BlueTooth
// [BT] <-->  [Arduino]
// VCC  <-->  3.3V
// GND  <-->  GND
// TxD  <-->  pin D2
// RxD  <-->  pin D3
//SoftwareSerial mySerial (rxPin, txPin);
SoftwareSerial Serial_BT(2,3);

// WQM Variables
Adafruit_ADS1115 WQM_adc1(0x48);  // THIS IS THE ONE
Adafruit_ADS1115 WQM_adc2(0x49);

bool ClSwState = false;

float voltage_pH = 0.0; //Voltage in mV
float current_Cl = 0.0; //Current in nA
float fVoltageAdc = 0.0; //Voltage in mV
float temperature = 0.0; //Temperature in deg. C
float V_temp = 0.0; //Voltage for temperature calculation
float voltage_alkalinity = 0.0; //Voltage for alkalinity calculation

//Current time in seconds since start of free Cl measurement collection (milliseconds)
long switchTimeACC = 0;
//Total time of on and off phases
long switchTimePRE = CL_SW_ON_TIME + CL_MEASURE_TIME;

int16_t WQM_adc1_diff_0_1;  // pin0 - pin1, raw ADC val
int16_t WQM_adc1_diff_2_3;  // pin2 - pin3, raw ADC val this one
int16_t WQM_adc2_diff_0_1;  // pin0 - pin1, raw ADC val
int16_t WQM_adc2_diff_2_3;  // pin0 - pin1, raw ADC val

//end WQM vars

// PS ADC declaration
//Adafruit_ADS1115 PS_adc1(0x4B);

//int16_t PS_adc1_diff_0_1;  // pin0 - pin1, raw ADC val
//int16_t PS_adc1_diff_2_3;  // pin2 - pin3, raw ADC val

//Number of parameters required per experiment, index 0 = null/not used, index 1 = CSV/LSV, index 2 = DPV
const int PARAMS_REQD[3] = {0, 9, 10};
//Experiment parameter limits: [Experiment][Parameter][Max/Min]
const long EXP_LIMITS[3][10][2] =  {
  {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}}, //null experiment / not used
  {{LIMS_CLEANT}, {LIMS_CLEANV}, {LIMS_CLEANT}, {LIMS_CLEANV}, {LIMS_CSV0}, {LIMS_CSV1}, {LIMS_CSV2}, {LIMS_CSV3}, {LIMS_CSV4}, {LIMS_CSV5}}, //CSV limits
  {{LIMS_CLEANT}, {LIMS_CLEANV}, {LIMS_CLEANT}, {LIMS_CLEANV}, {LIMS_DPV0}, {LIMS_DPV1}, {LIMS_DPV2}, {LIMS_DPV3}, {LIMS_DPV4}, {LIMS_DPV5}}  //DPV limits
};

unsigned int timer1_preload;
unsigned int timer2_preload;

unsigned long tExpStart = 0; // experiment start time
unsigned long tExp = 0; // current experiment time since start (total)
unsigned long tInt = 0; // current time since start of experiment interval

unsigned long tScratch = 0;

// current interval during experiment
byte currInterval = 0; // 0 = not started / NA, 1 = cleaning, 2 = deposition, 3 = 1st exp int., 4 = 2nd exp int., 5 = complete

// current cycle during experiment
int currCycle = 0;

// experiment started flag
uint8_t expStarted = 0;
// async. adc sampling started flag
boolean samplingStarted = false;

// start conversion flags set by ISR
boolean startDAC = false;
//boolean PS_startADC = false;
boolean WQM_startADC = false;

//FWD and REV sampling completed for current cycle / period
boolean syncADCcompleteFWD = false;
boolean syncADCcompleteREV = false;

float vIn = 0.0;  //mV
float iIn = 0.0;  //uA
float vOut = 0.0; //V

uint16_t dacOut = DACVAL0; //Raw value for DAC output

//selected TIA feedback resistor value, in k ohm
float rGain;

//Structure for storing experiment config
struct Experiment {
  unsigned long tClean;
  float vClean;
  unsigned long tDep;
  float vDep;
  unsigned long tSwitch;    //start time of 2nd interval (us)
  unsigned long tOffset;    //offset time, used to get correct initial V
  unsigned long tCycle;     //total time per cycle, period (us)
  float vStart[2];          //start voltage of each interval (V)
  float vSlope[2];          //slope (V/us)
  float offset;             //voltage offset per cycle (V)
  int cycles;               //total cycles
  unsigned int sampRate;    //ADC sampling rate, if async sampling implemented
  boolean syncSamplingEN;     //Sync samping - false: ADC sampling based on Timer1 (CV, LSV) true: ADC samp. occurs twice per cycle (DPV, SWV)
  unsigned long tSyncSample;  //ADC start time for sync sampling
  /* Gain Setting (0-7): Determines TIA feedback resistance and ADC PGA setting:
     0: RG = 500, PGA = 4X, 2000uA
     1: RG = 500, PGA = 16X, 500uA
     2: RG = 10k, PGA = 4X, 100uA
     3: RG = 10k, PGA = 16X, 25uA
     4: RG = 200k, PGA = 4X, 5uA
     5: RG = 200k, PGA = 16X, 1.25uA
     6: RG = 4M, PGA = 4X, 250nA
     7: RG = 4M, PGA = 16X, 63nA
  */
  byte gain;
};

Experiment e; //current experiment config
char charRcvd;
/*
 * setup()
 *
 * Executes onnce board boot for setup
 *
 * Initializes IO, starts communications
 */
void setup() {
  //Initialize IO
  //main board (UNO) led pin
  pinMode(MB_LED, OUTPUT);
  //External led
  pinMode(EXT_LED, OUTPUT);

  //WQM board present input
  pinMode(WQM_BrdPresent, INPUT_PULLUP);

  delay(250);

  //Check for boards present
  WQM_Present =  true;//digitalRead(WQM_BrdPresent) ? false : true;

  delay(5000);
  //Initialize Serial port - setup BLE shield
    Serial.begin(9600);
    //while (!Serial) {
     // Serial.println("Master Baud Rate: = 9600");
       // wait for serial port to connect. Needed for native USB port only
    //}
    Serial_BT.println("Master Baud Rate: = 9600");
    Serial_BT.println("Setting BLE shield comms settings, name/baud rate(115200)");
    delay(500);
    Serial_BT.print("AT+NAMEIMWQMS"); //Set board name
    delay(250);
    Serial_BT.print("AT+BAUD4"); //Set baud rate to 115200 on BLE Shield
    delay(250);
    Serial_BT.println();
    Serial_BT.println("Increasing MCU baud rate to 115200");
    delay(500);
    Serial_BT.begin(115200);
    delay(200);
    Serial_BT.println("Master Baud Rate: = 115200");

  //Initialize I2C
  Wire.begin(); //Start I2C
  Wire.setClock(400000L);


  if (WQM_Present) {
    //Setup WQM outputs
    //WQM LED
    pinMode(WQM_LED, OUTPUT);

    //Free Cl digital switch enable
    pinMode(WQM_ClSwEn, OUTPUT);
    wqm_led(ON);
    WQM_adc1.begin();
    WQM_adc2.begin();
    WQM_adc1.setGain(GAIN_TWO); // set PGA gain to 2 (LSB=0.0625 mV, FSR=2.048)
    WQM_adc2.setGain(GAIN_FOUR); // set PGA gain to 2 (LSB=0.0625 mV, FSR=2.048)

    sendInfo("WQM Setup complete");
    //Run WQM only
    delay(1000);
    startExperimentWQM(); //TODO: Delete when comms complete
  } else {
    sendInfo("No WQM board detected");
  }
  delay(100);
 
   digitalWrite(PS_LED1, OFF);
  //digitalWrite(PS_LED2, OFF);
  wqm_led(OFF);

}
/*
 * Interrupt Service Routine
 * called by TMR2 overflow
 * raises flag in main loop to start DAC
 */
ISR(TIMER2_OVF_vect)
{
  TCNT2 = timer2_preload;   // preload timer
  startDAC = true;
}
/*
 * Interrupt Service Routine
 * called by TMR2 overflow
 * raises flag in main loop to start ADC
 */
ISR(TIMER1_OVF_vect)        // interrupt service routine
{
  TCNT1 = timer1_preload;   // preload timer
 // if (expStarted == PS_EXP_RUNNING) {
  //  PS_startADC = true;
  //  WQM_startADC = false;
 // } else 
  if (expStarted == WQM_EXP_RUNNING) {
   // PS_startADC = false;
    WQM_startADC = true;
    //Calculate time used for switched Cl measurements
    switchTimeACC = switchTimeACC + (1000 / WQM_SAMP_RATE);
    if (switchTimeACC > switchTimePRE) {
      switchTimeACC = switchTimeACC - switchTimePRE;
    }
  } else {
  //  PS_startADC = false;
    WQM_startADC = false;
    stopTimers();
  }

}

/*
 * Main program execution loop
 * 1. Calculate and start DAC conversion if flag set high (interrupt)
 * 2. Start WQM ADC conversion and process resul if flag set high (interrupt)
 * 3. Receive and respond to exeternal serial comms (Start/Stop experiment)
 */
void loop()
{
 
  //WQM_startADC flag set  (set from interrupt)
  if (WQM_startADC) {
    getMeasurementsWQM();
    sendValues();
    if (switchTimeACC >= CL_SW_ON_TIME && switchTimeACC < switchTimePRE) {
      ClSwState = true;
    } else {
      ClSwState = false;
    }
    setClSw(ClSwState);
    wqm_led(ClSwState);

    WQM_startADC = false;
    digitalWrite(EXT_LED,ClSwState);
  }
}
/*
 * FUNCTIONS
 */


/* Convert a series (array) of chars into an integer
 *
 *  returns: true if successful, false if invalid / non-numeric char is in array
 */

//Check if char represents a number
boolean isNum(char c) {
  return (c >= 0x30 && c <= 0x39);
}


//Add error prefix text to message and send to user
size_t sendError(String s) {
  //return 0;
  return Serial.println(String("Error: " + s));
}

//Add info prefix text to message and send to user
size_t sendInfo(String s) {
  //return 0;
  return Serial.println(String("Info: " + s));
}


/* Start timer used for to trigger interrupt for ADC conversion
    Timer preload/prescalers based on sample rate in current config (global var)
*/
void startTimerADC()
{
  // initialize timer1 (ADC interrupt)
  TCCR1A = 0;
  TCCR1B = 0;

  if (!e.syncSamplingEN) {
    // Set timer1_counter to the correct value for our interrupt interval
    timer1_preload = 131;

    if (e.sampRate > 30) {
      // 8x prescaler
      TCCR1B |= (1 << CS11);
      // preload timer 65536-16MHz/8/XXHz
      timer1_preload = 65536 - (2000000 / e.sampRate);
      //check if result should be rounded up
      if (2000000 % e.sampRate > e.sampRate / 2)
        timer1_preload += 1;
    } else {
      // 64x prescaler
      TCCR1B |= (1 << CS10);
      TCCR1B |= (1 << CS11);
      // preload timer 65536-16MHz/64/XXHz
      timer1_preload = 65536 - (250000 / e.sampRate);
      //check if result should be rounded up
      if (250000 % e.sampRate > e.sampRate / 2)
        timer1_preload += 1;
    }
    TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
    TCNT1 = timer1_preload;   // preload timer
    samplingStarted = true;
  }
}


/* Start timer used for to trigger interrupt for DAC conversion
    Timer preload/prescalers based on desired DAC output rate
*/
void startTimerDAC()
{
  // initialize timer2 (DAC interrupt)
  TCCR2A = 0;
  TCCR2B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  timer2_preload = 131;   // preload timer 256-16MHz/256/500Hz

  TCCR2B |= (1 << CS22);
  TCCR2B |= (1 << CS21);    // 256 prescaler
  TIMSK2 |= (1 << TOIE2);   // enable timer overflow interrupt
  TCNT2 = timer2_preload;   // preload timer

}
// Stops timer 1 and timer 2, disables interrupts
void stopTimers()
{
  // disable timer interrupts (DAC/ADC interrupt)
  TIMSK1 &= (0 << TOIE1);   // disable timer overflow interrupt
  TIMSK2 &= (0 << TOIE2);   // disable timer overflow interrupt
  //Stop timer
  TCCR1B = 0;
  TCCR2B = 0;
  samplingStarted = false;
}

//Turn Arduino board LED ON or OFF
void led(bool b) {
  digitalWrite(MB_LED, b);
}

//Flash Arduino board led n times with on/off time of d ms
void flashLed(byte n, unsigned int d) {
  for (byte i = 0; i < n * 2;  i++ ) {
    digitalWrite(MB_LED, digitalRead(MB_LED) ^ 1);
    digitalWrite(EXT_LED, digitalRead(MB_LED) ^ 1);
    delay(d);
  }
}

void startExperiment() {
  flashLed(4, 150);
  sendInfo("Starting Experiment");
  tExpStart = micros();
  samplingStarted = false;
  startTimerDAC();
  //expStarted = PS_EXP_RUNNING;

}

/* Finish active experiment
    reset DAC to default, reset timers, reset expStarted status
*/
void finishExperiment() {
  tExpStart = 0;
  currCycle = 0;
  stopTimers();
  expStarted = 0;
  switchTimeACC = 0; //Reset WQM switch time
  flashLed(2, 300);
}

// Action if unexpected error occurs
// lock program and flash led until board reset
void programFail(byte code) {
  stopTimers();
  while (1) {
    flashLed(code, 175);
    delay(2500);
  }
}

//WQM FUNCTIONS:
  void startExperimentWQM() {
    flashLed(4, 150);
    sendInfo("Starting WQM Experiment");
    samplingStarted = false;
    e.syncSamplingEN = false;
    e.sampRate = WQM_SAMP_RATE;
    expStarted = WQM_EXP_RUNNING;
    startTimerADC();
  }

  void getMeasurementsWQM() {
    // read from the ADC, and obtain a sixteen bits integer as a result
    if (WQM_Present) {
      WQM_adc1_diff_2_3 = WQM_adc1.readADC_Differential_2_3();
      delay(5);
      WQM_adc2_diff_0_1 = WQM_adc2.readADC_Differential_0_1();
      delay(5);
      WQM_adc1_diff_0_1 = WQM_adc1.readADC_Differential_0_1();
      delay(5);
      WQM_adc2_diff_2_3 =  WQM_adc1_diff_2_3;//WQM_adc2.readADC_Differential_2_3();

    } else {
      //Simulated ADC signals for when not connected to WQM board (temp) (fudges random data)
      WQM_adc1_diff_0_1 = 2000 + random(100);


      if (ClSwState) {
        WQM_adc1_diff_2_3 = -2000 + random(100);
        WQM_adc2_diff_2_3 = -2000 + random(100);
      } else {
        WQM_adc1_diff_2_3 = 0;
        WQM_adc2_diff_2_3 = 0;
      }
      WQM_adc2_diff_0_1 = 2000 + random(100);
      WQM_adc2_diff_2_3 = 2000 + random(100);

    }
    voltage_pH = WQM_adc1_diff_0_1 * 0.0625; // in mV
    current_Cl = -WQM_adc1_diff_2_3 * 0.0625 / 0.0255; // in nA, feedback resistor = 500k
    V_temp = WQM_adc2_diff_0_1 * 0.03125; // in mV
    voltage_alkalinity = WQM_adc2_diff_2_3 * 0.03125; // in mV
  }


  //Send WQM meas. values over serial port
  void sendValues() {


    Serial_BT.print(" ");
    Serial_BT.print(V_temp, 4);
    Serial_BT.print(" ");
    Serial_BT.print(voltage_pH, 4);
    Serial_BT.print(" ");
    Serial_BT.print(current_Cl, 4);
    Serial_BT.print(" ");
    Serial_BT.print(voltage_alkalinity, 4);  //Make changes in app to read the proper order #TODO
    Serial_BT.print(" ");
    Serial_BT.print((float)switchTimeACC / 1000.0, 1);  //Turns off the switch for free chlorine
    Serial_BT.print(" ");

    // Serial_BT.print(fVoltageAdc,2);
    // Serial_BT.print("\t");
    // Serial_BT.print(WQM_adc1_diff_2_3);
    // Serial_BT.print("\t");
    // Serial_BT.print(current_Cl, 2);  //Make changes in app to read the proper order #TODO
    Serial_BT.print("\t");
    if (ClSwState) {
      Serial_BT.print("1");
    } else {
      Serial_BT.print("0");
    }
    Serial_BT.print("\t");
    Serial_BT.print((float)switchTimeACC / 1000.0, 1);
    Serial_BT.print(" ");
    Serial_BT.print("\n");

  }

  //Set free Cl switch ON or OFF
  void setClSw(boolean b) {
    digitalWrite(WQM_ClSwEn, b);
  }

  //Turn WQM board LED ON or OFF
  void wqm_led(boolean b) {
    digitalWrite(WQM_LED, b);
  }
