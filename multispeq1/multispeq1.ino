
// Firmware for MultispeQ 1.0 hardware.   Part of the PhotosynQ project.

// setup() and support routines - also see loop.h


// FIRMWARE VERSION OF THIS FILE (SAVED TO EEPROM ON FIRMWARE FLASH)
#define FIRMWARE_VERSION "0.50"
#define DEVICE_NAME "MultispeQ"

// update DAC and get lights working in [{}]
// update printf everywhere, make sure everything is Print_...
// once lights work, comparison test old and new adc routines, with timing
//

/*

  + test do we need to calibrate offsets (like we did with the betas?)

  Create API for read_userdef, save_userdev, and reset_eeprom, delete all other eeprom commands, clean up all 1000… calls.
  Using bluetooth ID as ID -
  Firmware needs api call to write code to eeprom (unique ID).
  Get rid of user_enter… and replace with new user enter, then delete main function
  Pull out the alert/confirm/prompt into a subroutine.
  If averages == 1, then print data in real time
  Reimplement Actinic background light… make sure to update (currently set to 13)
  reimplement print_offset and get_offset

  Make the “environmental” a separate subroutine and pass the before or after 0,1 to it.

  I would like to set the micro-einstein level for the lights in my measurements rather than a raw (unitless) 12 bit value.


  Convert all possible into an array to make designing protocols more user friendly
  x turn pulse_distance and pulse_size → into an array

  x Greg - find suitable small magnet to embed, talk with Geoff

  Android to check for empty ID (if all 0s, or all 1s, then set api call to make unique ID == BLE mac address.
  Check protocol routines (produce error codes if fail):
   Battery check: Calculate battery output based on flashing the 4 IR LEDs at 250 mA each for 10uS.  This should run just before any new protocol - if it’s too low, report to the user
   (greg) Overheat LED check: adds up time + intensity + pulsesize and length of pulses, and calculates any overages.  Report overages - do not proceed if over.  Also needs a shutoff which is available through a 1000 call.
   Syntax check: make sure that the structure of the JSON is correct, report corrupted
   CRC check: so we expect CRC on end of protocol JSON, and check to make sure it’s valid.  Report corrupted
   LED intensity range check?  Ie, certain LEDs can only go up to a certain intensity

  Define and then code 1000+ calls for all of the sensors (for chrome app to call)
  Sebastian - can we change the sensor read commands 1000 over to normal protocols - then you output as per normal?)
  Check with sebastian about adding comments to protocols (even the inventor of json thinks there is a place for them)
  Clean up the protocols - light intensity (make into a single 1000+ call, see old code to bring it in)
  Check to make sure “averages” works in the protocols
  Clean up the pulsesize = 0 and all that stuff… meas_intensity…
  Attach par sensor to USB-C breakout, make calibration routine.
  Look up and see why iOS doesn’t connect.
  Test the power down feature from the bluetooth.  Determine how long it takes when powered down to come back to life (from bluetooth).  Include auto power off in the main while loop - auto-off after 10 seconds.
  Troubleshoot issues with bluetooth between protocols.


  Start documenting the commands + parameters to pass…
  And the eeprom commands

   Next to do:
   Add calibration commands back in
   consolidate commands to end up with:
   get rid of droop in dac values at <10.

   First thing - go through eeprom.h and set all of the variables in top and bottom of file...
   expose only a portion to users via 1000+ commands

   I suggest that all userdefs be a single float - JZ
   read_userdef - option to get a single userdef or all userdefs.  include the long arrays (like calibration_slope) as well as the offset and other variables which are not currently saved as userdefs.  All saved values are saveable in get_userdef.  This should have a print option when someone wants to also print it, and a get all option to print all userdefs (as JSON follow existing structure).
   replaces get_calibration, get_calibration_userdef, call_print_calibration, print_sensor_calibration, print_offset, set_device_info

   save_userdef - saves a value to EEPROM.  The size of the saved array here is defined by the userdef number.  So maybe number 1- 20 are user definable (some doubles, some triples, some long arrays), then 21 - 60 are 'reserved' for us developers (ie we don't expose them). is device info, 2 - 45 are 2 arrays, 45 - 50 are 3 array, and 50 - 60 are 25 array (or something).  Should include a standard print option to show success (as JSON).
   replaces add_calibration, add_userdef, save_calibration_slope, save_calibration_yint, calibrate_offset

   reset_eeprom - option to reset all, or just reset the exposed user values (1 - 20)
   replaces reset_all

   Note: code for each function should have some intial comments describing what it does

   Note: eeprom writing needs refactoring - eliminate all EEPROMxAnything()

   Here's the remaining commands in this file which should be moved.  I've organized them by function.  Actually this will be nice as some of them need to be renamed and cleaned up anyway.

  // power off
  pwr_off

  // Calibration commands
  add_calibration             // this adds a single element to the calibrations stored as an array (containing all of the LED pins)
  get_calibration             // this gets and prints to serial a saved calibration value
  print_cal                   // print a single calibration value
  add_userdef                 // this adds a set of userdefs (may be 2, 3, or 4 objects in length)
  get_calibration_userdef     // this gets and prints to serial a saved userdef value
  print_get_userdef           // this prints all of the saved userdef values
  call_print_calibration      // call or call + print all calibration values
  save_calibration_slope      // get rid of these - integrate into rest of normal save system
  save_calibration_yint       // get rid of these - integrate into rest of normal save system
  reset_all                   // reset all calibration values, or only all except the device info.

  // Calculation of PAR from PAR sensor, and from MultispeQ LEDs, and calibration of light sensor
  Light_Intensity
  calculate_intensity
  calculate_intensity_background
  lux_to_uE
  uE_to_intensity
  calibrate_light_sensor

  // Calculate frequency of the timers
  reset_freq

  // CoralspeQ related functions
  readSpectrometer
  print_data

  // Commands related to the detector offset
  calibrate_offset
  calculate_offset
  print_sensor_calibration
  print_offset

  // Device info command
  set_device_info

  // EEPROMxAnything - delete this and use new eeprom.h

  NEXT STEPS:

  - CHANGE THE WAY WE UPTAKE THE ACTNINIC AND MEASURING LIGHTS - ACT_LIGHTS, MEAS_LIGHTS, DETECTORS 1/2/3/4, SEE HOW IT'S ITERPRETED IN THE CODE.

  a_lights : [[3,4,5],[3],[3]]
  a_intensities : [[254,450,900],[254],[254]]
  meas_lights : [[2,3],[2,3],[3]]
  meas_intensities[[500,100],[500,100],[100]]
  detectors : [[2, 1],[2, 1],[1]]
  pulses : [100,100,200]
  pulse_distance : [10000,10000,1000]
  pulse_size :  [10,100,10]
  message : [["alert":"alert message"],["prompt":"prompt message"],["0","0"]]


  A flat structure is also possible... where each pulse set is a separate object.  In that case I'd have to interpret each pulse set separately using the JSON interpret tool.  This takes time, and it's preferable to completely
  interpret the JSON prior to starting the measurement.  Interpreting it between pulses would likely cause delays and increase the shortest possible measurement pulse distance.
  The other option is to interpret each JSON and rebuild the arrays at the beginning of the measurement - this would be quite tedious and hard to do.

*/

/*
  ////////////////////// HARDWARE NOTES //////////////////////////

  The detector operates with an AC filter, so only pulsing light (<100us pulses) passes through the filter.  Permanent light sources (like the sun or any other constant light) is completely
  filtered out.  So in order to measure absorbance or fluorescence a pulsing light must be used to be detectedb by the detector.  Below are notes on the speed of the measuring lights
  and actinic lights used in the MultispeQ, and the noise level of the detector:

  Optical:
  RISE TIME, Measuring: 0.4us
  FALL TIME, Measuring: 0.2us
  RISE TIME Actinic (bright): 2us
  FALL TIME Actinic (bright): 2us
  RISE TIME Actinic (dim): 1.5us
  FALL TIME Actinic (dim): 1.5us

  Electrical:
  RISE TIME, Measuring: .4us
  FALL TIME, Measuring: .2us
  RISE TIME Actinic: 1.5us
  FALL TIME Actinic: 2us
  NOISE LEVEL, detector 1: ~300ppm or ~25 detector units from peak to peak
  OVERALL: Excellent results - definitely good enough for the spectroscopic measurements we plant to make (absorbance (ECS), transmittance, fluorescence (PMF), etc.)

  //////////////////////  I2C Addresses //////////////////////
  TCS34725 0x29

*/

//#define DEBUG 1         // uncomment to add full debug features
//#define DEBUGSIMPLE 1   // uncomment to add partial debug features
//#define DAC 1           // uncomment for boards which do not use DAC for light intensity control
//#define PULSERDEBUG 1   // uncomment to debug the pulser and detector
//#define NO_ADDON        // uncomment if add-on board isn't present (one missing DAC, etc)

// includes
#include <i2c_t3.h>
#include <Time.h>                                                             // enable real time clock library
#include "utility/Adafruit_Sensor.h"
#include "json/JsonParser.h"
#include "utility/mcp4728.h"              // delete this once PAR is fixed
#include "DAC.h"
#include "AD7689.h"               // external ADC
#include "utility/Adafruit_BME280.h"      // temp/humidity/pressure sensor
#define EXTERN
#include "eeprom.h"
#include <typeinfo>
#include <ADC.h>                  // internal ADC
#include "serial.h"
#include "flasher.h"
#include "crc32.h"
#include <SPI.h>    // include the new SPI library:


// forward declarations (better to use .h files)

// routines for over-the-air firmware updates
void upgrade_firmware(void);
void boot_check(void);
int Light_Intensity(int var1);
void recall_save(JsonArray _recall_eeprom, JsonArray _save_eeprom);

//void call_print_calibration (int _print);


// replace legacy routines with new equivalents
/*
  #define user_enter_str(timeout, pwr_off) Serial_Input_String("+", (unsigned long) timeout)
  #define user_enter_dbl(timeout) Serial_Input_Double("+", (unsigned long) timeout)
  #define user_enter_long(timeout) Serial_Input_Long("+",(unsigned long) timeout)
*/

// remove these after PCB testing
#define GAIN_BITS 0        // extra effective bits due to gain being higher than beta detector - example, 4x more gain = 2.0 bits
#define DIV_BITS  0        // extra bits due to missing voltage divider
// Gain reduces headroom and anything more than 4x will clip with Greg's examples
// Note: low signal levels cause an effective reduction in bits (worse SNR)


//////////////////////DEVICE ID FIRMWARE VERSION////////////////////////
//float device_id = 0;
//float manufacture_date = 0;

//////////////////////PIN DEFINITIONS AND TEENSY SETTINGS////////////////////////
//Serial, I2C, SPI...
#define RX       0        // serial port pins
#define TX       1

#define MOSI1    11       // SPI pins
#define MISO1    12

#define MOSI2    7
#define MISO2    8

#define SDA1     18       // I2C
#define SCL1     19

#define SCL2     29
#define SDA2     30

#define SS1      22
#define SCK1     13
#define SS2      9

// hall effect sensor (analog)
#define HALL_OUT 35

// Lights - map LED pin # to MCU pin #
// document colors and which board
#define PULSE1   5
#define PULSE2   20
#define PULSE3   3
#define PULSE4   10
#define PULSE5   4
#define PULSE6   24
#define PULSE7   27
#define PULSE8   26
#define PULSE9   25
#define PULSE10  23
// better method
unsigned short LED_to_pin[11] = {0, PULSE1, PULSE2, PULSE3, PULSE4, PULSE5, PULSE6, PULSE7, PULSE8, PULSE9, PULSE10 }; // NOTE!  We skip the first element in the array so that the array lines up correctly (PULSE1 == 1, PULSE2 == 2 ... )

// bluetooth
#define BLERESET 14  // deprecated in favor of power down
#define DEBUG_DC 2   // could allow reflashing of BLE module
#define DEBUG_DD 36

// sample and hold (hold + release detector cap)
#define HOLDM    6
#define HOLDADD 21

// peripheral USB 3.0 connector pins
#define DACT     40
#define ADCT     37
#define IOEXT1   31
#define IOEXT2   32

// battery management
// batt_me normally should be pulled high
// to check battery level, pull batt_me low and then measure batt_test
// check data sheet for ISET... pull high or low?
#define ISET     28          // controls charge rate (deprecated)
#define BATT_ME  33
#define BATT_TEST 34

/*NOTES*/// blank pin (used when no other pin is selected - probably should change this later
#define BLANK    32   // error - same as IOEXT2

// set internal analog reference
// any unused pins? ... organize by function on teensy please!  Make sure thisis updated to teensy 3.2 (right now file == teens 31.sch

/*
  #define ANALOGRESOLUTION 16
  #define MEASURINGLIGHT1 15      // comes installed with 505 cyan z led with a 20 ohm resistor on it.  The working range is 0 - 255 (0 is high, 255 is low)
  #define MEASURINGLIGHT2 16      // comes installed with 520 green z led with a 20 ohm resistor on it.  The working range is 0 - 255 (0 is high, 255 is low)
  #define MEASURINGLIGHT3 11      // comes installed with 605 amber z led with a 20 ohm resistor on it.  The working range is high 0 to low 80 (around 80 - 90), with zero off safely at 120
  #define MEASURINGLIGHT4 12      // comes installed with 3.3k ohm resistor with 940 LED from osram (SF 4045).  The working range is high 0 to low 95 (80 - 100) with zer off safely at 110
  #define ACTINICLIGHT1 20
  #define ACTINICLIGHT2 2
  #define CALIBRATINGLIGHT1 14
  #define CALIBRATINGLIGHT2 10
  #define ACTINICLIGHT_INTENSITY_SWITCH 5
  #define DETECTOR1 34
  #define DETECTOR2 35
  #define HOLDM 6
  #define PWR_OFF_LIGHTS 22           // teensy shutdown pin auto power off
  #define PWR_OFF 21           // teensy shutdown pin auto power off for lights power supply (TL1963)
  #define BATT_LEVEL 17               // measure battery level
  #define LDAC1 23
*/

#ifdef CORAL_SPEQ
//////////////////////PIN DEFINITIONS FOR CORALSPEQ////////////////////////
#define SPEC_GAIN      28
//#define SPEC_EOS       NA
#define SPEC_ST        26
#define SPEC_CLK       25
#define SPEC_VIDEO     A10
//#define LED530         15
//#define LED2200k       16
//#define LED470         20
//#define LED2200K       2
#define SPEC_CHANNELS    256
uint16_t spec_data[SPEC_CHANNELS];
unsigned long spec_data_average[SPEC_CHANNELS];            // saves the averages of each spec measurement
int idx = 0;
#endif
int spec_on = 0;                                           // flag to indicate that spec is being used during this measurement

int _meas_light;           // measuring light to be used during the interrupt
static const int serial_buffer_size = 5000;                                        // max size of the incoming jsons
static const int max_jsons = 15;                                                   // max number of protocols per measurement

#define NUM_PINS 26

// MCU pins that are controllable by the user
// More explanation???

float all_pins [NUM_PINS] = {
  15, 16, 11, 12, 2, 20, 14, 10, 34, 35, 36, 37, 38, 3, 4, 9, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33
};
float calibration_slope [NUM_PINS] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
float calibration_yint [NUM_PINS] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
float calibration_slope_factory [NUM_PINS] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
float calibration_yint_factory [NUM_PINS] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
float calibration_baseline_slope [NUM_PINS] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
float calibration_baseline_yint [NUM_PINS] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
float calibration_blank1 [NUM_PINS] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
float calibration_blank2 [NUM_PINS] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
float calibration_other1 [NUM_PINS] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
float calibration_other2 [NUM_PINS] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

// ???
int averages = 1;

//////////////////////Shared Variables///////////////////////////
volatile int off = 0, on = 0;
int analogresolutionvalue;
IntervalTimer timer0, timer1, timer2;
float data = 0;
float data_ref = 0;
int act_background_light = 0;
//extern float light_y_intercept;
//char* bt_response = "OKOKlinvorV1.8OKsetPINOKsetnameOK115200"; // Expected response from bt module after programming is done.
float freqtimer0;
float freqtimer1;
float freqtimer2;

// shared with PAR.cpp
// these should be eliminated
extern float light_slope;
extern float lux_local;
extern float r_local;
extern float g_local;
extern float b_local;
extern float lux_average;
extern float r_average;
extern float g_average;
extern float b_average;
extern float lux_average_forpar;
extern float r_average_forpar;
extern float g_average_forpar;
extern float b_average_forpar;

/*
  extern float light_y_intercept;
  extern float lux_to_uE(float _lux_average);
  extern int Light_Intensity(int var1);
  extern int calculate_intensity(int _light, int tcs_on, int _cycle, float _light_intensity);
  extern int calculate_intensity_background(int _light, int tcs_on, int _cycle, float _light_intensity, int _background_intensity);
*/

////////////////////ENVIRONMENTAL variables averages (must be global) //////////////////////
float analog_read_average = 0;
float digital_read_average = 0;
float relative_humidity_average = 0;
float temperature_average = 0;
float objt_average = 0;
float co2_value_average = 0;

// pressure/temp/humidity sensors
Adafruit_BME280 bme1;        // I2C sensor
Adafruit_BME280 bme2;       // I2C sensor

// This routine is called first

void setup() {

  delay(600);

  // set up serial ports (Serial and Serial1)
  Serial_Set(4);
  Serial_Begin(57600);

#ifdef DEBUGSIMPLE
  Serial_Print_Line("serial works");
#endif

  // Set up I2C bus
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);  // using alternative wire library
  //  Serial_Print_Line("I2C works");

  // initialize SPI bus
  SPI.begin ();
  //  Serial_Print_Line("SPI works");

  // initialize DACs
  DAC_init();

  // set up MCU pins

  //  pinMode(29,OUTPUT);
  //  pinMode(3,INPUT);

  // set up LED on/off pins
  for (int i = 1; i < 11; ++i)
    pinMode(LED_to_pin[i], OUTPUT);

  pinMode(HALL_OUT, INPUT);                                                       // set hall effect sensor to input so it can be read
  pinMode(DEBUG_DC, INPUT);                                                       // leave floating
  pinMode(DEBUG_DD, INPUT);                                                       // leave floating
  //  pinMode(BLERESET, OUTPUT);                                                  // leave floating - deprecated
  //  digitalWriteFast(BLERESET, HIGH);
  //  delay(100);
  //  digitalWriteFast(BLERESET, LOW);                                                // set BLE reset low for the bluetooth to be active
  pinMode(ISET, OUTPUT);                                                          // set USB charge level - deprecated
  digitalWriteFast(ISET, HIGH);
  //  pinMode(ACTINICLIGHT_INTENSITY_SWITCH, OUTPUT);
  //  digitalWriteFast(ACTINICLIGHT_INTENSITY_SWITCH, HIGH);                     // preset the switch to the actinic (high) preset position, DAC channel 0

  // pins used to turn on/off detector integration/discharge
  pinMode(HOLDM, OUTPUT);
  pinMode(HOLDADD, OUTPUT);
  pinMode(BLANK, OUTPUT);                                                            // used as a blank pin to pull high and low if the meas lights is otherwise blank (set to 0)

#if 0
  // ???
  float default_resolution = 488.28;
  int timer0 [8] = {
    5, 6, 9, 10, 20, 21, 22, 23
  };
  int timer1 [2] = {
    3, 4
  };
  int timer2 [2] = {
    25, 32
  };
#endif

  // ??
  analogWriteFrequency(3, 187500);                                              // Pins 3 and 5 are each on timer 0 and 1, respectively.  This will automatically convert all other pwm pins to the same frequency.
  analogWriteFrequency(5, 187500);

#ifdef PULSERDEBUG
  // Set pinmodes for the coralspeq
  //pinMode(SPEC_EOS, INPUT);
  pinMode(SPEC_GAIN, OUTPUT);
  pinMode(SPEC_ST, OUTPUT);
  pinMode(SPEC_CLK, OUTPUT);

  digitalWrite(SPEC_GAIN, LOW);
  digitalWrite(SPEC_ST, HIGH);
  digitalWrite(SPEC_CLK, HIGH);
  digitalWrite(SPEC_GAIN, HIGH); //High Gain
  //digitalWrite(SPEC_GAIN, LOW); //LOW Gain
#endif

  /*NOTES*/  // REINITIATE ONCE MAG AND ACCEL ARE CONNECTED
  //  MAG3110_init();           // initialize compass
  //  MMA8653FC_init();         // initialize accelerometer

#ifdef BME280
  // pressure/humidity/temp sensors
  // note: will need 0x76 or 0x77 to support two chips
  assert (bme.begin(0x76) && bme2.begin(0x77));

#ifdef DEBUGSIMPLE
  Serial_Print("BME280 Temperature = ");
  Serial_Print(bme.readTemperature());
  Serial_Print_Line(" *C");
#endif
#endif

  //  PAR_init();               // color sensor

#ifdef MLX90615
  MLX90615_init();          // IR sensor
#ifdef DEBUGSIMPLE
  Serial.printf("IR temp = %f C\n", MLX90615_Read(0));
#endif
#endif

#undef DEBUGSIMPLE

  // test expressions - works! pass it a string and it is evaluated
  //double expr(const char s[]);
  //Serial_Printf("expr = %f\n",expr("userdef1/2"));  // userdef1 is from eeprom

  assert(sizeof(eeprom_class) < 2048);                    // check that we haven't exceeded eeprom space

  Serial_Print_Line("MultispeQ Ready");

}  // setup()


// ??
void reset_freq() {
  analogWriteFrequency(5, 187500);                                               // reset timer 0
  analogWriteFrequency(3, 187500);                                               // reset timer 1
  analogWriteFrequency(25, 488.28);                                              // reset timer 2
  /*
    Teensy 3.0              Ideal Freq:
    16      0 - 65535       732 Hz          366 Hz
    15      0 - 32767       1464 Hz         732 Hz
    14      0 - 16383       2929 Hz         1464 Hz
    13      0 - 8191        5859 Hz         2929 Hz
    12      0 - 4095        11718 Hz        5859 Hz
    11      0 - 2047        23437 Hz        11718 Hz
    10      0 - 1023        46875 Hz        23437 Hz
    9       0 - 511         93750 Hz        46875 Hz
    8       0 - 255         187500 Hz       93750 Hz
    7       0 - 127         375000 Hz       187500 Hz
    6       0 - 63          750000 Hz       375000 Hz
    5       0 - 31          1500000 Hz      750000 Hz
    4       0 - 15          3000000 Hz      1500000 Hz
    3       0 - 7           6000000 Hz      3000000 Hz
    2       0 - 3           12000000 Hz     6000000 Hz

  */
}

// interrupt service routine which turns the measuring light on

void pulse1() {
#ifdef PULSERDEBUG
  startTimer = micros();
#endif
  digitalWriteFast(LED_to_pin[_meas_light], HIGH);            // turn on measuring light
  delayMicroseconds(10);             // this delay gives the LED current controller op amp the time needed to turn
  // the light on completely + stabilize.
  // Very low intensity measuring pulses may require an even longer delay here.
  digitalWriteFast(HOLDM, LOW);          // turn off sample and hold, and turn on lights for next pulse set
  digitalWriteFast(HOLDADD, LOW);        // turn off sample and hold, and turn on lights for next pulse set
  on = 1;                               // flag for foreground to read
}

// interrupt service routine which turns the measuring light off
// consider merging this into pulse1()

void pulse2() {
#ifdef PULSERDEBUG
  endTimer = micros();
#endif
  digitalWriteFast(LED_to_pin[_meas_light], LOW);
  off = 1;
}

/*
  void call_print_calibration (int _print) {

  // delete all of these
  //  EEPROM_readAnything(0,tmp006_cal_S);
  EEPROM_readAnything(4, light_slope);
  EEPROM_readAnything(8, light_y_intercept);
  EEPROM_readAnything(12, device_id);
  // location 16 - 20 is open!
  EEPROM_readAnything(20, manufacture_date);
  EEPROM_readAnything(24, slope_34);
  EEPROM_readAnything(28, yintercept_34);
  EEPROM_readAnything(32, slope_35);
  EEPROM_readAnything(36, yintercept_35);
  EEPROM_readAnything(40, userdef0);
  EEPROM_readAnything(60, calibration_slope);
  EEPROM_readAnything(180, calibration_yint);
  EEPROM_readAnything(300, calibration_slope_factory);
  EEPROM_readAnything(420, calibration_yint_factory);
  EEPROM_readAnything(540, calibration_baseline_slope);
  EEPROM_readAnything(660, calibration_baseline_yint);
  EEPROM_readAnything(880, calibration_blank1);
  EEPROM_readAnything(1000, calibration_blank2);
  EEPROM_readAnything(1120, calibration_other1);
  EEPROM_readAnything(1240, calibration_other2);
  EEPROM_readAnything(1440, pwr_off_ms);
  }
*/


void set_device_info(const int _set) {
  Serial_Printf("{\"device_name\":\"%s\",\"device_id\":\"%ld\",\"device_firmware\":\"%s\",\"device_manufacture\":\"%d\"}", DEVICE_NAME, eeprom->device_id, FIRMWARE_VERSION, eeprom->manufacture_date);
  Serial_Print_CRC();

  if (_set == 1) {
    // please enter new device ID (lower 4 bytes of BLE MAC address) followed by '+'
    eeprom->device_id = Serial_Input_Long("+", 0);              // save to eeprom
    delay(1);

    // please enter new date of manufacture (yyyymm) followed by '+'
    eeprom->manufacture_date = Serial_Input_Long("+", 0);
    delay(1);

    // print again for verification

    Serial_Printf("{\"device_name\":\"%s\",\"device_id\":\"%ld\",\"device_firmware\":\"%s\",\"device_manufacture\":\"%d\"}", DEVICE_NAME, eeprom->device_id, FIRMWARE_VERSION, eeprom->manufacture_date);
    Serial_Print_CRC();
  } // if

  return;

} // set_device_info()

#ifdef CORAL_SPEQ
void readSpectrometer(int intTime, int delay_time, int read_time, int accumulateMode)
{
  /*
    //int delay_time = 35;     // delay per half clock (in microseconds).  This ultimately conrols the integration time.
    int delay_time = 1;     // delay per half clock (in microseconds).  This ultimately conrols the integration time.
    int idx = 0;
    int read_time = 35;      // Amount of time that the analogRead() procedure takes (in microseconds)
    int intTime = 100;
    int accumulateMode = false;
  */

  // Step 1: start leading clock pulses
  for (int i = 0; i < SPEC_CHANNELS; i++) {
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
  }

  // Step 2: Send start pulse to signal start of integration/light collection
  digitalWrite(SPEC_CLK, LOW);
  delayMicroseconds(delay_time);
  digitalWrite(SPEC_CLK, HIGH);
  digitalWrite(SPEC_ST, LOW);
  delayMicroseconds(delay_time);
  digitalWrite(SPEC_CLK, LOW);
  delayMicroseconds(delay_time);
  digitalWrite(SPEC_CLK, HIGH);
  digitalWrite(SPEC_ST, HIGH);
  delayMicroseconds(delay_time);

  // Step 3: Integration time -- sample for a period of time determined by the intTime parameter
  int blockTime = delay_time * 8;
  int numIntegrationBlocks = (intTime * 1000) / blockTime;
  for (int i = 0; i < numIntegrationBlocks; i++) {
    // Four clocks per pixel
    // First block of 2 clocks -- measurement
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);

    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
  }


  // Step 4: Send start pulse to signal end of integration/light collection
  digitalWrite(SPEC_CLK, LOW);
  delayMicroseconds(delay_time);
  digitalWrite(SPEC_CLK, HIGH);
  digitalWrite(SPEC_ST, LOW);
  delayMicroseconds(delay_time);
  digitalWrite(SPEC_CLK, LOW);
  delayMicroseconds(delay_time);
  digitalWrite(SPEC_CLK, HIGH);
  digitalWrite(SPEC_ST, HIGH);
  delayMicroseconds(delay_time);

  // Step 5: Read Data 2 (this is the actual read, since the spectrometer has now sampled data)
  idx = 0;
  for (int i = 0; i < SPEC_CHANNELS; i++) {
    // Four clocks per pixel
    // First block of 2 clocks -- measurement
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, LOW);

    // Analog value is valid on low transition
    if (accumulateMode == false) {
      spec_data[idx] = analogRead(SPEC_VIDEO);
      spec_data_average[idx] += spec_data[idx];
    } else {
      spec_data[idx] += analogRead(SPEC_VIDEO);
    }
    idx += 1;
    if (delay_time > read_time) delayMicroseconds(delay_time - read_time);   // Read takes about 135uSec

    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);

    // Second block of 2 clocks -- idle
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
  }

  // Step 6: trailing clock pulses
  for (int i = 0; i < SPEC_CHANNELS; i++) {
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delay_time);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delay_time);
  }
}

#endif

#ifdef CORAL_SPEQ
void print_data()
{
  Serial_Print("\"data_raw\":[");
  for (int i = 0; i < SPEC_CHANNELS; i++)
  {
    Serial_Print((int)spec_data[i]);
    if (i != SPEC_CHANNELS - 1) {               // if it's the last one in printed array, don't print comma
      Serial_Print(",");
    }
  }
  Serial_Print("]");
}
#endif

// give it a channel, outputs outputs x,y,z 3 axis data comes out (16 bit)
// perhaps add an option to return raw values or values with calibration applied
void compass (uint16_t array[]) {};

// give it a channel, outputs outputs x,y,z  3 axis data comes out (16 bit)
// perhaps add an option to return raw values or values with calibration applied
void accel (uint16_t array[]) {};

// read this channel from ??, replys with one number (16 bit)
// maybe best to take 33 readings and return the median
uint16_t atod (int channel) {
  return 0;
};


// qsort uint16_t comparison function (tri-state) - needed for median16()

int uint16_cmp(const void *a, const void *b)
{
  const uint16_t *ia = (const uint16_t *)a; // casting pointer types
  const uint16_t *ib = (const uint16_t *)b;

  if (*ia == *ib)
    return 0;

  if (*ia > *ib)
    return 1;
  else
    return -1;
}

// return the median value of an array of 16 bit unsigned values
// note: this also sorts the array
// percentile is normally .50

uint16_t median16(uint16_t array[], const int n, const float percentile)
{
  qsort(array, (size_t) n, sizeof(uint16_t), uint16_cmp);
  return (array[(int) roundf(n * percentile)]);
}


// this routine gets called repeatibly after setup()
#include "loop.h"

