
// Global defines and some misc functions

#include <Arduino.h>
#include <stdint.h>

//#define DEBUG 1         // uncomment to add full debug features
const int DEBUGSIMPLE= 0;   // uncomment to add partial debug features
//#define DAC 1           // uncomment for boards which do not use DAC for light intensity control
const int PULSERDEBUG=0;   // uncomment to debug the pulser and detector
//#define NO_ADDON        // uncomment if add-on board isn't present (one missing DAC, etc)
#define CORAL_SPEQ 0

// FIRMWARE VERSION OF THIS FILE (SAVED TO EEPROM ON FIRMWARE FLASH)
#define DEVICE_FIRMWARE "0.54"
#define DEVICE_NAME "MultispeQ"
#define DEVICE_VERSION "1"
     
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
// 1-5 on main board, 6-10 on add-on board
// document colors
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


// use this to store values to eeprom 
#define store_float(location, value)   { float f = value;  if (eeprom->location != f) eeprom->location = f; delay(1); }


const unsigned NUM_LEDS=10;
// map LED (1-10) to MCU pin
const uint8_t LED_to_pin[NUM_LEDS + 1] = {0, PULSE1, PULSE2, PULSE3, PULSE4, PULSE5, PULSE6, PULSE7, PULSE8, PULSE9, PULSE10 }; // NOTE!  We skip the first element in the array so that the array lines up correctly (PULSE1 == 1, PULSE2 == 2 ... )

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

// Functions

#include <stdint.h>

void activity(void);
void powerdown(void);
uint16_t median16(uint16_t array[], const int n, const float percentile = .50);
float stdev16(uint16_t array[], const int n);
int check_protocol(char *str);


// use this to store values to eeprom 
#define store_float(location, value)   { float f = value;  if (eeprom->location != f) eeprom->location = f; delay(1); }

/*
 * Sensor-related variables which are accessed in loop
 */

#ifndef EXTERN
#define EXTERN extern
#endif

EXTERN float light_intensity;
EXTERN float light_intensity_averaged;
EXTERN float light_intensity_raw;
EXTERN float light_intensity_raw_averaged;
EXTERN float r;
EXTERN float r_averaged;
EXTERN float g;
EXTERN float g_averaged;
EXTERN float b;
EXTERN float b_averaged;

EXTERN float thickness;
EXTERN float thickness_averaged;
EXTERN int thickness_raw;
EXTERN float thickness_raw_averaged;

EXTERN float contactless_temp;
EXTERN float contactless_temp_averaged;

EXTERN int cardinal;
EXTERN int cardinal_averaged;
EXTERN int x_cardinal_raw, y_cardinal_raw, z_cardinal_raw;
EXTERN float x_cardinal_raw_averaged, y_cardinal_raw_averaged, z_cardinal_raw_averaged;

EXTERN float x_tilt, y_tilt, z_tilt;
EXTERN float x_tilt_averaged, y_tilt_averaged, z_tilt_averaged;
EXTERN int x_tilt_raw, y_tilt_raw, z_tilt_raw;
EXTERN float x_tilt_raw_averaged, y_tilt_raw_averaged, z_tilt_raw_averaged;

EXTERN float temperature, humidity, pressure;

// Coral SpeQ
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
EXTERN uint16_t spec_data[SPEC_CHANNELS];
EXTERN unsigned long spec_data_average[SPEC_CHANNELS];            // saves the averages of each spec measurement
EXTERN int idx;

#undef EXTERN
