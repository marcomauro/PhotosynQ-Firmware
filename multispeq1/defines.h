
// Global defines and some misc functions


//#define DEBUG 1         // uncomment to add full debug features
//#define DEBUGSIMPLE 1   // uncomment to add partial debug features
//#define DAC 1           // uncomment for boards which do not use DAC for light intensity control
//#define PULSERDEBUG 1   // uncomment to debug the pulser and detector
//#define NO_ADDON        // uncomment if add-on board isn't present (one missing DAC, etc)


// FIRMWARE VERSION OF THIS FILE (SAVED TO EEPROM ON FIRMWARE FLASH)
#define FIRMWARE_VERSION "0.50"
#define DEVICE_NAME "MultispeQ"
      
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


// map LED (1-10) to MCU pin number
#define NUM_LEDS 10
extern unsigned short LED_to_pin[NUM_LEDS+1];

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

