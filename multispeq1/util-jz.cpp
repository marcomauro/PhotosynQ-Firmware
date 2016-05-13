
// If your name isn't Jon, don't touch this file

// reasonably generic utility functions
// put function prototypes in util.h

#include "defines.h"
#include "eeprom.h"
#include "utility/crc32.h"
#include "DAC.h"
#include "util.h"
#include "serial.h"

unsigned int read_once(unsigned char address);
void program_once(unsigned char address, unsigned int value);

int jz_test_mode = 0;

// qsort uint16_t comparison function (tri-state) - needed for median16()

static int uint16_cmp(const void *a, const void *b)
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


// return the stdev value of an array of 16 bit unsigned values

float stdev16(uint16_t val[], const int count)
{
  // calc stats
  double mean = 0, delta = 0, m2 = 0, variance = 0, stdev = 0, n = 0;

  for (int i = 0; i < count; i++) {
    ++n;
    delta = val[i] - mean;
    mean += delta / n;
    m2 += (delta * (val[i] - mean));
  } // for
  variance = m2 / (count - 1);  // (n-1):Sample Variance  (n): Population Variance
  stdev =  sqrt(variance);        // Calculate standard deviation
  //Serial_Printf("single pulse stdev = %.2f, mean = %.2f AD counts\n", stdev, mean);
  //Serial_Printf("bits (95%%) = %.2f\n", (15 - log(stdev * 2) / log(2.0))); // 2 std dev from mean = 95%
  return stdev;
} // stdev16()


// check a json protocol for validity (matching [] and {})
// return 1 if OK, otherwise 0
// also check CRC value if present

int check_protocol(char *str)
{
  int bracket = 0, curly = 0;
  char *ptr = str;

  while (*ptr != 0) {
    switch (*ptr) {
      case '[':
        ++bracket;
        break;
      case ']':
        --bracket;
        break;
      case '{':
        ++curly;
        break;
      case '}':
        --curly;
        break;
    } // switch
    ++ptr;
  } // while

  if (bracket != 0 || curly != 0)  // unbalanced - can't be correct
    return 0;

  // check CRC - 8 hex digits immediately after the closing ]
  ptr = strrchr(str, ']'); // find last ]
  if (!ptr)                        // no ] found - how can that be?
    return 0;

  ++ptr;                      // char after last ]

  if (!isxdigit(*(ptr)))      // hex digit follows last ] ?
    return 1;                    // no CRC so report OK

  // CRC is there - check it

  crc32_init();     // find crc of json (from first [ to last ])
  crc32_buf (str, ptr - str);

  // note: must be exactly 8 upper case hex digits
  if (strncmp(int32_to_hex (crc32_value()), ptr, 8) != 0) {
    return 0;                 // bad CRC
  }

  *ptr = 0;                   // remove the CRC

  return 1;                   // CRC is OK
} // check_protocol()



// Battery check: Calculate battery output based on flashing the 4 IR LEDs at 250 mA each for 10uS.
// This should run just before any new protocol - if it’s too low, report to the user
// return 1 if low, otherwise 0
// for flash == 0, make no assumptions about pins being initialized

const float MIN_BAT_LEVEL (3.4 * (16. / (16 + 47)) * (65536 / 1.2)); // 3.4V min battery voltage, voltage divider, 1.2V reference, 16 bit ADC

int battery_low(int flash)         // 0 for no load, 1 to flash LEDs to create load
{
  return 0;

  // enable bat measurement
  pinMode(BATT_ME, OUTPUT);
  digitalWriteFast(BATT_ME, LOW);
  delay(20);

  // find voltage before high load
  uint32_t initial_value = 0;
  uint32_t value;

  for (int i = 0 ; i < 100; ++i)
    initial_value += analogRead(BATT_TEST);  // test A10 analog input
  initial_value /= 100;

  value = initial_value;

  if (flash) {   // flash LEDs if needed to create load
    // set DAC values to 1/4 of full output to create load
    DAC_set(1, 4096 / 4);
    DAC_set(2, 4096 / 4);
    DAC_set(5, 4906 / 4);
    DAC_set(6, 4906 / 4);
    DAC_change();
    delay(1);       // stabilize

    // turn on 4 LEDs
    digitalWriteFast(PULSE1, 1);
    digitalWriteFast(PULSE2, 1);
    digitalWriteFast(PULSE5, 1);
    digitalWriteFast(PULSE6, 1);

    delay(20);          // there a slow filter on the circuit

    // value after load
    value = 0;
    for (int i = 0 ; i < 100; ++i)
      value += analogRead(BATT_TEST);  // test A10 analog input
    value /= 100;

    // turn off 4 LEDs
    digitalWriteFast(PULSE1, 0);
    digitalWriteFast(PULSE2, 0);
    digitalWriteFast(PULSE5, 0);
    digitalWriteFast(PULSE6, 0);

    // turn off BATT_ME (let float)
    pinMode(BATT_ME, INPUT);

    //Serial_Printf("bat = %d counts %fV\n", value, value * (1.2 / 65536));

    // TODO - make use of intial value?
  } // if

  if (value  < MIN_BAT_LEVEL)
    return 1;                  // too low
  else
    return 0;  // OK

} // battery_low()


// return 1 if the accelerometer values haved changed
#define ACCEL_CHANGE 100
void MMA8653FC_read(int *axeXnow, int *axeYnow, int *axeZnow);

int accel_changed()
{
  int x, y, z;
  static int prev_x, prev_y, prev_z;
  int changed = 0;

  MMA8653FC_read(&x, &y, &z);

  if (abs(x - prev_x) > ACCEL_CHANGE || abs(y - prev_y) > ACCEL_CHANGE || abs(z - prev_z) > ACCEL_CHANGE)
    changed = 1;

  prev_x = x;        // always update this so slow change won't trigger it
  prev_y = y;
  prev_z = z;

  return changed;
}  // accel_changed()

const unsigned long SHUTDOWN = 90000;   // power down after X ms of inactivity
static unsigned long last_activity = millis();

// record that we have seen serial port activity (used with powerdown())
void activity() {
  last_activity = millis();
}

// if not on USB and there hasn't been any activity for x seconds, then power down BLE and sleep

void powerdown() {

  //return;    // this is still experimental
  
  if ((millis() - last_activity > SHUTDOWN /* && !Serial */) || battery_low(0)) {   // if USB is active, no timeout sleep

#ifdef LEGACY
    pinMode(POWERDOWN_REQUEST, OUTPUT);               // legacy: ask BLE to power down MCU (active low)
    digitalWriteFast(POWERDOWN_REQUEST, LOW);
#endif

    // turn off BLE, turn off analog circuitry (should already be off), then enter a sleep loop
    // TODO

    // wake up if the device has changed orientation
    accel_changed();     // update values with current
    
    for (;;) {
      sleep_mode(2000);

      if (accel_changed()) {    // note: accelerometer doesn't seem to need any initialization after being turned off then on
        if (battery_low(0)) {
          sleep_mode(60000);    // longer sleep for low bat
          continue;
        } else
          break;
      } // if
    } // for

    // note, peripherals are now in an unknown state

    // calling setup() might also work
    // reboot to turn BLE on and re-intialize peripherals
#define CPU_RESTART_ADDR ((uint32_t *)0xE000ED0C)
#define CPU_RESTART_VAL 0x5FA0004
    *CPU_RESTART_ADDR = CPU_RESTART_VAL;

  } // if
}  // powerdown()


#define USE_HIBERNATE  // doesn't work, you have to edit the library source code
#include <Snooze.h>

static SnoozeBlock config;

// enter sleep mode for n milliseconds
void sleep_mode(const int n)
{
  // Set Low Power Timer wake up in milliseconds.
  config.setTimer(n);      // milliseconds

  Snooze.deepSleep( config );
  //    Snooze.hibernate( config );

} // sleep_mode()


// print message for every I2C device on the bus
// original author unknown

#include <i2c_t3.h>

void scan_i2c(void)
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

} // scan_i2c()

//======================================

// read/write device_id and manufacture_date to eeprom

void get_set_device_info(const int _set) {

  if (_set == 1) {
    long val;

    // please enter new device ID (lower 4 bytes of BLE MAC address as a long int) followed by '+'
    Serial_Print_Line("{\"message\": \"Please enter device mac address (long int) followed by +: \"}\n");
    val =  Serial_Input_Long("+", 0);              // save to eeprom
    store(device_id, val);              // save to eeprom

    // please enter new date of manufacture (yyyymm) followed by '+'
    Serial_Print_Line("{\"message\": \"Please enter device manufacture date followed by + (example 052016): \"}\n");
    val = Serial_Input_Long("+", 0);
    store(device_manufacture, val);

  } // if

  // print
  Serial_Printf("{\"device_name\":\"%s\",\"device_version\":\"%s\",\"device_id\":\"d4:f5:%2.2x:%2.2x:%2.2x:%2.2x\",\"device_firmware\":\"%s\",\"device_manufacture\":%6.6d}", DEVICE_NAME, DEVICE_VERSION,
                (unsigned)eeprom->device_id >> 24,
                ((unsigned)eeprom->device_id & 0xff0000) >> 16,
                ((unsigned)eeprom->device_id & 0xff00) >> 8,
                (unsigned)eeprom->device_id & 0xff,
                DEVICE_FIRMWARE, eeprom->device_manufacture);
  Serial_Print_CRC();

  return;

} // get_set_device_info()

// ======================================

