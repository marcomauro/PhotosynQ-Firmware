
// reasonably generic utility functions
// put function prototypes in util.h

#include "defines.h"
#include "eeprom.h"
#include "utility/crc32.h"
#include "DAC.h"
#include "util.h"
#include "serial.h"

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

const unsigned long SHUTDOWN = 60000;   // power down after X ms of inactivity
static unsigned long last_activity = millis();

// record that we have seen serial port activity (used with powerdown())
void activity() {
  last_activity = millis();
}

// if not on USB and there hasn't been any activity for x seconds, then power down BLE and sleep

void powerdown() {

  if ((millis() - last_activity > SHUTDOWN && !Serial) || battery_low(0)) {   // if USB is active, no timeout sleep
#define LEGACY
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

      if (accel_changed()) {
        if (battery_low(0)) {
          sleep_mode(60000);    // longer sleep for low bat
          continue;
        } else
          break;
      } // if
    } // for

    // turn on BLE
    // TODO

    activity();    // save currrent time

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
  //delay(1);      // maybe some time is needed before everything works?
  // restore time from RTC?

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


//apply the calibration values for magnetometer from EEPROM
void applyMagCal(float * arr) {

  arr[0] -= eeprom->mag_bias[0];
  arr[1] -= eeprom->mag_bias[1];
  arr[2] -= eeprom->mag_bias[2];

  float tempY = arr[0] * eeprom->mag_cal[0][0] + arr[1] * eeprom->mag_cal[0][1] + arr[2] * eeprom->mag_cal[0][2];
  float tempX = arr[0] * eeprom->mag_cal[1][0] + arr[1] * eeprom->mag_cal[1][1] + arr[2] * eeprom->mag_cal[1][2];
  float tempZ = arr[0] * eeprom->mag_cal[2][0] + arr[1] * eeprom->mag_cal[2][1] + arr[2] * eeprom->mag_cal[2][2];

  arr[0] = tempX;
  arr[1] = tempY;
  arr[2] = tempZ;

  arr[1] *= -1;
  arr[0] *= -1;
}

//return compass heading (RADIANS) given pitch, roll and magentotmeter measurements
float getCompass(const float magX, const float magY, const float magZ, const float & pitch, const float & roll) {
  float negBfy = magZ * sine_internal(roll) - magY * cosine_internal(roll);
  float Bfx = magX * cosine_internal(pitch) + magY * sine_internal(roll) * sine_internal(pitch) + magZ * sine_internal(pitch) * cosine_internal(roll);

  return atan2(negBfy, Bfx);
}

//return roll (RADIANS) from accelerometer measurements
float getRoll(const int accelY, const int accelZ) {
  return atan2(accelY, accelZ);
}

//return pitch (RADIANS) from accelerometer measurements + roll
float getPitch(const int accelX, const int accelY, const int accelZ, const float & roll) {
  return atan(-1 * accelX / (accelY * sine_internal(roll) + accelZ * cosine_internal(roll)));
}

// return 0-7 based on 8 segments of the compass

int compass_segment(float angle)    // in degrees, assume no negatives
{
  return (round( angle / 45) % 8);
}

//get the direction (N/S/E/W/NW/NE/SW/SE) from the compass heading
String getDirection(int segment) {

  if (segment > 7 || segment < 0) {
    return "Invalid compass segment";
  }

  String names[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};

  return "\"" + names[segment] + "\"";
}

//calculate tilt angle and tilt direction given roll, pitch, compass heading
Tilt calculateTilt(const float & roll, const float & pitch, float compass) {

  compass *= 180 / PI;

  Tilt deviceTilt;

  //equation derived from rotation matricies in AN4248 by Freescale
  float a = (cosine_internal(roll) * cosine_internal(pitch));
  float b = sqrt((sine_internal(roll) * sine_internal(roll) + (sine_internal(pitch) * sine_internal(pitch) * cosine_internal(roll) * cosine_internal(roll))));
  deviceTilt.angle = atan2(a, b);

  deviceTilt.angle *= 180 / PI;

  deviceTilt.angle  = 90 - deviceTilt.angle;

  if (0 <= compass && compass <= 360) {
    deviceTilt.angle_direction = "\"Invalid compass heading\"";
  }

  float tilt_angle = atan2((sine_internal(roll)), cosine_internal(roll) * sine_internal(pitch));
  tilt_angle *= 180 / PI;

  if (tilt_angle < 0) {
    tilt_angle += 360;
  }


  int tilt_segment = compass_segment(tilt_angle);

  int comp_segment = compass_segment(compass) + tilt_segment;
  comp_segment = comp_segment % 8;

  deviceTilt.angle_direction = getDirection(comp_segment);

  return deviceTilt;
}

//Internal sine calculation in RADIANS
float sine_internal(float angle) {
  if (angle > PI) {
    angle -= 2 * PI;
  }

  return angle - angle * angle * angle / 6 + 
  angle * angle * angle * angle * angle / 120 - 
  angle * angle * angle * angle * angle * angle * angle / 5040 +
  angle * angle * angle * angle * angle * angle * angle * angle * angle / 362880;
}

//Internal cosine calculation in RADIANS
float cosine_internal(float angle) {
  if (angle > PI) {
    angle -= 2 * PI;
  }

  return 1 - angle * angle / 2 + angle * angle * angle * angle / 24 - 
  angle * angle * angle * angle * angle * angle  / 720 +
  angle * angle * angle * angle * angle * angle * angle * angle / 40320 - 
  angle * angle * angle * angle * angle * angle * angle * angle * angle * angle / 3628800;
}
//this arctan approximation only works for -pi/4 to pi/4 - can be modified for that to work, but atan2 and atan
//only takes up <2kb, if we need the space I'll fix it but otherwise I'll leave the originals in place
/*
//Internal arctangent calculation in RADIANS
float arctan_internal(float x, float y){
  float small, large;
  int sign = 1;
  if((x < 0 && y > 0) || (x > 0 && y < 0)){
    sign *= -1;
    (x < 0) ? x *= -1 : y *= -1;
  }

  large = float_max(x, y);
  small = float_min(x, y);
  
  float angle = small / large;

  return PI / 4 * angle - angle * (angle - 1) * (0.2447 + 0.0663 * angle);
}

float float_max(float x, float y){
  return (x > y) ? x : y;
}

float float_min(float x, float y){
  return (x < y) ? x : y;
}
*/
