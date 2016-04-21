

// code related to the PAR/color sensor

#include <Arduino.h>
#include "utility/TCS3471.h"              // color sensor
#include "serial.h"
#include "utility/crc32.h"
#include "json/JsonParser.h"
#include "utility/mcp4728.h"              // DAC
#include "eeprom.h"

// external function declarations
void i2cWrite(byte address, byte count, byte* buffer);
void i2cRead(byte address, byte count, byte* buffer);

// external variables
extern int averages;  // ??
mcp4728 dac1 = mcp4728(1); // instantiate mcp4728 object, Device ID = 1

// global variables (should be all static)
float light_intensity;
float light_intensity_averaged;
float light_intensity_raw;
float light_intensity_raw_averaged;
float r;
float r_averaged;
float g;
float g_averaged;
float b;
float b_averaged;
static TCS3471 *par_sensor;


// initialize the PAR/color sensor

void PAR_init()
{
// color sensor init

  par_sensor = new TCS3471(i2cWrite, i2cRead);  

  par_sensor->setWaitTime(200.0);
  par_sensor->setIntegrationTime(700.0);
  par_sensor->setGain(TCS3471_GAIN_1X);
  par_sensor->enable();

  // should use new DAC() routines
  dac1.setVref(1, 1, 1, 1);
  dac1.setGain(0, 0, 0, 0);
  delay(1);
  
}  // PAR_init()

uint16_t par_to_dac (float _par, uint16_t _pin) {                                             // convert dac value to par, in form y = mx + b where y is the dac value
  int dac_value = _par * eeprom->par_to_dac_slope[_pin] + eeprom->par_to_dac_yint[_pin];
  return dac_value;
}

float light_intensity_raw_to_par (float _light_intensity_raw, float _r, float _g, float _b) {
  int par_value = eeprom->light_slope_all * _light_intensity_raw + _r * eeprom->light_slope_r + _g * eeprom->light_slope_g + _b * eeprom->light_slope_b + eeprom->light_yint;
  return par_value;
}

int get_light_intensity(int notRaw,int _averages) {

  r = par_sensor->readRData();
  g = par_sensor->readGData();
  b = par_sensor->readBData();
  light_intensity_raw = par_sensor->readCData();
  light_intensity = light_intensity_raw_to_par(light_intensity_raw,r,g,b);

    r_averaged += r / _averages;
    g_averaged += g / _averages;
    b_averaged += b / _averages;
  if (notRaw == 0) {
    light_intensity_raw_averaged += light_intensity_raw / _averages;
  }
  else if (notRaw == 1) {
    light_intensity_averaged += light_intensity / _averages;
  }
  return light_intensity;
} 

