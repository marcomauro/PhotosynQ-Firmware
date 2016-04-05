

// code related to the PAR/color sensor

#include <Arduino.h>
#include "Adafruit_TCS34725.h"    // color sensor
#include "TCS3471.h"              // color sensor
#include "serial.h"
#include "crc32.h"
#include "JsonParser.h"
#include "mcp4728.h"              // DAC
#include "EEPROMAnything.h"
#include "eeprom.h"

// external function declarations
void i2cWrite(byte address, byte count, byte* buffer);
void i2cRead(byte address, byte count, byte* buffer);
void call_print_calibration (int _print);
double user_enter_dbl(long timeout);


// external variables
extern int act_intensity;
extern int meas_intensity;
extern int cal_intensity;
extern JsonArray act_intensities;                         // write to input register of a dac1. channel 0 for low (actinic).  1 step = +3.69uE (271 == 1000uE, 135 == 500uE, 27 == 100uE)
extern JsonArray meas_intensities;                        // write to input register of a dac1. channel 3 measuring light.  0 (high) - 4095 (low).  2092 = 0.  From 2092 to zero, 1 step = +.2611uE
extern JsonArray cal_intensities;                        // write to input register of a dac1. channel 2 calibrating light.  0 (low) - 4095 (high).
extern int averages;  // ??
mcp4728 dac1 = mcp4728(1); // instantiate mcp4728 object, Device ID = 1
#define NUM_PINS 26
extern float all_pins [NUM_PINS];
extern float calibration_slope [NUM_PINS]; 
extern float calibration_yint [NUM_PINS];
extern float calibration_slope_factory [NUM_PINS];
extern float calibration_yint_factory [NUM_PINS]; 
extern float calibration_baseline_slope [NUM_PINS];
extern float calibration_baseline_yint [NUM_PINS]; 
extern float calibration_blank1 [NUM_PINS];
extern float calibration_blank2 [NUM_PINS]; 
extern float calibration_other1 [NUM_PINS];
extern float calibration_other2 [NUM_PINS];

// forward declarations
float lux_to_uE(float _lux_average);
void calibrate_light_sensor();
int uE_to_intensity(int _pin, int _uE);

// global variables (should be all static)
float light_slope = 3;
float lux_local = 0;
float r_local = 0;
float g_local = 0;
float b_local = 0;
float lux_average = 0;
float r_average = 0;
float g_average = 0;
float b_average = 0;
float lux_average_forpar = 0;
float r_average_forpar = 0;
float g_average_forpar = 0;
float b_average_forpar = 0;
float light_y_intercept = 0;
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


// ??
void calibrate_light_sensor() {

  call_print_calibration(1);

  light_slope = user_enter_dbl(60000);
  EEPROM_writeAnything(4, light_slope);

  light_y_intercept = user_enter_dbl(60000);
  EEPROM_writeAnything(8, light_y_intercept);

  call_print_calibration(1);
}


// ??
int Light_Intensity(int var1) {

  lux_local = par_sensor->readCData();                  // take 3 measurements, outputs in format - = 65535 or whatever 16 bits is
  r_local = par_sensor->readRData();
  g_local = par_sensor->readGData();
  b_local = par_sensor->readBData();

#ifdef DEBUGSIMPLE
  Serial_Print("\"light_intensity\": ");
  Serial_Print(lux_local, DEC);
  Serial_Print(",");
  Serial_Print("\"red\": ");
  Serial_Print(r_local, DEC);
  Serial_Print(",");
  Serial_Print("\"green\": ");
  Serial_Print(g_local, DEC);
  Serial_Print(",");
  Serial_Print("\"blue\": ");
  Serial_Print(b_local, DEC);
  Serial_Print(",");
  //  Serial_Print("\cyan\": ");
  //  Serial_Print(c, DEC);
#endif
  if (var1 == 0) {
    lux_average += lux_local / averages;
    r_average += r_local / averages;
    g_average += g_local / averages;
    b_average += b_local / averages;
  }
  if (var1 == 1) {
    lux_average_forpar += lux_local / averages;
    r_average_forpar += r_local / averages;
    g_average_forpar += g_local / averages;
    b_average_forpar += b_local / averages;
  }
  return lux_local;
}


int calculate_intensity(int _light, int tcs_on, int _cycle, float _light_intensity) {
#ifdef DEBUGSIMPLE
  Serial_Print("calculate intensity vars _light, tcs_on, _light_intensity, _tcs, cycle, act_intensities.getLong(_cycle), meas_intensities.getLong(_cycle), cal_intensities.getLong(_cycle)   ");
  Serial_Print(",");
  Serial_Print(_light);
  Serial_Print(",");
  Serial_Print(tcs_on);
  Serial_Print(",");
  Serial_Print(_cycle);
  Serial_Print(",");
  Serial_Print(_light_intensity);
  Serial_Print(",");
  Serial_Print(act_intensities.getLong(_cycle));
  Serial_Print(",");
  Serial_Print(meas_intensities.getLong(_cycle));
  Serial_Print(",");
  Serial_Print_Line(cal_intensities.getLong(_cycle));
#endif

  int on = 0;                                                                            // so identify the places to turn the light on by flipping this to 1
  int _tcs = 0;
  if (_light == 2 || _light == 20) {                                                      // if it's a saturating light, and...
    if (act_intensities.getLong(_cycle) > 0) {                                            // if the actinic intensity is greater than zero then...
      on = 1;
      act_intensity = act_intensities.getLong(_cycle);                                    // turn light on and set intensity equal to the intensity specified in the JSON
    }
    else if (act_intensities.getLong(_cycle) < 0 && tcs_on > 0 && _light_intensity > 0) {   // if the intensity is -1 AND tcs_to_act is on AND the uE value _tcs_to_act is > 0 (ie ambient light is >0)
      on = 1;
      _tcs = (uE_to_intensity(_light, _light_intensity) * tcs_on) / 100;
      act_intensity = _tcs;                                                                 // then turn light on, and set intensity to ambient
      //      Serial_Print_Line(_light);
      //      Serial_Print_Line(_tcs);
    }
  }

  else if (_light == 15 || _light == 16 || _light == 11 || _light == 12) {                     // if it's a measuring light, and...
    if (meas_intensities.getLong(_cycle) > 0) {                                            // if the actinic intensity is greater than zero then...
      on = 1;
      meas_intensity = meas_intensities.getLong(_cycle);                                    // turn light on and set intensity equal to the intensity specified in the JSON
    }
    else if (meas_intensities.getLong(_cycle) < 0 && tcs_on > 0 && _light_intensity > 0) {      // if the intensity is -1 AND tcs_to_act is on AND the uE value _tcs_to_act is > 0 (ie ambient light is >0)
      on = 1;
      _tcs = (uE_to_intensity(_light, _light_intensity) * tcs_on) / 100;
      meas_intensity = _tcs;                                                                 // then turn light on, and set intensity to ambient
      //      Serial_Print_Line(_light);
      //      Serial_Print_Line(_tcs);
    }
  }

  else if (_light == 14 || _light == 10) {                                                   // if it's a calibrating light, and...
    if (cal_intensities.getLong(_cycle) > 0) {                                            // if the actinic intensity is greater than zero then...
      on = 1;
      cal_intensity = cal_intensities.getLong(_cycle);                                    // turn light on and set intensity equal to the intensity specified in the JSON
    }
    else if (cal_intensities.getLong(_cycle) < 0 && tcs_on > 0 && _light_intensity > 0) {      // if the intensity is -1 AND tcs_to_act is on AND the uE value _tcs_to_act is > 0 (ie ambient light is >0)
      on = 1;
      cal_intensity = _tcs;                                                       // then turn light on, and set intensity to ambient
    }
  }
  return on;
}


int calculate_intensity_background(int _light, int tcs_on, int _cycle, float _light_intensity, int _background_intensity) { // calculate intensity of background light if it's set at a constant level by the user
#ifdef DEBUGSIMPLE
  Serial_Print("calculate background intensity vars _light, tcs_on, _light_intensity, _tcs, cycle, act_intensities.getLong(_cycle), meas_intensities.getLong(_cycle), cal_intensities.getLong(_cycle)   ");
  Serial_Print(",");
  Serial_Print(_light);
  Serial_Print(",");
  Serial_Print(tcs_on);
  Serial_Print(",");
  Serial_Print(_cycle);
  Serial_Print(",");
  Serial_Print(_light_intensity);
  Serial_Print(",");
  Serial_Print(act_intensities.getLong(_cycle));
  Serial_Print(",");
  Serial_Print(meas_intensities.getLong(_cycle));
  Serial_Print(",");
  Serial_Print(cal_intensities.getLong(_cycle));
#endif
  int on = 0;   // so identify the places to turn the light on by flipping this to 1
  int _tcs = 0;

  if (_light == 2 || _light == 20) {                                                      // if it's a saturating light, and...
    if (_background_intensity > 0) {                                            // if actinic background intensity is preset then
      dac1.analogWrite(0, _background_intensity);                                // set the actinic to that value
      on = 1;                                                                            // so identify the places to turn the light on by flipping this to 1
    }
    else if (tcs_on > 0 && _light_intensity > 0) {                                       // or if tcs_to_act is on and ambient light is greater than zero then...
      _tcs = (uE_to_intensity(_light, _light_intensity) * tcs_on) / 100;
      //      Serial_Print_Line(_light);
      //      Serial_Print_Line(_tcs);
      dac1.analogWrite(0, _tcs);                                                          // set the actinic to that value
      on = 1;                                                                            // so identify the places to turn the light on by flipping this to 1
    }
  }
  else if (_light == 15 || _light == 16 || _light == 11 || _light == 12) {                       // if it's a measuring light, and...
    if (_background_intensity > 0) {                                            // if actinic background intensity is preset then
      dac1.analogWrite(3, _background_intensity);                                // set the actinic to that value
      on = 1;                                                                            // so identify the places to turn the light on by flipping this to 1
    }
    else if (tcs_on > 0 && _light_intensity > 0) {                                       // or if tcs_to_act is on and ambient light is greater than zero then...
      _tcs = (uE_to_intensity(_light, _light_intensity) * tcs_on) / 100;
      //      Serial_Print_Line(_light);
      //      Serial_Print_Line(_tcs);
      dac1.analogWrite(3, _tcs);                                                          // set the actinic to that value
      on = 1;                                                                            // so identify the places to turn the light on by flipping this to 1
    }
  }
  else if (_light == 14 || _light == 10) {                                                     // if it's a calibrating light, and...
    if (_background_intensity > 0) {                                            // if actinic background intensity is preset then
      dac1.analogWrite(2, _background_intensity);                                // set the actinic to that value
      on = 1;                                                                            // so identify the places to turn the light on by flipping this to 1
    }
    else if (tcs_on > 0 && _light_intensity > 0) {                                       // or if tcs_to_act is on and ambient light is greater than zero then...
      dac1.analogWrite(2, _tcs);                                                          // set the actinic to that value
      on = 1;                                                                            // so identify the places to turn the light on by flipping this to 1
    }
  }
  return on;
}

float lux_to_uE(float _lux_average) {                                                      // convert the raw signal value to uE, based on a calibration curve
  float uE = (_lux_average - light_y_intercept) / light_slope;
#ifdef DEBUGSIMPLE
  Serial_Print(_lux_average);
  Serial_Print(",");
  Serial_Print(light_y_intercept);
  Serial_Print(",");
  Serial_Print(light_slope);
  Serial_Print(",");
  Serial_Print_Line(uE);
#endif
  return uE;
}

int uE_to_intensity(int _pin, int _uE) {                                                  // convert PAR value from ambient uE to LED intensity, based on a specific chosen LED
  float _slope = 0;
  float _yint = 0;
  float intensity_drift_slope = 0;
  float intensity_drift_yint = 0;
  unsigned int _intensity = 0;
  for (unsigned i = 0; i < sizeof(all_pins) / sizeof(int); i++) {                                              // loop through all_pins
    if (all_pins[i] == _pin) {                                                                        // when you find the pin your looking for
      intensity_drift_slope = (calibration_slope_factory[i] - calibration_slope[i]) / calibration_slope_factory[i];
      intensity_drift_yint = (calibration_yint_factory[i] - calibration_yint[i]) / calibration_yint_factory[i];
      _slope = calibration_other1[i] + calibration_other1[i] * intensity_drift_slope;                                                              // go get the calibration slope and yintercept, multiply by the intensity drift
      _yint = calibration_other2[i] + calibration_other2[i] * intensity_drift_yint;
      break;
    }
  }
  if (_slope != 0 || _yint != 0) {                                                                      // if calibration values exist then...
    _intensity = (_uE - _yint) / _slope;                                                                // calculate the resulting intensity DAC value
  }
#ifdef DEBUGSIMPLE
  Serial_Print("uE, slope, yint, act_background pin, DAC intensity:   ");
  Serial_Print(_uE);
  Serial_Print(",");
  Serial_Print(_slope);
  Serial_Print(",");
  Serial_Print(_yint);
  Serial_Print(",");
  Serial_Print_Line(_intensity);
#endif
  return _intensity;
}



