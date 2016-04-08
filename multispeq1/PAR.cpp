

// code related to the PAR/color sensor

#include <Arduino.h>
#include "utility/Adafruit_TCS34725.h"    // color sensor
#include "utility/TCS3471.h"              // color sensor
#include "serial.h"
#include "crc32.h"
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


