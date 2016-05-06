
// main loop and some support routines

#define EXTERN
#include "defines.h"             // various globals                                                         // enable real time clock library
#include "json/JsonParser.h"
#include "DAC.h"
#include "AD7689.h"               // external ADC
#include "eeprom.h"
#include "serial.h"
#include "flasher.h"
#include "utility/crc32.h"
#include <TimeLib.h>

// function declarations

inline static void startTimers(unsigned _pulsedistance);
inline static void stopTimers(void);
void reset_freq(void);
void upgrade_firmware(void);            // for over-the-air firmware updates
void boot_check(void);                  // for over-the-air firmware updates
int get_light_intensity(int notRaw, int x);
static void recall_save(JsonArray _recall_eeprom, JsonArray _save_eeprom);
void get_set_device_info(const int _set);
int abort_cmd(void);
static void environmentals(JsonArray a, const int _averages, const int x, int beforeOrAfter);
void readSpectrometer(int intTime, int delay_time, int read_time, int accumulateMode);
void MAG3110_read (int *x, int *y, int *z);
void MMA8653FC_read(int *axeXnow, int *axeYnow, int *axeZnow);
float MLX90615_Read(int TaTo);
uint16_t par_to_dac (float _par, uint16_t _pin);
float light_intensity_raw_to_par (float _light_intensity_raw, float _r, float _g, float _b);
static void print_all (void);
static void print_userdef (void);
double expr(const char str[]);
void do_protocol(void);
void do_command(void);
int battery_low(void);
void print_calibrations(void);

//////////////////////// MAIN LOOP /////////////////////////

// process ascii serial input commands of two forms:
// 1010+<parameter1>+<parameter2>+...  (a command)
// [...] (a json protocol to be executed)

void loop() {

  // read and process n+ commands from the serial port until we see the start of a json

  for (;;) {
    int c = Serial_Peek();

    powerdown();            // power down if no activity for x seconds (could also be a timer interrupt)

    if (c == -1)
      continue;             // nothing available, try again

    activity();             // record fact that we have seen activity (used with powerdown())

    crc32_init();

    if (c == '[')
      break;                // start of json, exit this for loop to process it

    // received a non '[' char - processs n+ command

    do_command();

    Serial_Flush_Output();

  } // for

  // here if not a + command

  // read in and process a protocol (starts with '[', ends with '!' or timeout)
  // example: [{"pulses": [150],"a_lights": [[3]],"a_intensities": [[50]],"pulsedistance": 1000,"m_intensities": [[125]],"pulsesize": 2,"detectors": [[3]],"meas_lights": [[1]],"protocols": 1}]<newline>

  do_protocol();

  Serial_Flush_Output();

  return;

} // loop()

// =========================================

// globals - try to avoid
static uint8_t _meas_light;                    // measuring light to be used during the interrupt
static uint16_t _pulsesize = 0;         // pulse width in usec
static volatile uint8_t led_off = 0;    // status of LED set by ISR

// process a numeric + command

void do_command()
{
  char choose[50];
  Serial_Input_Chars(choose, "+", 500, sizeof(choose) - 1);

  if (strlen(choose) < 3) {        // short or null command, quietly ignore it
    return;
  }

  if (isprint(choose[0]) && !isdigit(choose[0])) {
    Serial_Print("{\"error\":\"commands must be numbers or json\"}\n");
    return;                     // go read another command
  }

  // process + command

  switch (atoi(choose)) {
    case 1000:                                                                    // print "Ready" to USB and Bluetooth
      Serial_Print(DEVICE_NAME);
      Serial_Print_Line(" Ready");
      break;
    case 1001:                                                                      // standard startup routine for new device
      DAC_set_address(LDAC1, 0, 1);                                                 // Set DAC addresses to 1,2,3 assuming addresses are unset and all are factory (0,0,0)
      DAC_set_address(LDAC2, 0, 2);
      DAC_set_address(LDAC3, 0, 3);
      get_set_device_info(1);                                                           //  input device info and write to eeprom
      break;
    case 1002:                                                                          // continuously output until user enter -1+
      {
        int Xcomp, Ycomp, Zcomp;
        int Xval, Yval, Zval;
        int leave = 0;
        const int samples = 1000;
        while (leave != -1) {
          long sum = 0;
          leave = Serial_Input_Long("+", 1000);
          int par_raw = get_light_intensity(0, 1);
          float contactless_temp = (MLX90615_Read(0) + MLX90615_Read(0) + MLX90615_Read(0)) / 3.0;
          MMA8653FC_read(&Xval, &Yval, &Zval);
          MAG3110_read(&Xcomp, &Ycomp, &Zcomp);
          delay(200);
          for (int i = 0; i < samples; ++i) {
            sum += analogRead(HALL_OUT);
          }
          int hall = (sum / samples);
          delay(1);
          float temperature1 = bme1.readTempC();
          float temperature2 = bme2.readTempC();
          float relative_humidity1 = bme1.readHumidity();
          float relative_humidity2 = bme2.readHumidity();
          float pressure1 = bme1.readPressure() / 100;
          float pressure2 = bme2.readPressure() / 100;
          Serial_Printf("{\"par_raw\":%d,\"temperature\":%f,\"relative_humidity\":%f,\"pressure\":%f,\"temperature2\":%f,\"relative_humidity2\":%f,\"pressure2\":%f,\"contactless_temp\":%f,\"hall\":%d,\"accelerometer\":[%d,%d,%d],\"magnetometer\":[%d,%d,%d]}", par_raw, temperature1, relative_humidity1, pressure1, temperature2, relative_humidity2, pressure2, contactless_temp, hall, Xval, Yval, Zval, Xcomp, Ycomp, Zcomp);
          //          Serial_Printf("{\"par_raw\":%d,\"contactless_temp\":%f,\"hall\":%d,\"accelerometer\":[%d,%d,%d],\"magnetometer\":[%d,%d,%d]}", par_raw, contactless_temp, hall, Xval, Yval, Zval, Xcomp, Ycomp, Zcomp);
          Serial_Print_CRC();
        }
      }
      break;
    case 1003:
      {
        Serial_Print_Line("\"message\": \"Enter led # setting followed by +: \"}");
        int led =  Serial_Input_Double("+", 0);
        Serial_Print_Line("\"message\": \"Enter dac setting followed by +:  \"}");
        int setting =  Serial_Input_Double("+", 0);
        DAC_set(led, setting);
        DAC_change();
        digitalWriteFast(LED_to_pin[led], HIGH);
        delay(5000);
        digitalWriteFast(LED_to_pin[led], LOW);
        DAC_set(led, 0);
        DAC_change();
      }
      break;

    case 1004: {
        Serial_Print_Line("enter GMT hours+min+sec+day+month+year+");
        int hours, minutes, seconds, days, months, years;
        hours =  Serial_Input_Long("+");
        minutes =  Serial_Input_Long("+");
        seconds =  Serial_Input_Long("+");
        days =  Serial_Input_Long("+");
        months =  Serial_Input_Long("+");
        years =  Serial_Input_Long("+");
        setTime(hours, minutes, seconds, days, months, years);
        delay(2000);
      }
    // fall through to print
    case 1005:
      // example: 2004-02-12T15:19:21.000Z
      if (year() >= 2016) {
        Serial_Printf("{\"device_time\":\"%d-%d-%dT%d:%d:%d.000Z\"}\n", year(), month(), day(), hour(), minute(), second());
        Serial_Printf("{\"device_time\":%u}\n", now()); // since 1970 format
      }
      break;

    case 1006:
      print_calibrations();
      break;
    case 1007:
      get_set_device_info(0);
      break;

    case 1008:
      Serial_Print_Line("a");
      pinMode(POWERDOWN_REQUEST, OUTPUT);     //  bring P0.6 (2nd pin) low
      digitalWrite(POWERDOWN_REQUEST, LOW);
      Serial_Print_Line("b");
      delay(11000);                  // device should power off here - P0.5 (third pin) should go low
      Serial_Print_Line("c");        // shouldn't get here
      digitalWrite(POWERDOWN_REQUEST, HIGH); // put it back
      break;
    case 1011:
      Serial_Print_Line("PULSE1");
      DAC_set(1, 50);
      DAC_change();
      digitalWriteFast(PULSE1, HIGH);
      delay(1000);
      digitalWriteFast(PULSE1, LOW);
      DAC_set(1, 0);
      DAC_change();
      break;
    case 1012:
      Serial_Print_Line("PULSE2");
      DAC_set(2, 50);
      DAC_change();
      digitalWriteFast(PULSE2, HIGH);
      delay(1000);
      digitalWriteFast(PULSE2, LOW);
      DAC_set(2, 0);
      DAC_change();
      break;
    case 1013:
      Serial_Print_Line("PULSE3");
      DAC_set(3, 50);
      DAC_change();
      digitalWriteFast(PULSE3, HIGH);
      delay(1000);
      digitalWriteFast(PULSE3, LOW);
      DAC_set(3, 0);
      DAC_change();
      break;
    case 1014:
      Serial_Print_Line("PULSE4");
      DAC_set(4, 50);
      DAC_change();
      digitalWriteFast(PULSE4, HIGH);
      delay(1000);
      digitalWriteFast(PULSE4, LOW);
      DAC_set(4, 0);
      DAC_change();
      break;
    case 1015:
      Serial_Print_Line("PULSE5");
      DAC_set(5, 50);
      DAC_change();
      digitalWriteFast(PULSE5, HIGH);
      delay(1000);
      digitalWriteFast(PULSE5, LOW);
      DAC_set(5, 0);
      DAC_change();
      break;
    case 1016:
      Serial_Print_Line("PULSE6");
      DAC_set(6, 50);
      DAC_change();
      digitalWriteFast(PULSE6, HIGH);
      delay(1000);
      digitalWriteFast(PULSE6, LOW);
      DAC_set(6, 0);
      DAC_change();
      break;
    case 1017:
      Serial_Print_Line("PULSE7");
      DAC_set(7, 50);
      DAC_change();
      digitalWriteFast(PULSE7, HIGH);
      delay(1000);
      digitalWriteFast(PULSE7, LOW);
      DAC_set(7, 0);
      DAC_change();
      break;
    case 1018:
      Serial_Print_Line("PULSE8");
      DAC_set(8, 50);
      DAC_change();
      digitalWriteFast(PULSE8, HIGH);
      delay(1000);
      digitalWriteFast(PULSE8, LOW);
      DAC_set(8, 0);
      DAC_change();
      break;
    case 1019:
      Serial_Print_Line("PULSE9");
      DAC_set(9, 50);
      DAC_change();
      digitalWriteFast(PULSE9, HIGH);
      delay(1000);
      digitalWriteFast(PULSE9, LOW);
      DAC_set(9, 0);
      DAC_change();
      break;
    case 1020:
      Serial_Print_Line("PULSE10");
      DAC_set(10, 50);
      DAC_change();
      digitalWriteFast(PULSE10, HIGH);
      delay(1000);
      digitalWriteFast(PULSE10, LOW);
      DAC_set(10, 0);
      DAC_change();
      break;

    case 1021:                                                                            // variety of test commands used during development
      {
        /*
                  char S[10];
                  Serial_Print_Line(eeprom->userdef[0], 4);                                                                   // test the Serial_Input_Chars() command and saving values in userdef
                  Serial_Input_Chars(S, "+", 20000, sizeof(S));
                  Serial_Printf("output is %s \n", S);
                  eeprom->userdef[0] = atof(S);
        */
        Serial_Print_Line(eeprom->userdef[1], 4);
        store(userdef[1], Serial_Input_Double("+", 20000));                                                      // test Serial_Input_Double, save as userdef and recall

        Serial_Printf("output is %f \n", (float) eeprom->userdef[1]);
        // so measure the size of the string and if it's > 5000 then tell the user that the protocol is too long
        /*
                  Serial_Print("enter BLE baud rate (9600, 19200, 38400,57600) followed by +");                         //  Change the baud rate of the BLE
                  long baudrate = Serial_Input_Long();
                  Serial_Begin((int) baudrate);

                  Serial_Print("Enter 1/2/3/4+\n");
                  long setserial = Serial_Input_Long();
                  Serial_Printf("set serial to %d\n", (int)setserial);
                  Serial_Set((int) setserial);
                  Serial_Print_Line("test print");
        */
      }
      break;

    case 1027:                                                                                // restart teensy (keep here!)
      _reboot_Teensyduino_();
      break;
    case 1028:
      print_userdef();                                                                        // print only the userdef eeprom values
      break;
    case 1029:
      print_all();                                                                            // print everything in the eeprom (all values defined in eeprom.h)
      break;
    case 1030:
      //      Serial_Print("{\"message\": \"input 3 magnetometer bias values, each followed by +: \"}");
      store(mag_bias[0], Serial_Input_Double("+", 0));
      store(mag_bias[1], Serial_Input_Double("+", 0));
      store(mag_bias[2], Serial_Input_Double("+", 0));
      break;
    case 1031:
      //      Serial_Print("{\"message\": \"input 9 magnetometer calibration values, each followed by +: \"}");
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          store(mag_cal[j][i], Serial_Input_Double("+", 0));
        }
      }
      break;
    case 1032:
      //      Serial_Print("{\"message\": \"input 3 accelerometer bias values, each followed by +: \"}");
      store(accel_bias[0], Serial_Input_Double("+", 0));
      store(accel_bias[1], Serial_Input_Double("+", 0));
      store(accel_bias[2], Serial_Input_Double("+", 0));
      break;
    case 1033:
      //      Serial_Print("{\"message\": \"input 9 accelerometer calibration values, each followed by +: \"}");
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          store(accel_cal[j][i], Serial_Input_Double("+", 0));
        }
      }
      break;
    case 1034:
      //      Serial_Print_Line(eeprom->light_slope_all);
      //      Serial_Print_Line("{\"message\": \"input light slope for ambient par calibration followed by +: \"}");
      store(light_slope_all, Serial_Input_Double("+", 0));
      break;
    case 1035:
      //      Serial_Print_Line(eeprom->light_yint);
      //      Serial_Print_Line("{\"message\": \"input y intercept for ambient par calibration followed by +: \"}");
      store(light_yint, Serial_Input_Double("+", 0));
      break;
    case 1036:
      //      Serial_Print_Line(eeprom->light_slope_r);
      //      Serial_Print_Line("{\"message\": \"input r slope for ambient par calibration followed by +: \"}");
      store(light_slope_r, Serial_Input_Double("+", 0));
      break;
    case 1037:
      //      Serial_Print_Line(eeprom->light_slope_g);
      //      Serial_Print_Line("{\"message\": \"input g slope for ambient par calibration followed by +: \"}");
      store(light_slope_g, Serial_Input_Double("+", 0));
      break;
    case 1038:
      //      Serial_Print_Line(eeprom->light_slope_b);
      //      Serial_Print_Line("{\"message\": \"input b slope for ambient par calibration followed by +: \"}");
      store(light_slope_b, Serial_Input_Double("+", 0));
      break;
    case 1039:
      //      Serial_Print_Linef("%f, %f, %f",eeprom->thickness_a,eeprom->thickness_a,eeprom->thickness_a);
      //      Serial_Print_Line("{\"message\": \"input thickness calibration values (a, b, d) a for leaf thickness followed by +: \"}");
      store(thickness_a, Serial_Input_Double("+", 0));
      store(thickness_b, Serial_Input_Double("+", 0));
      store(thickness_d, Serial_Input_Double("+", 0));
      break;
    case 1040:
      //      Serial_Print_Linef("%f, %f",eeprom->detector_offset_slope[0],eeprom->detector_offset_yint[0]);
      //      Serial_Print_Line("{\"message\": \"input detector offset for detector 1 (main detector) followed by +: \"}");
      store(detector_offset_slope[0], Serial_Input_Double("+", 0));
      store(detector_offset_yint[0], Serial_Input_Double("+", 0));
      break;
    case 1041:
      //      Serial_Print_Linef("%f, %f",eeprom->detector_offset_slope[1],eeprom->detector_offset_yint[1]);
      //      Serial_Print_Line("{\"message\": \"input detector offset for detector 2 (reference on main) followed by +: \"}");
      store(detector_offset_slope[1], Serial_Input_Double("+", 0));
      store(detector_offset_yint[1], Serial_Input_Double("+", 0));
      break;
    case 1042:
      //      Serial_Print_Linef("%f, %f",eeprom->detector_offset_slope[2],eeprom->detector_offset_yint[2]);
      //      Serial_Print_Line("{\"message\": \"input detector offset for detector 3 (clamp detector) followed by +: \"}");
      store(detector_offset_slope[2], Serial_Input_Double("+", 0));
      store(detector_offset_yint[2], Serial_Input_Double("+", 0));
      break;
    case 1043:
      //      Serial_Print_Linef("%f, %f",eeprom->detector_offset_slope[3],eeprom->detector_offset_yint[3]);
      //      Serial_Print_Line("{\"message\": \"input detector offset for detector 4 (reference on clamp) followed by +: \"}");
      store(detector_offset_slope[3], Serial_Input_Double("+", 0));
      store(detector_offset_yint[3], Serial_Input_Double("+", 0));
      break;
    case 1044:
      //      Serial_Print_Line("{\"message\": \"input the LED #, slope, and y intercept for LED PAR calibration, each followed by +.  Set LED to -1 followed by + to exit loop: \"}");
      for (;;) {
        int led = Serial_Input_Double("+", 0);
        if (led == -1) {                                    // user can bail with -1+ setting as LED
          break;
        }
        else if (led > 0 || led < NUM_LEDS + 1) {
          store(par_to_dac_slope[led], Serial_Input_Double("+", 0));
          store(par_to_dac_yint[led], Serial_Input_Double("+", 0));
        }
        else {
          Serial_Printf("\"error\": \" User entered incorrect value.  Should be between 0 and %d", NUM_LEDS + 1);
        }
      }
      break;
    case 1045:
      //      Serial_Print_Line("{\"message\": \"input the IR baseline slope and yint: \"}");
      for (;;) {
        int led = Serial_Input_Double("+", 0);
        if (led == -1) {                                    // user can bail with -1+ setting as LED
          break;
        }
        else if (led > 0 || led < NUM_LEDS + 1) {

          store(ir_baseline_slope[led], Serial_Input_Double("+", 0));
          store(ir_baseline_yint[led], Serial_Input_Double("+", 0));
        }
        else {
          Serial_Printf("\"error\": \" User entered incorrect value.  Should be between 0 and %d", NUM_LEDS + 1);
        }
      }
      break;
    case 1046:
      //      Serial_Print_Line("{\"message\": \"input the LED #, slope, and y intercept for color calibration 1, each followed by +.  Set LED to -1 followed by + to exit loop: \"}");
      for (;;) {
        int led = Serial_Input_Double("+", 0);
        if (led == -1) {                                    // user can bail with -1+ setting as LED
          break;
        }
        else if (led > 0 || led < NUM_LEDS + 1) {

          store(colorcal_intensity1_slope[led], Serial_Input_Double("+", 0));
          store(colorcal_intensity1_yint[led], Serial_Input_Double("+", 0));
        }
        else {
          Serial_Printf("\"error\": \" User entered incorrect value.  Should be between 0 and %d", NUM_LEDS + 1);
        }
      }
      break;
    case 1047:
      //      Serial_Print_Line("{\"message\": \"input the LED #, slope, and y intercept for color calibration 2, each followed by +.  Set LED to -1 followed by + to exit loop: \"}");
      for (;;) {
        int led = Serial_Input_Double("+", 0);
        if (led == -1) {                                    // user can bail with -1+ setting as LED
          break;
        }
        else if (led > 0 || led < NUM_LEDS + 1) {
          store(colorcal_intensity2_slope[led], Serial_Input_Double("+", 0));
          store(colorcal_intensity2_yint[led], Serial_Input_Double("+", 0));
        }
        else {
          Serial_Printf("\"error\": \" User entered incorrect value.  Should be between 0 and %d", NUM_LEDS + 1);
        }
      }
      break;
    case 1048:
      //      Serial_Print_Line("{\"message\": \"input the LED #, slope, and y intercept for color calibration 3, each followed by +.  Set LED to -1 followed by + to exit loop: \"}");
      for (;;) {
        int led = Serial_Input_Double("+", 0);
        if (led == -1) {                                    // user can bail with -1+ setting as LED
          break;
        }
        else if (led > 0 || led < NUM_LEDS + 1) {
          store(colorcal_intensity3_slope[led], Serial_Input_Double("+", 0));
          store(colorcal_intensity3_yint[led], Serial_Input_Double("+", 0));
        }
        else {
          Serial_Printf("\"error\": \" User entered incorrect value.  Should be between 0 and %d", NUM_LEDS + 1);
        }
      }
      break;
    case 1049:
      //      Serial_Print_Line("{\"message\": \"input the LED #, slope, and y intercept for blank at thickness 1 (true blank), thickness 2 (1 piece of white paper), and thickness 3 (3 pieces of white paper), each followed by +.  Set LED to -1 followed by + to exit loop: \"}");
      for (;;) {
        int led = Serial_Input_Double("+", 0);
        if (led == -1) {                                    // user can bail with -1+ setting as LED
          break;
        }
        else if (led > 0 || led < NUM_LEDS + 1) {
          store(colorcal_blank1[led], Serial_Input_Double("+", 0));
          store(colorcal_blank2[led], Serial_Input_Double("+", 0));
          store(colorcal_blank3[led], Serial_Input_Double("+", 0));
        }
        else {
          Serial_Printf("\"error\": \" User entered incorrect value.  Should be between 0 and %d", NUM_LEDS + 1);
        }
      }
      break;
    case 1050:
      //      Serial_Print_Line("{\"message\": \"input the LED #, slope, and y intercept for color calibration 3, each followed by +.  Set LED to -1 followed by + to exit loop: \"}");
      for (;;) {
        int userdefID = Serial_Input_Double("+", 0);
        if (userdefID == -1) {                                    // user can bail with -1+ setting as LED
          break;
        }
        else if (userdefID > 0 || userdefID < 50) {
          store(userdef[userdefID], Serial_Input_Double("+", 0));
        }
        else {
          Serial_Printf("\"error\": \" User entered incorrect value.  Should be between 0 and %d", (sizeof(eeprom->userdef) / sizeof(float)));
        }
      }
      break;

    case 1078:                                                                   // over the air update of firmware.   DO NOT MOVE THIS!
      upgrade_firmware();
      break;

    case 1200:
      battery_low();  // test battery
      break;

    case 1100:
      break;
    case 1101:
      {
        Serial_Print("let's start with a test, this is more than 20 chars long.\n");
        Serial_Flush_Output();
        char c[2];
        int count = 0;
        c[1] = 0;
        for (;;)  {
          c[0] = Serial_Read();
          Serial_Print(c);
          if (c[0] < ' ') continue;
          ++count;
          if (c[0] == 'X')
            break;
        } // for
        Serial_Printf("%d chars\n", count);
        Serial_Flush_Output();
      }
      break;
    case 2000:
      Serial_Printf("Compiled on: %s %s\n", __DATE__, __TIME__);
      break;

    case 4044:
      {
        // JZ test - do not remove
        // read and analyze noise on ADC from a single LED pulse
        const int LED = 5;                              // 1 = green, 2 = red, 5 = IR
        const int SAMPLES = 100;
        uint16_t val[SAMPLES];
        Serial_Print_Line("JZ test");
        DAC_set(LED, 300);                             // set LED intensity
        DAC_change();
        AD7689_set(0);                                  // select ADC channel
        digitalWriteFast(HOLDM, HIGH);                  // discharge cap
        delay(1000);
        noInterrupts();
        digitalWriteFast(LED_to_pin[LED], HIGH);        // turn on LED
        delayMicroseconds(30);                          // allow slow actopulser to stabilize (longer is better)
        digitalWriteFast(HOLDM, LOW);                   // start integrating (could take baseline value here)
        delayMicroseconds(10);                          // measuring width
        digitalWriteFast(LED_to_pin[LED], LOW);         // turn off LED
        delayMicroseconds(60);                          // experimental - some early samples are rising
        uint32_t delta_time = micros();
        AD7689_read_array(val, SAMPLES);                // read values
        delta_time = micros() - delta_time;
        interrupts();
        for (int i = 0; i < SAMPLES; ++i) {
          val[i] += i * 1.4;                                // adjust for droop (about 1 count per sample)
          Serial_Printf(" % d\n", (int)val[i]);
        }
        Serial_Printf("single pulse stdev = % .2f AD counts\n", stdev16(val, SAMPLES));
        Serial_Printf("time = % d usec for % d samples\n", delta_time, SAMPLES);
      }
      break;

    case 4047:
      {
        // JZ test - do not remove
        // read multiple pulses with increasing intensity or pulse width for linearity test
        // with constant DAC value and pulse width, it is good for a pulse-to-pulse stdev test
        const int LED = 5;                              // 1 = green, 2 = red, 3 = yellow, 5 = IR (keep DAC < 100)
        Serial_Print_Line("using delay - wait...");
        AD7689_set(0);                                  // 0 is main detector
        DAC_set(LED, 700);                               // set initial LED intensity
        DAC_change();
        const int MAX = 100;                            // try a variety of intensities 0 up to 4095
        int count = 0;
        uint16_t data[100];

        for (int i = 0; i < MAX; i += MAX / 100) {
          //DAC_set(LED, i);                              // change LED intensity
          //DAC_change();
          digitalWriteFast(HOLDM, HIGH);                  // discharge cap
          delay(33);                                     // also allows LED to cool and DC filter to adjust
          noInterrupts();
          digitalWriteFast(LED_to_pin[LED], HIGH);        // turn on LED
          delayMicroseconds(10);                          // allow slow actopulser to stabilize
          digitalWriteFast(HOLDM, LOW);                   // start integrating
          delayMicroseconds(i);                          // pulse width (depends on sensitivity needed)
          digitalWriteFast(LED_to_pin[LED], LOW);         // turn off LED
          const int SAMPLES = 21;                         // reduce noise with multiple reads
          uint16_t val[SAMPLES];
          AD7689_read_array(val, SAMPLES);                // read values
          interrupts();
          data[count] = median16(val, SAMPLES);
          if (data[count] >= 65535) break;                 // saturated the ADC, no point in continuing
          Serial_Printf(" % d, % d\n", i, data[count]);
          ++count;
        } // for
        // results from each pulse are in data[]
        Serial_Printf("pulse to pulse stdev = % .2f AD counts, first = % d\n\n", stdev16(data, count), data[0]);
      }
      break;

    case 4048:
      {
        // JZ test - do not remove
        // read multiple pulses with increasing intensity or pulse width for linearity test
        // with constant DAC value and pulse width, it is good for a pulse-to-pulse stdev test
        const int LED = 5;                              // 1 = green, 2 = red, 3 = yellow, 5 = IR (keep DAC < 100)
        Serial_Print_Line("using 2 timers - wait...");
        AD7689_set(0);                                  // 0 is main detector
        DAC_set(LED, 200);                               // set initial LED intensity
        DAC_change();
        const int MAX = 200;                            // try a variety of intensities 0 up to 4095
        int count = 0;
        uint16_t data[100];

        _meas_light = LED;
        _pulsesize = 30 + 10;                  // account for actopulser delay
        unsigned _pulsedistance = 2000;                 // lower has less jitter

        startTimers(_pulsedistance);        // schedule continuous LED pulses

        for (int i = 1; i < MAX; i += MAX / 100) {
          //DAC_set(LED, i);                              // change LED intensity
          //DAC_change();
          digitalWriteFast(HOLDM, HIGH);                  // discharge cap

          led_off = 0;

          while (led_off == 0) {}                  // wait till pulse is done

          // a pulse completed
          noInterrupts();

          const int SAMPLES = 19;                         // reduce noise with multiple reads
          uint16_t val[SAMPLES];
          AD7689_read_array(val, SAMPLES);                // read values
          interrupts();

          data[count] = median16(val, SAMPLES);
          if (data[count] >= 65535) break;                 // saturated the ADC, no point in continuing
          //Serial_Printf(" % d, % d\n", i, data[count]);
          ++count;
        } // for

        stopTimers();

        // results from each pulse are in data[]
        Serial_Printf("pulse to pulse stdev = % .2f AD counts, first = % d\n\n", stdev16(data, count), data[0]);
      }
      break;

    default:
      Serial_Printf("{\"error\":\"bad command # %s\"}\n", choose);
      break;

  }  // switch()

} // do_command()

// read in and execute a protocol
// example: [{"pulses": [150],"a_lights": [[3]],"a_intensities": [[50]],"pulsedistance": 1000,"m_intensities": [[125]],"pulsesize": 2,"detectors": [[3]],"meas_lights": [[1]],"protocols": 1}]<newline>

void do_protocol()
{
  static const int serial_buffer_size = 5000;                                        // max size of the incoming jsons
  static const int max_jsons = 15;                                                   // max number of protocols per measurement
  static const int MAX_JSON_ELEMENTS = 600;      //

  static int averages = 1;                        // ??
  static uint8_t spec_on = 0;                    // flag to indicate that spec is being used during this measurement
  static float data = 0;
  static float data_ref = 0;
  static int act_background_light = 0;
  //static float freqtimer0;
  //static float freqtimer1;
  //static float freqtimer2;
  int measurements = 1;                                     // the number of times to repeat the entire measurement (all protocols)
  unsigned long measurements_delay = 0;                     // number of seconds to wait between measurements
  unsigned long measurements_delay_ms = 0;                  // number of milliseconds to wait between measurements
  unsigned long meas_number = 0;                            // counter to cycle through measurement lights 1 - 4 during the run
  //unsigned long end1;
  //unsigned long start1 = millis();

  // these variables could be pulled from the JSON at the time of use... however, because pulling from JSON is slow, it's better to create a int to save them into at the beginning of a protocol run and use the int instead of the raw hashTable.getLong type call
  int _a_lights [NUM_LEDS] = {};
  int _a_intensities [NUM_LEDS] = {};
  int _a_lights_prev [NUM_LEDS] = {};
  int act_background_light_prev = 0;
  int cycle = 0;                                                                // current cycle number (start counting at 0!)
  int pulse = 0;                                                                // current pulse number
  //int total_cycles;                                                           // Total number of cycles - note first cycle is cycle 0
  int meas_array_size = 0;                                                      // measures the number of measurement lights in the current cycle (example: for meas_lights = [[15,15,16],[15],[16,16,20]], the meas_array_size's are [3,1,3].
  //int end_flag = 0;

  int number_of_protocols = 0;

  String json2 [max_jsons];     // TODO - don't use String   // will contain each json
  for (int i = 0; i < max_jsons; i++) {
    json2[i] = "";                                                              // reset all json2 char's to zero (ie reset all protocols)
  }


  { // create limited scope for serial_buffer
    char serial_buffer[serial_buffer_size + 1];     // large buffer for reading in a json protocol from serial port

    Serial_Input_Chars(serial_buffer, "\r\n", 500, serial_buffer_size);       // input the protocol

    if (!check_protocol(serial_buffer)) {         // sanity check
      Serial_Print("{\"error\":\"bad json protocol (braces or CRC) got:\"}");
      Serial_Print(serial_buffer);
      return;
    }

    // break up the protocol into individual jsons

    // TODO improve this - use in place, stretch it in place or copy to new C strings?
    // make json2 an array of char pointers to each protocol
    for (unsigned i = 1; i < strlen(serial_buffer); i++) {         // increments through each char in incoming transmission - if it's open curly, it saves all chars until closed curly.  Any other char is ignored.
      if (serial_buffer[i] == '{') {                               // wait until you see a open curly bracket
        while (serial_buffer[i] != '}') {                          // once you see it, save incoming data to json2 until you see closed curly bracket
          json2[number_of_protocols] += serial_buffer[i];          // add single char to json
          i++;
        }
        json2[number_of_protocols] += serial_buffer[i];           // catch the last closed curly
        number_of_protocols++;
      }
    }  // for

  } // no more need for the serial input buffer

  if (DEBUGSIMPLE) {
    Serial_Printf("got %d protocols\n", number_of_protocols);

    // print each json
    for (int i = 0; i < number_of_protocols; i++) {
      Serial_Printf("Incoming JSON %d as received by Teensy : %s\n", i, json2[i].c_str());
    } // for
  } // DEBUGSIMPLE

  Serial_Printf("{\"device_id\":%ld", eeprom->device_id);   // change to : format?
  Serial_Printf(",\"device_version\":%s", DEVICE_VERSION);
  Serial_Printf(",\"firmware_version\":%s", DEVICE_FIRMWARE);
  if (year() >= 2016)
    Serial_Printf(",\"device_time\":%u", now());
  Serial_Print(",\"sample\":[");

  // discharge sample and hold in case the cap is currently charged (on add on and main board)
  digitalWriteFast(HOLDM, HIGH);
  digitalWriteFast(HOLDADD, HIGH);
  delay(10);

  // loop through the all measurements to create a measurement group
  for (int y = 0; y < measurements; y++) {                   // measurements is initially 1, but gets updated after the json is parsed

    Serial_Print("[");                                                                        // print brackets to define single measurement

    for (int q = 0; q < number_of_protocols; q++) {                                           // loop through all of the protocols to create a measurement

      JsonHashTable hashTable;
      JsonParser<MAX_JSON_ELEMENTS> root;
      char json[json2[q].length() + 1];                                     // we need a writeable C string
      strncpy(json, json2[q].c_str(), json2[q].length());
      json[json2[q].length()] = '\0';                                       // Add closing character to char*
      json2[q] = "";                                                        // attempt to release the String memory
      hashTable = root.parseHashTable(json);                                // parse it
      if (!hashTable.success()) {                                           // NOTE: if the incomign JSON is too long (>~5000 bytes) this tends to be where you see failure (no response from device)
        Serial_Print("{\"error\":\"JSON failure with:\"}");
        Serial_Print(json);
        goto abort;
      }

#if 0
      // TODO - is this the right place?
      // wait until the jaw is closed (detected with the hall sensor)
      if (measurements == 0 && q == 0 && hashTable.getLong("start_on_close") == 1) {
        wait_for_closed();
      }
#endif

      int protocols = 1;                                                                       // starts as 1 but gets updated when the json is parsed
      int quit = 0;

      for (int u = 0; u < protocols; u++) {                                                    // the number of times to repeat the current protocol
        JsonArray save_eeprom    = hashTable.getArray("save");
        JsonArray recall_eeprom  = hashTable.getArray("recall");                                // userdef values to recall
        JsonArray number_samples = hashTable.getArray("number_samples");                       // number of samples on the cap during sample + hold phase (default is 40);
        JsonArray reference =     hashTable.getArray("reference");                              // subtract reference value if set to 1 (also causes 1/2 the sample rate per detector) due to time constraints.  Default is no reference (== 0)
        uint16_t dac_lights =     hashTable.getLong("dac_lights");                                // when inputting light intensity values, use DAC instead of calibrated outputs (do not use expr() function on coming API data)
        uint16_t adc_show =       hashTable.getLong("adc_show");                                // this tells the MultispeQ to print the ADC values only instead of the normal data_raw (for signal quality debugging)
        uint16_t adc_only[150];                                                   // this and first_adc_ref are used to save the first set of ADC averages produced, so it can be optionally displayed instead of data_raw (for signal quality debugging).  USE ONLY WITH REFERENCE == 0 (ie reference is OFF!)
        JsonArray pulses =        hashTable.getArray("pulses");                                // the number of measuring pulses, as an array.  For example [50,10,50] means 50 pulses, followed by 10 pulses, follwed by 50 pulses.
        String protocol_id =      hashTable.getString("protocol_id");                          // used to determine what macro to apply
        int analog_averageds =     hashTable.getLong("analog_averageds");                          // DEPRECIATED IN NEWEST HARDWARE 10/14 # of measurements per measurement pulse to be internally averaged (min 1 measurement per 6us pulselengthon) - LEAVE THIS AT 1 for now
        if (analog_averageds == 0) {                                                              // if averages don't exist, set it to 1 automatically.
          analog_averageds = 1;
        }
        averages =                hashTable.getLong("averages");                               // The number of times to average this protocol.  The spectroscopic and environmental data is averaged in the device and appears as a single measurement.
        if (averages == 0) {                                                                   // if averages don't exist, set it to 1 automatically.
          averages = 1;
        }
        int averages_delay =      hashTable.getLong("averages_delay");
        int averages_delay_ms =   hashTable.getLong("averages_delay_ms");                       // same as above but in ms
        measurements =            hashTable.getLong("measurements");                            // number of times to repeat a measurement, which is a set of protocols
        measurements_delay =      hashTable.getLong("measurements_delay");                      // delay between measurements in seconds
        measurements_delay_ms =      hashTable.getLong("measurements_delay_ms");                      // delay between measurements in milliseconds
        protocols =               hashTable.getLong("protocols");                               // delay between protocols within a measurement
        if (protocols == 0) {                                                                   // if averages don't exist, set it to 1 automatically.
          protocols = 1;
        }
        int protocols_delay =     hashTable.getLong("protocols_delay");                         // delay between protocols within a measurement
        int protocols_delay_ms =     hashTable.getLong("protocols_delay_ms");                         // delay between protocols within a measurement in milliseconds
        if (hashTable.getLong("act_background_light") == 0) {                                    // The Teensy pin # to associate with the background actinic light.  This light continues to be turned on EVEN BETWEEN PROTOCOLS AND MEASUREMENTS.  It is always Teensy pin 13 by default.
          act_background_light =  0;                                                            // change to new background actinic light
        }
        else {
          act_background_light =  hashTable.getLong("act_background_light");
        }

        //averaging0 - 1 - 30
        //averaging1 - 1 - 30
        //resolution0 - 2 - 16
        //resolution1 - 2 - 16
        //conversion_speed - 0 - 5
        //sampling_speed - 0 - 5

        int averaging0 =          hashTable.getLong("averaging");                               // # of ADC internal averages
        if (averaging0 == 0) {                                                                   // if averaging0 don't exist, set it to 10 automatically.
          averaging0 = 10;
        }
        //int averaging1 = averaging0;
        int resolution0 =         hashTable.getLong("resolution");                               // adc resolution (# of bits)
        if (resolution0 == 0) {                                                                   // if resolution0 don't exist, set it to 16 automatically.
          resolution0 = 16;
        }
        //int resolution1 = resolution0;
        int conversion_speed =    hashTable.getLong("conversion_speed");                               // ADC speed to convert analog to digital signal (5 fast, 0 slow)
        if (conversion_speed == 0) {                                                                   // if conversion_speed don't exist, set it to 3 automatically.
          conversion_speed = 3;
        }
        int sampling_speed =      hashTable.getLong("sampling_speed");                               // ADC speed of sampling (5 fast, 0 slow)
        if (sampling_speed == 0) {                                                                   // if sampling_speed don't exist, set it to 3 automatically.
          sampling_speed = 3;
        }
        //        int tcs_to_act =            hashTable.getLong("tcs_to_act");                               // sets the % of response from the tcs light sensor to act as actinic during the run (values 1 - 100).  If tcs_to_act is not defined (ie == 0), then the act_background_light intensity is set to actintensity1.
        //int offset_off =          hashTable.getLong("offset_off");                               // turn off detector offsets (default == 0 which is on, set == 1 to turn offsets off)

        ///*
        JsonArray pulsedistance =   hashTable.getArray("pulsedistance");                            // distance between measuring pulses in us.  Minimum 1000 us.
        JsonArray pulsesize =       hashTable.getArray("pulsesize");                            // distance between measuring pulses in us.  Minimum 1000 us.

        JsonArray a_lights =        hashTable.getArray("a_lights");
        JsonArray a_intensities =   hashTable.getArray("a_intensities");
        JsonArray m_intensities =   hashTable.getArray("m_intensities");

        //        int get_offset =          hashTable.getLong("get_offset");                               // include detector offset information in the output
        // NOTE: it takes about 50us to set a DAC channel via I2C at 2.4Mz.

        JsonArray detectors =     hashTable.getArray("detectors");                               // the Teensy pin # of the detectors used during those pulses, as an array of array.  For example, if pulses = [5,2] and detectors = [[34,35],[34,35]] .
        JsonArray meas_lights =   hashTable.getArray("meas_lights");
        JsonArray message =       hashTable.getArray("message");                                // sends the user a message to which they must reply <answer>+ to continue
        //*/
        JsonArray environmental = hashTable.getArray("environmental");

        // ********************INPUT DATA FOR CORALSPEQ*******************
        JsonArray spec =          hashTable.getArray("spec");                                // defines whether the spec will be called during each array.  note for each single plus, the spec will call and add 256 values to data_raw!
        JsonArray delay_time =    hashTable.getArray("delay_time");                          // delay per half clock (in microseconds).  This ultimately conrols the integration time.
        JsonArray read_time =     hashTable.getArray("read_time");                           // Amount of time that the analogRead() procedure takes (in microseconds)
        JsonArray intTime =       hashTable.getArray("intTime");                             // delay per half clock (in microseconds).  This ultimately conrols the integration time.
        JsonArray accumulateMode = hashTable.getArray("accumulateMode");

        long size_of_data_raw = 0;
        long total_pulses = 0;

        for (int i = 0; i < pulses.getLength(); i++) {                                      // count the number of non zero lights and total pulses
          total_pulses += pulses.getLong(i) * meas_lights.getArray(i).getLength();          // count the total number of pulses
          int non_zero_lights = 0;
          for (int j = 0; j < meas_lights.getArray(i).getLength(); j++) {                   // count the total number of non zero pulses
            if (meas_lights.getArray(i).getLong(j) > 0) {
              non_zero_lights++;
            }
          }

          // redefine the size of data raw to account for the 256 spec measurements per 1 pulse if spec is used (for coralspeq)
          if (spec.getLong(i) == 1) {
            size_of_data_raw += pulses.getLong(i) * non_zero_lights * 256;
          }
          else {
            size_of_data_raw += pulses.getLong(i) * non_zero_lights;
          }

        } // for each pulse

        unsigned long data_raw_average[size_of_data_raw];                                          // buffer for ADC output data
        for (int i = 0; i < size_of_data_raw; ++i)                                                 // zero it
          data_raw_average[i] = 0;

        if (DEBUGSIMPLE) {
          Serial_Print_Line("");
          Serial_Print("size of data raw:  ");
          Serial_Print_Line(size_of_data_raw);

          Serial_Print_Line("");
          Serial_Print("total number of pulses:  ");
          Serial_Print_Line(total_pulses);

          Serial_Print_Line("");
          Serial_Print("all data in data_raw_average:  ");
          for (int i = 0; i < size_of_data_raw; i++) {
            Serial_Print((unsigned)data_raw_average[i]);
          }
          Serial_Print_Line("");

          Serial_Print_Line("");
          Serial_Print("number of pulses:  ");
          Serial_Print_Line(pulses.getLength());

          Serial_Print_Line("");
          Serial_Print("arrays in meas_lights:  ");
          Serial_Print_Line(meas_lights.getLength());

          Serial_Print_Line("");
          Serial_Print("length of meas_lights arrays:  ");
          for (int i = 0; i < meas_lights.getLength(); i++) {
            Serial_Print(meas_lights.getArray(i).getLength());
            Serial_Print(", ");
          }
          Serial_Print_Line("");
        } // DEBUGSIMPLE

        Serial_Print("{");

        Serial_Print("\"protocol_id\":\"");
        Serial_Print(protocol_id.c_str());
        Serial_Print("\",");

        /*
                if (get_offset == 1) {
                  print_offset(1);
                }
        */

        if (averages > 1) {
          Serial_Print("\"averages\":");
          Serial_Print(averages);
          Serial_Print(",");
        }

        //        print_sensor_calibration(1);                                               // print sensor calibration data

        // clear variables (many not needed)

        light_intensity = light_intensity_averaged = 0;
        light_intensity_raw = light_intensity_raw_averaged = 0;
        r = r_averaged =  g = g_averaged = b = b_averaged = 0;

        thickness = thickness_averaged = 0;
        thickness_raw = thickness_raw_averaged = 0;

        contactless_temp = contactless_temp_averaged = 0;

        cardinal = cardinal_averaged = 0;
        x_cardinal_raw = 0, y_cardinal_raw = 0, z_cardinal_raw = 0;
        x_cardinal_raw_averaged = 0, y_cardinal_raw_averaged = 0, z_cardinal_raw_averaged = 0;

        x_tilt = y_tilt = z_tilt = 0;
        x_tilt_averaged = 0, y_tilt_averaged = 0, z_tilt_averaged = 0;
        x_tilt_raw = 0, y_tilt_raw = 0, z_tilt_raw = 0;
        x_tilt_raw_averaged = 0, y_tilt_raw_averaged = 0, z_tilt_raw_averaged = 0;

        temperature = humidity = pressure = 0;
        temperature_averaged = humidity_averaged = pressure_averaged = 0;

        temperature2 = humidity2 = pressure2 = 0;
        temperature2_averaged = humidity2_averaged = pressure2_averaged = 0;

        //!!! when offset gets recalculated I need to reposition this later, since pulsesize is now an array
        //        calculate_offset(pulsesize);                                                                    // calculate the offset, based on the pulsesize and the calibration values (ax+b)

        // perform the protocol averages times
        for (int x = 0; x < averages; x++) {                                                 // Repeat the protocol this many times

          if (Serial_Available() && Serial_Input_Long("+", 1) == -1) {        // test for abort command
            q = number_of_protocols - 1;
            y = measurements - 1;
            u = protocols;
            x = averages;
          }

          int background_on = 0;
          long data_count = 0;
          int message_flag = 0;                                                              // flags to indicate if an alert, prompt, or confirm have been called at least once (to print the object name to data JSON)
          unsigned _pulsedistance = 0;                                                  // initialize variables for pulsesize and pulsedistance (as well as the previous cycle's pulsesize and pulsedistance).  We define these only once per cycle so we're not constantly calling the JSON (which is slow)
          unsigned _pulsedistance_prev = 0;
          uint16_t  _pulsesize_prev = 0;
          uint16_t _reference_flag = 0;                                                           // used to note if this is the first measurement
          float _reference_start = 0;                                                            // reference value at data point 0 - initial value for normalizing the reference (normalized based on the values from main and reference in the first point in the trace)
          float _main_start = 0;                                                               // main detector (sample) value at data point 0 - initial value for normalizing the reference (normalized based on the values from main and reference in the first point in the trace)
          uint16_t _number_samples = 0;                                                               // create the adc sampling rate number
          //          lux_local = 0;                                                                     // reset local (this measurement) light levels
          //          r_local = 0;
          //          g_local = 0;
          //          b_local = 0;

          //      options for relative humidity, temperature, contactless temperature. light_intensity,co2
          //           0 - take before spectroscopy measurements
          //           1 - take after spectroscopy measurements
          environmentals(environmental, averages, x, 0);

          //&&
          //          analogReadAveraging(analog_averageds);                                      // set analog averaging (ie ADC takes one signal per ~3u)

          //////////////////////ADC SETUP////////////////////////

          //int actfull = 0;
          //          int _tcs_to_act = 0;

          //          float _light_intensity = 0;

          //          float _light_intensity = lux_to_uE(lux_local);
          //          _tcs_to_act = (uE_to_intensity(act_background_light,_light_intensity)*tcs_to_act)/100;  // set intensity of actinic background light

          if (DEBUGSIMPLE) {
            Serial_Print_Line("");
            Serial_Print("tcs to act: ");
            //Serial_Print_Line(_tcs_to_act);
            Serial_Print("ambient light in uE: ");
            //Serial_Print_Line(lux_to_uE(lux_local));
            Serial_Print("ue to intensity result:  ");
            //Serial_Print_Line(uE_to_intensity(act_background_light, lux_to_uE(lux_local))); // NOTE: wait turn flip DAC switch until later.
          } // DEBUGSIMPLE

          for (int z = 0; z < total_pulses; z++) {                                      // cycle through all of the pulses from all cycles
            int first_flag = 0;                                                           // flag to note the first pulse of a cycle
            int _spec = 0;                                                              // create the spec flag for the coralspeq

            int _intTime = 0;                                                           // create the _intTime flag for the coralspeq
            int _delay_time = 0;                                                        // create the _delay_time flag for the coralspeq
            int _read_time = 0;                                                         // create the _read_time flag for the coralspeq
            int _accumulateMode = 0;                                                    // create the _accumulateMode flag for the coralspeq

            if (pulse == 0) {                                                                                     // if it's the first pulse of a cycle, we need to set up the new set of lights and intensities...
              meas_array_size = meas_lights.getArray(cycle).getLength();                                          // get the number of measurement/detector subsets in the new cycle

              if (PULSERDEBUG) {
                Serial_Printf("\n _number_samples: %d \n", _number_samples);
              } // PULSERDEBUG

              for (unsigned i = 0; i < NUM_LEDS; i++) {                                  // save the list of act lights in the previous pulse set to turn off later
                _a_lights_prev[i] = _a_lights[i];
                if (PULSERDEBUG) {
                  Serial_Printf("\n all a_lights_prev: %d\n", _a_lights_prev[i]);
                } // PULSERDEBUG
              }
              for (unsigned i = 0; i < NUM_LEDS; i++) {                                   // save the current list of act lights, determine if they should be on, and determine their intensity
                _a_lights[i] = a_lights.getArray(cycle).getLong(i);                        // save which light should be turned on/off
                String intensity_string = a_intensities.getArray(cycle).getString(i);
                _a_intensities[i] = expr(intensity_string.c_str());                       // evaluate as an expression

                if (PULSERDEBUG) {
                  Serial_Printf("\n all a_lights, intensities: %d,%d\n", _a_lights[i], _a_intensities[i]);
                } // PULSERDEBUG
              }
              //              }

              if (CORAL_SPEQ) {
                _spec = spec.getLong(cycle);                                                      // pull whether the spec will get called in this cycle or not for coralspeq and set parameters.  If they are empty (not defined by the user) set them to the default value
                if (_spec == 1) {
                  _intTime = intTime.getLong(cycle);
                  if (_intTime == 0) {
                    _intTime = 100;
                  }
                  _delay_time = delay_time.getLong(cycle);
                  if (_delay_time == 0) {
                    _delay_time = 35;
                  }
                  _read_time = read_time.getLong(cycle);
                  if (_read_time == 0) {
                    _read_time = 35;
                  }
                  _accumulateMode = accumulateMode.getLong(cycle);
                  if (_accumulateMode == 0) {
                    _accumulateMode = false;
                  }
                }
              } // CORAL_SPEQ

              if (cycle != 0) {
                _pulsedistance_prev = _pulsedistance;
                _pulsesize_prev = _pulsesize;
              }
              _pulsedistance = pulsedistance.getLong(cycle);                                                    // initialize variables for pulsesize and pulsedistance (as well as the previous cycle's pulsesize and pulsedistance).  We define these only once per cycle so we're not constantly calling the JSON (which is slow)
              _pulsesize = pulsesize.getLong(cycle);
              first_flag = 1;                                                                                   // flip flag indicating that it's the 0th pulse and a new cycle
              if (cycle == 0) {                                                                                 // if it's the beginning of a measurement (cycle == 0 and pulse == 0), then...
                digitalWriteFast(act_background_light_prev, LOW);                                               // turn off actinic background light and...
                startTimers(_pulsedistance);                                                                    // Use the two interrupt service routines timers (pulse1 and pulse2) in order to turn on (pulse1) and off (pulse2) the measuring lights.
              }
              else if (cycle != 0 && (_pulsedistance != _pulsedistance_prev || _pulsesize != _pulsesize_prev)) {    // if it's not the 0th cycle and the last pulsesize or pulsedistance was different than the current one, then stop the old timers and set new ones.   If they were the same, avoid resetting the timers by skipping this part.
                //              stopTimers();                                                                                   // stop the old timers
                startTimers(_pulsedistance);                                    // restart the measurement light timer
              }
            }

            if (PULSERDEBUG) {
              Serial_Printf("pulsedistance = %d, pulsesize = %d, cycle = %d, measurement number = %d, measurement array size = %d,total pulses = %d\n", (int) _pulsedistance, (int) _pulsesize, (int) cycle, (int) meas_number, (int) meas_array_size, (int) total_pulses);
            } // PULSERDEBUG

            _number_samples = number_samples.getLong(cycle);                                               // set the _number_samples for this cycle
            //            assert(_number_samples >= 0 && _number_samples < 500);

            _meas_light = meas_lights.getArray(cycle).getLong(meas_number % meas_array_size);             // move to next measurement light
            String intensity_string = m_intensities.getArray(cycle).getString(meas_number % meas_array_size);          // evaluate inputted intensity to see if it was an expression
            uint16_t _m_intensity = expr(intensity_string.c_str());
            //            assert(_m_intensity >= 0 && _m_intensity <= 4095);

            uint16_t detector = detectors.getArray(cycle).getLong(meas_number % meas_array_size);          // move to next detector
            //            assert(detector >= 1 && detector <= 10);

            uint16_t _reference = reference.getArray(cycle).getLong(meas_number % meas_array_size);

            if (_number_samples == 0) {                                                                    // if _number_samples wasn't set or == 0, set it automatically to 19 (default)
              _number_samples = 19;
            }
            if (_reference != 0) {                                                                      // if using the reference detector, make sure to half the sample rate (1/2 sample from main, 1/2 sample from detector)
              _number_samples = _number_samples / 2;
            }
            if (_reference == 0) {                                                                      // If the reference detector isn't turned on, then we need to set the ADC first
              AD7689_set (detector - 1);        // set ADC channel as specified
            }

            if (PULSERDEBUG) {
              Serial_Printf("measurement light, intensity, detector, reference:  %d, %d, %d, %d\n", _meas_light, _m_intensity, detector, _reference);
            } // PULSERDEBUG

            if (pulse < meas_array_size) {   // if it's the first pulse of a cycle, then change act 1,2,3,4 values as per array's set at beginning of the file

              if (pulse == 0) {
                String _message_type = message.getArray(cycle).getString(0);                                // get what type of message it is
                if ((_message_type != "" || quit == -1) && x == 0) {                                         // if there are some messages or the user has entered -1 to quit AND it's the first repeat of an average (so it doesn't ask these question on every average), then print object name...
                  if (message_flag == 0) {                                                                 // if this is the first time the message has been printed, then print object name
                    Serial_Print("\"message\":[");
                    message_flag = 1;
                  }
                  Serial_Print("[\"");
                  Serial_Print(_message_type.c_str());                                                               // print message
                  Serial_Print("\",");
                  Serial_Print("\"");
                  Serial_Print(message.getArray(cycle).getString(1));
                  Serial_Print("\",");
                  if (_message_type == "0") {
                    Serial_Print("\"\"]");
                  }
                  else if (_message_type == "alert") {                                                    // wait for user response to alert
                    stopTimers();                                                                         // pause the timers (so the measuring light doesn't stay on
                    while (1) {
                      long response = Serial_Input_Long("+", 0);
                      if (response == -1) {
                        Serial_Print("\"ok\"]");
                        break;
                      }
                    }
                    startTimers(_pulsedistance);                                                // restart the measurement light timer
                  }
                  else if (_message_type == "confirm") {                                                  // wait for user's confirmation message.  If enters '1' then skip to end.
                    stopTimers();                                                                         // pause the timers (so the measuring light doesn't stay on
                    while (1) {
                      long response = Serial_Input_Long("+", 0);
                      if (response == 1) {
                        Serial_Print("\"cancel\"]]");                                                     // set all loops (protocols, measurements, averages, etc.) to the last loop value so it exits gracefully
                        q = number_of_protocols;
                        y = measurements - 1;
                        u = protocols - 1;
                        x = averages;
                        z = total_pulses;
                        break;
                      }
                      if (response == -1) {
                        Serial_Print("\"ok\"]");
                        break;
                      }
                    }
                    startTimers(_pulsedistance);                                                // restart the measurement light timer
                  }
                  else if (_message_type == "prompt") {                                                    // wait for user to input information, followed by +
                    stopTimers();                                                                         // pause the timers (so the measuring light doesn't stay on
                    char response[100];
                    Serial_Input_Chars(response, "+", 0, sizeof(response));
                    Serial_Print("\"");
                    Serial_Print(response);
                    Serial_Print("\"]");
                    startTimers(_pulsedistance);                                                // restart the measurement light timer
                  }
                  if (cycle != pulses.getLength() - 1) {                                                  // if it's not the last cycle, then add comma
                    Serial_Print(",");
                  }
                  else {                                                                                 // if it is the last cycle, then close out the array
                    Serial_Print("],");
                  }
                }
              }

              // calculate_intensity(_meas_light, tcs_to_act, cycle, _light_intensity);                   // in addition, calculate the intensity of the current measuring light

              //            Serial_Printf("_meas_light = %d, par_to_dac = %d, _m_intensity = %d, dac_lights = %d\n",_meas_light,par_to_dac(_m_intensity, _meas_light),_m_intensity,dac_lights);
              if (!dac_lights) {                                                                            // evaluate as an expression...
                DAC_set(_meas_light, par_to_dac(_m_intensity, _meas_light));                                // set the DAC, make sure to convert PAR intensity to DAC value
              }
              else {                                                                                      // otherwise evaluate directly as a number to enter into the DAC
                DAC_set(_meas_light, _m_intensity);                                // set the DAC, make sure to convert PAR intensity to DAC value
              }
              DAC_change();

              for (unsigned i = 0; i < NUM_LEDS; i++) {                         // set the DAC lights for actinic lights in the current pulse set
                if (_a_lights[i] != 0) {                                        // if there's a light there, then change it, otherwise skip
                  if (!dac_lights) {                                                                            // evaluate as an expression...
                    DAC_set(_a_lights[i], par_to_dac(_a_intensities[i], _a_lights[i]));
                  }
                  else {                                                                                      // otherwise evaluate directly as a number to enter into the DAC
                    DAC_set(_a_lights[i], _a_intensities[i]);
                  }
                  if (PULSERDEBUG) {
                    Serial_Printf("actinic pin : %d \nactinic intensity %d \n", _a_lights[i], _a_intensities[i]);
                    Serial_Printf("length of _a_lights : %d \n ", sizeof(_a_lights));
                    Serial_Printf("\n _number_samples, _reference, adc_show: %d %d %d\n", _number_samples, _reference, adc_show);
                  } // PULSERDEBUG
                }
              } // for
            }

            if (Serial_Available() && Serial_Input_Long("+", 1) == -1) {                                      // exit protocol completely if user enters -1+
              q = number_of_protocols;
              y = measurements - 1;
              u = protocols - 1;
              x = averages;
              z = total_pulses;
            }

            uint16_t sample_adc[_number_samples];                                                             // initialize the variables to hold the main and reference detector data
            uint16_t sample_adc_ref[_number_samples];
            //            uint16_t startTimer;                                                                            // to measure the actual time it takes to perform the ADC reads on the sample (for debugging)
            //            uint16_t endTimer;

            while (led_off == 0) {                                                                     // wait for LED pulse complete (in ISR)
              if (abort_cmd()) goto abort;   // or just reboot?
            }

            if (_reference != 0) {
              AD7689_read_arrays((detector - 1), sample_adc, (_reference - 1), sample_adc_ref, _number_samples); // also reads reference detector - note this function takes detectors 0 - 3, so must subtract detector value by 1
            }
            else {
              AD7689_read_array(sample_adc, _number_samples);                                              // just reads the detector defined by the user
            }

            interrupts();                                             // re-enable interrupts (left off after LED ISR)

            digitalWriteFast(HOLDM, HIGH);                            // discharge integrators
            digitalWriteFast(HOLDADD, HIGH);

            if (adc_show == 1) {                                                                        // save the individual ADC measurements separately if adc_show is set to 1 (to be printed to the output later instead of data_raw)
              //              Serial_Print(",\"last_adc_ref\":[");
              for (unsigned i = 0; i < sizeof(sample_adc) / sizeof(uint16_t); i++) {                           //
                adc_only[i] = sample_adc[i];
                //                Serial_Print(adc_only[i]);
                //                if (i != sizeof(sample_adc)/sizeof(uint16_t) - 1) {
                //                  Serial_Print(",");
                //                }
              }
              //              Serial_Print("],");
            }
            data = median16(sample_adc, _number_samples);                                             // using median - 25% improvements over using mean to determine this value

            if (_reference != 0) {                                                        // if also using reference, then ...
              data_ref = median16(sample_adc_ref, _number_samples);                       // generate median of reference values
              if (_reference_flag == 0) {                                                 // if this is the first pulse set which uses the reference, then flip the flag noting that a reference measurement has been taken, and collect the main and reference detectors starting measurements for normalization
                _reference_start = data_ref;
                _main_start = data;
                _reference_flag = 1;
              }
            }
            if (_reference != 0) {                                                        // now calculate the outputted data value based on the main and reference values, and the initial main and reference values for normalization
              if (PULSERDEBUG) {
                Serial_Printf("reference_start = %d, reference_now = %d,       _main_start = %d, main_now = %d", (int) _reference_start, (int) data_ref, (int) _main_start, (int) data);
              } // PULSERDEBUG

              data = data - _main_start * (data_ref - _reference_start) / _reference_start; // adjust main value according to the % change in the reference relative to main on the first pulse.  You adjust the main in the opposite direction of the reference (to compensate)

              if (PULSERDEBUG) {
                float changed = (data_ref - _reference_start) / _reference_start;
                Serial_Printf(",      main = %d, ref = %d, data_normalized = %f, percent_change = %f\n", detector, _reference, data, changed);
              } // PULSERDEBUG
            }

            if (PULSERDEBUG) {        // use this to see all of the adc reads which are being averaged
              Serial_Printf("median + first value :%d,%d", data, sample_adc[5]);
              Serial_Printf("median + first value reference :%d,%d", data_ref, sample_adc_ref[5]);
            } // PULSERDEBUG

            if (first_flag == 1) {                                                                    // if this is the 0th pulse and a therefore new cycle
              for (unsigned i = 0; i < NUM_LEDS; i++) {                            // Turn off all of the previous actinic lights
                if (_a_lights_prev[i] != 0) {                                                                 // just skip it if it's zero
                  digitalWriteFast(LED_to_pin[_a_lights_prev[i]], LOW);
                  if (PULSERDEBUG) {
                    Serial_Printf("turned off actinic light: %d\n", LED_to_pin[_a_lights_prev[i]]);
                  } // PULSERDEBUG
                }
              }
              DAC_change();                                                                               // initiate actinic lights which were set above

              for (unsigned i = 0; i < NUM_LEDS; i++) {                            // Turn on all the new actinic lights for this pulse set
                if (_a_lights[i] != 0) {                                                                 // just skip it if it's zero
                  digitalWriteFast(LED_to_pin[_a_lights[i]], HIGH);
                  if (PULSERDEBUG) {
                    Serial_Printf("turned on new actinic light: %d\n", LED_to_pin[_a_lights[i]]);
                  } // PULSERDEBUG
                }
              }
              first_flag = 0;                                                              // reset flag
            }

            float _offset = 0;
            /*
                        if (offset_off == 0) {
                          switch (detector) {                                                          // apply offset to whicever detector is being used
                            case 34:
                              _offset = offset_34;
                              break;
                            case 35:
                              _offset = offset_35;
                              break;
                          }
                        }
            */

            if (DEBUGSIMPLE) {
              Serial_Print("data count, size of raw data                                   ");
              Serial_Print((int)data_count);
              Serial_Print(",");
              Serial_Print_Line(size_of_data_raw);
            } // DEBUGSIMPLE

            if (_spec != 1) {                                                    // if spec_on is not equal to 1, then coralspeq is off and proceed as per normal MultispeQ measurement.
              if (_meas_light  != 0) {                                                      // save the data, so long as the measurement light is not equal to zero.
                data_raw_average[data_count] += data - _offset;
                data_count++;
              }
            }
            else if (_spec == 1) {                                              // if spec_on is 1 for this cycle, then collect data from the coralspeq and save it to data_raw_average.
              readSpectrometer(_intTime, _delay_time, _read_time, _accumulateMode);                                                        // collect a reading from the spec
              for (int i = 0 ; i < SPEC_CHANNELS; i++) {
                data_raw_average[data_count] += spec_data[i];
                data_count++;
              }
            }

            noInterrupts();                                                              // turn off interrupts because we're checking volatile variables set in the interrupts
            led_off = 0;                                                                // reset pulse status flags
            pulse++;                                                                     // progress the pulse counter and measurement number counter

            if (DEBUGSIMPLE) {
              Serial_Print("data point average, current data                               ");
              Serial_Print((int)data_raw_average[meas_number]);
              Serial_Print("!");
              Serial_Print_Line(data);
            } // DEBUGSIMPLE

            interrupts();                                                              // done with volatile variables, turn interrupts back on
            meas_number++;                                                              // progress measurement number counters

            if (pulse == pulses.getLong(cycle)*meas_lights.getArray(cycle).getLength()) { // if it's the last pulse of a cycle...
              pulse = 0;                                                               // reset pulse counter
              cycle++;                                                                 // ...move to next cycle
            }
          }
          background_on = 0;
          /*
                    background_on = calculate_intensity_background(act_background_light, tcs_to_act, cycle, _light_intensity, act_background_light_intensity); // figure out background light intensity and state
          */

          for (unsigned i = 0; i < NUM_LEDS; i++) {
            if (_a_lights[i] != act_background_light) {                                  // turn off all lights unless they are the actinic background light
              digitalWriteFast(LED_to_pin[_a_lights[i]], LOW);
            }
          }

          if (background_on == 1) {
            DAC_change();                                                                               // initiate actinic lights which were set above
            digitalWriteFast(act_background_light, HIGH);                                // turn on actinic background light in case it was off previously.
          }
          else {
            digitalWriteFast(act_background_light, LOW);                                // turn on actinic background light in case it was off previously.
          }

          stopTimers();
          cycle = 0;                                                                     // ...and reset counters
          pulse = 0;
          led_off = 0;
          meas_number = 0;

          /*
            options for relative humidity, temperature, contactless temperature. light_intensity,co2
            0 - take before spectroscopy measurements
            1 - take after spectroscopy measurements
          */
          environmentals(environmental, averages, x, 1);

          if (x + 1 < averages) {                                                             //  to next average, unless it's the end of the very last run
            if (averages_delay > 0) {
              Serial_Input_Long("+", averages_delay * 1000);
            }
            if (averages_delay_ms > 0) {
              Serial_Input_Long("+", averages_delay_ms);
            }
          }

        }  // for each protocol repeat

        /*
           Recall and save values to the eeprom
        */

        recall_save(recall_eeprom, save_eeprom);                                                    // Recall and save values to the eeprom.  If you save values to eeprom, you get those saved values returned to you to confirm that they have been saved successfully (so save automatically calls recall once complete)

        if (spec_on == 1) {                                                                    // if the spec is being used, then read it and print data_raw as spec values.  Otherwise, print data_raw as multispeq detector values as per normal
          Serial_Print("\"data_raw\":[");
          for (int i = 0; i < SPEC_CHANNELS; i++) {
            Serial_Print((unsigned)(spec_data_average[i] / averages));
            if (i != SPEC_CHANNELS - 1) {                                                     // if it's the last one in printed array, don't print comma
              Serial_Print(",");
            }
          } // for
          Serial_Print("]}");
        }

        if (spec_on == 0) {
          Serial_Print("\"data_raw\":[");
          if (adc_show == 0) {                                                             // normal condition - show data_raw as per usual
            for (int i = 0; i < size_of_data_raw; i++) {                                     // print data_raw, divided by the number of averages
              Serial_Print((unsigned)(data_raw_average[i] / averages));
              // if average = 1, then it would be better to print data as it is collected
              if (i != size_of_data_raw - 1) {
                Serial_Print(",");
              }
            }
          }
          else {                                                                         // if adc_show == 1, show first individual adc's only - do not show normal data_raw (used for signal debugging only)
            for (int i = 0; i < number_samples.getLong(0); i++) {
              Serial_Print(adc_only[i]);
              if (i != number_samples.getLong(0) - 1) {
                Serial_Print(",");
              }
            }
          }
          Serial_Print("]}");
        }

        if (DEBUGSIMPLE) {
          Serial_Print("# of protocols repeats, current protocol repeat, number of total protocols, current protocol      ");
          Serial_Print(protocols);
          Serial_Print(",");
          Serial_Print(u);
          Serial_Print(",");
          Serial_Print(number_of_protocols);
          Serial_Print(",");
          Serial_Print_Line(q);
        } // DEBUGSIMPLE

        if (q < number_of_protocols - 1 || u < protocols - 1) {                           // if it's not the last protocol in the measurement and it's not the last repeat of the current protocol, add a comma
          Serial_Print(",");
          if (protocols_delay > 0) {
            Serial_Input_Long("+", protocols_delay * 1000);
          }
          if (protocols_delay_ms > 0) {
            Serial_Input_Long("+", protocols_delay_ms);
          }
        }
        else if (q == number_of_protocols - 1 && u == protocols - 1) {                  // if it is the last protocol, then close out the data json
          Serial_Print("]");
        }

        averages = 1;                                                 // number of times to repeat the entire run
        averages_delay = 0;                                                           // seconds wait time between averages
        averages_delay_ms = 0;                                                    // seconds wait time between averages
        analog_averageds = 1;                                                             // # of measurements per pulse to be averaged (min 1 measurement per 6us pulselengthon)
        for (unsigned i = 0; i < NUM_LEDS; i++) {
          _a_lights[i] = 0;
        }
        /*
          relative_humidity_average = 0;                                                // reset all environmental variables to zero
          temperature_average = 0;
          objt_average = 0;
          lux_averaged = 0;
          r_averaged = 0;
          g_averaged = 0;
          b_averaged = 0;
          lux_averaged_forpar = 0;
          r_averaged_forpar = 0;
          g_averaged_forpar = 0;
          b_averaged_forpar = 0;
        */

        if (CORAL_SPEQ) {
          for (int i = 0; i < SPEC_CHANNELS; i++)
            spec_data_average [i] = 0;
        } // CORAL_SPEQ

        act_background_light_prev = act_background_light;                               // set current background as previous background for next protocol
        spec_on = 0;                                                                    // reset flag that spec is turned on for this measurement

        if (DEBUGSIMPLE) {
          Serial_Print_Line("previous light set to:   ");
          Serial_Print_Line(act_background_light_prev);
        } // DEBUGSIMPLE
      }
    }
    Serial_Flush_Input();
    if (y < measurements - 1) {                                                    // add commas between measurements
      Serial_Print(",");
      if (measurements_delay > 0) {
        Serial_Input_Long("+", measurements_delay * 1000);
      }
      else if (measurements_delay_ms > 0) {
        Serial_Input_Long("+", measurements_delay_ms);
      }
    } // if

  }  // for each measurement

  /*
    // make sure one last time that all of the lights are turned off, including background light!
    for (unsigned i = 0; i < sizeof(LED_to_pin) / sizeof(unsigned short); i++) {
      digitalWriteFast(LED_to_pin[i], LOW);
      Serial_Print_Line(LED_to_pin[i]);
    }
  */

abort:

  Serial_Print("]}");                // terminate output json
  Serial_Print_CRC();
  Serial_Flush_Output();

  act_background_light = 0;          // ??

  // turn off all lights (just in case)
  for (unsigned i = 1; i <= NUM_LEDS; i++) {
    digitalWriteFast(LED_to_pin[i], LOW);
  }

  return;

} // do_protocol()


//  routines for LED pulsing

static void pulse3() {                           // ISR to turn on/off LED pulse - also controls integration switch
  const unsigned  STABILIZE = 10;                // this delay gives the LED current controller op amp the time needed to stabilize

  register int pin = LED_to_pin[_meas_light];
  register int pulse_size = _pulsesize;
  noInterrupts();
  digitalWriteFast(pin, HIGH);            // turn on measuring light
  delayMicroseconds(STABILIZE);           // this delay gives the LED current controller op amp the time needed to turn
  // the light on completely + stabilize.
  // Very low intensity measuring pulses may require an even longer delay here.
  digitalWriteFast(HOLDADD, LOW);        // turn off sample and hold discharge
  digitalWriteFast(HOLDM, LOW);          // turn off sample and hold discharge
  delayMicroseconds(pulse_size);         // pulse width
  digitalWriteFast(pin, LOW);            // turn off measuring light
  led_off = 1;                           // indicate that we are done
  // NOTE:  interrupts are left off and must be re-enabled
}

// schedule the turn on and off of the LED(s) via a single ISR

static IntervalTimer timer0;

inline static void startTimers(unsigned _pulsedistance) {
  timer0.begin(pulse3, _pulsedistance);             // schedule pulses
}

inline static void stopTimers() {
  timer0.end();                         // if it's the last cycle and last pulse, then... stop the timers
}

// write userdef values to eeprom
// example json for save: [{"save":[[1,3.43],[2,5545]]}]  for userdef[1] = 3.43 and userdef[2] = 5545
// read userdef and other values from eeprom
// example json for read: [{"recall":["light_slope_all","userdef[1]"]}]

static void recall_save(JsonArray _recall_eeprom, JsonArray _save_eeprom) {
  int number_saves = _save_eeprom.getLength();                                // define these explicitly to make it easier to understand the logic
  int number_recalls = _recall_eeprom.getLength();                            // define these explicitly to make it easier to understand the logic

  for (int i = 0; i < number_saves; i++) {                             // do any saves
    long location = _save_eeprom.getArray(i).getLong(0);
    double value_to_save = _save_eeprom.getArray(i).getDouble(1);
    if (location >= 0 && location <= (long)NUM_USERDEFS)
      store(userdef[location], value_to_save);                         // save new value in the defined eeprom location
  }

  if (number_recalls > 0) {                  // if the user is recalling any saved eeprom values then...
    Serial_Print("\"recall\":{");                                                       // then print the eeprom location number and the value located there

    for (int i = 0; i < number_recalls; i++) {

      String recall_string = _recall_eeprom.getString(i);

      Serial_Printf("\"%s\":%f", recall_string.c_str(), expr(recall_string.c_str()));

      if (i != number_recalls - 1) {
        Serial_Print(",");
      }
      else {
        Serial_Print("},");
      }
    } // for
  } // recall_save()
}

// return true if a Ctrl-A character has been typed
int abort_cmd()
{
  return 0;    // TODO
}

/*
   The structure of the sensor calls follows these general rules:
   1) user enters "environmental":[[...],[...]] structure into API, where ... is the call ("light_intensity" or "thickness" for example), and followed by a 0 or 1 (0 if it's before the main spec measurement, or 1 if it's after)
      An example - "environmental":[["tilt",1],["light_intensity",0]] would call tilt after the measurement, and light intensity before the measurement.
   2) Sensor data gets called in 3 ways: either inside the measurement as part of an expression (maybe "a_intensities":[light_intensity/2]), after a measurement or set of averaged measurements.
      In addition, there are often raw and calibrated versions of sensors, like raw tcs value versus the PAR value, or raw hall sensor versus calibrated thickness.
      As a result, for most sensors there is the base version (like x_tilt) which is available in that measurment, an averaged version (like x_tilt_averaged) which is outputted after averaging, and raw versions of each of those (x_tilt_raw and x_tilt_raw_averaged)
*/
void get_temperature_humidity_pressure (int _averages) {    // read temperature, relative humidity, and pressure BME280 module

  temperature = bme1.readTempC();                // temperature in C
  humidity = bme1.readHumidity();                      // humidity in %
  pressure = bme1.readPressure() / 100;               // pressure in millibar

  temperature_averaged += temperature / _averages;                // same as above, but averaged if API requests averaging
  humidity_averaged += humidity / _averages;
  pressure_averaged += pressure / _averages;

}

void get_temperature_humidity_pressure2 (int _averages) {    // read temperature, relative humidity, and pressure BME280 module

  temperature2 = bme2.readTempC();
  humidity2 = bme2.readHumidity();
  pressure2 = bme2.readPressure() / 100;

  temperature2_averaged += temperature2 / _averages;                // collect temp, rh, and humidity data (averaged if API requests averaging)
  humidity2_averaged += humidity2 / _averages;
  pressure2_averaged += pressure2 / _averages;

}

float get_contactless_temp (int _averages) {
  contactless_temp = (MLX90615_Read(0) + MLX90615_Read(0) + MLX90615_Read(0)) / 3.0;
  contactless_temp_averaged += contactless_temp / _averages;
  return contactless_temp;
}

// read accelerometer

void get_tilt (int notRaw, int _averages) {
  MMA8653FC_read(&x_tilt_raw, &y_tilt_raw, &z_tilt_raw);                      // saves x_tilt_raw, y_... and z_... values
  // consider adding a calibration with more useful outputs
  x_tilt = x_tilt_raw * (180. / 1000);
  y_tilt = y_tilt_raw * (180. / 1000);
  z_tilt = z_tilt_raw * (180. / 1000);
  if (notRaw == 0) {                                              // save the raw values average
    x_tilt_raw_averaged += (float)x_tilt_raw / _averages;
    y_tilt_raw_averaged += (float)y_tilt_raw / _averages;
    z_tilt_raw_averaged += (float)z_tilt_raw / _averages;
  }
  if (notRaw == 1) {                                              // save the calibrated values and average
    x_tilt_averaged += (float)x_tilt / _averages;
    y_tilt_averaged += (float)y_tilt / _averages;
    z_tilt_averaged += (float)z_tilt / _averages;
  }
  // add better routine here to produce clearer tilt values
}

// read compass

void get_cardinal (int notRaw, int _averages) {
  MAG3110_read(&x_cardinal_raw, &y_cardinal_raw, &z_cardinal_raw);            // saves x_cardinal, y_ and z_ values
  // add calibration here to give 0 - 360 directional values or N / NE / E / SE / S / SW / W / NW save that to variable called cardinal
  if (notRaw == 0) {                                              // save the raw values average
    x_cardinal_raw_averaged += (float)x_cardinal_raw / _averages;
    y_cardinal_raw_averaged += (float)y_cardinal_raw / _averages;
    z_cardinal_raw_averaged += (float)z_cardinal_raw / _averages;
  }
  if (notRaw == 1) {                                              // save the calibrated values and average
    cardinal_averaged += (float)x_cardinal_raw / _averages;
  }
}

// read the hall sensor to measure thickness of a leaf

float get_thickness (int notRaw, int _averages) {
  int sum = 0;
  for (int i = 0; i < 1000; ++i) {
    sum += analogRead(HALL_OUT);
  }
  thickness_raw = (sum / 1000);
  thickness = (sum / 1000);
  // add calibration routine here with calls to thickness_a thickness_b thickness_d;
  //  thickness += ...
  if (notRaw == 0) {                                              // save the raw values average
    thickness_raw_averaged += (float)thickness_raw / _averages;
    return thickness_raw;
  }
  else if (notRaw == 1) {                                              // save the calibrated values and average
    thickness_averaged += (float)thickness / _averages;
    return thickness;
  }
  else {
    return 0;
  }
}

// check for commands to read various envirmental sensors
// also output the value on the final call

static void environmentals(JsonArray environmental, const int _averages, const int count, int beforeOrAfter)
{

  for (int i = 0; i < environmental.getLength(); i++) {                                       // call environmental measurements after the spectroscopic measurement

    if (environmental.getArray(i).getLong(1) != beforeOrAfter)
      continue;            // not the right time

    if ((String) environmental.getArray(i).getString(0) == "temperature_humidity_pressure") {                   // measure light intensity with par calibration applied
      get_temperature_humidity_pressure(_averages);
      if (count == _averages - 1) {
        Serial_Printf("\"temperature\":%f,\"humidity\":%f,\"pressure\":%f,", temperature_averaged, humidity_averaged, pressure_averaged);
      }
    }
    if ((String) environmental.getArray(i).getString(0) == "temperature_humidity_pressure2") {                   // measure light intensity with par calibration applied
      get_temperature_humidity_pressure2(_averages);
      if (count == _averages - 1) {
        Serial_Printf("\"temperature2\":%f,\"humidity2\":%f,\"pressure2\":%f,", temperature2_averaged, humidity2_averaged, pressure2_averaged);
      }
    }

    if ((String) environmental.getArray(i).getString(0) == "light_intensity") {                   // measure light intensity with par calibration applied
      get_light_intensity(1, _averages);
      if (count == _averages - 1) {
        Serial_Printf("\"light_intensity\":%.2f,\"r\":%.2f,\"g\":%.2f,\"b\":%.2f,", light_intensity_averaged, r_averaged, g_averaged, b_averaged);
      }
    }

    if ((String) environmental.getArray(i).getString(0) == "light_intensity_raw") {              // measure raw light intensity from TCS sensor
      get_light_intensity(0, _averages);
      if (count == _averages - 1) {
        Serial_Printf("\"light_intensity_raw\":%.2f,\"r\":%.2f,\"g\":%.2f,\"b\":%.2f,", light_intensity_raw_averaged, r_averaged, g_averaged, b_averaged);
      }
    }

    if ((String) environmental.getArray(i).getString(0) == "contactless_temp") {                 // measure contactless temperature
      get_contactless_temp(_averages);
      if (count == _averages - 1) {
        Serial_Printf("\"contactless_temp\":%.2f,", contactless_temp_averaged);
      }
    }

    if ((String) environmental.getArray(i).getString(0) == "thickness") {                        // measure thickness via hall sensor, with calibration applied
      get_thickness(1, _averages);
      if (count == _averages - 1) {
        Serial_Printf("\"thickness\":%.2f,", thickness_averaged);
      }
    }

    if ((String) environmental.getArray(i).getString(0) == "thickness_raw") {                    // measure thickness via hall sensor, raw analog_read
      get_thickness(0, _averages);
      if (count == _averages - 1) {
        Serial_Printf("\"thickness_raw\":%.2f,", thickness_raw_averaged);
      }
    }

    if ((String) environmental.getArray(i).getString(0) == "tilt") {                             // measure tilt in -180 - 180 degrees
      get_tilt(1, _averages);
      if (count == _averages - 1) {
        Serial_Printf("\"x_tilt\":%.2f, \"y_tilt\":%.2f, \"z_tilt\":%.2f,", x_tilt_averaged, y_tilt_averaged, z_tilt_averaged);
      }
    }

    if ((String) environmental.getArray(i).getString(0) == "tilt_raw") {                         // measure tilt from -1000 - 1000
      get_tilt(0, _averages);
      if (count == _averages - 1) {
        Serial_Printf("\"x_tilt_raw\":%.2f, \"y_tilt_raw\":%.2f, \"z_tilt_raw\":%.2f,", x_tilt_raw_averaged, y_tilt_raw_averaged, z_tilt_raw_averaged);
      }
    }

    if ((String) environmental.getArray(i).getString(0) == "cardinal") {                         // measure cardinal direction, with calibration applied
      get_cardinal(1, _averages);
      if (count == _averages - 1) {
        Serial_Printf("\"cardinal\":%.2f,", cardinal_averaged);
      }
    }

    if ((String) environmental.getArray(i).getString(0) == "cardinal_raw") {                     // measure cardinal direction, raw values
      get_cardinal(0, _averages);
      if (count == _averages - 1) {
        Serial_Printf("\"x_cardinal_raw\":%.2f,\"y_cardinal_raw\":%.2f,\"z_cardinal_raw\":%.2f,", x_cardinal_raw_averaged, y_cardinal_raw_averaged, z_cardinal_raw_averaged);
      }
    }

    if ((String) environmental.getArray(i).getString(0) == "analog_read") {                      // perform analog reads
      int pin = environmental.getArray(i).getLong(2);
      pinMode(pin, INPUT);
      int analog_read = analogRead(pin);
      if (count == _averages - 1) {
        Serial_Printf("\"analog_read\":%f,", analog_read);
      }
    }

    if ((String) environmental.getArray(i).getString(0) == "digital_read") {                      // perform digital reads
      int pin = environmental.getArray(i).getLong(2);
      pinMode(pin, INPUT);
      int digital_read = digitalRead(pin);
      if (count == _averages - 1) {
        Serial_Printf("\"digital_read\":%f,", digital_read);
      }
    }

    if ((String) environmental.getArray(i).getString(0) == "digital_write") {                      // perform digital write
      int pin = environmental.getArray(i).getLong(2);
      int setting = environmental.getArray(i).getLong(3);
      pinMode(pin, OUTPUT);
      digitalWriteFast(pin, setting);
    }

    if ((String) environmental.getArray(i).getString(0) == "analog_write") {                      // perform analog write with length of time to apply the pwm
      int pin = environmental.getArray(i).getLong(2);
      int setting = environmental.getArray(i).getLong(3);
      int freq = environmental.getArray(i).getLong(4);
      int wait = environmental.getArray(i).getLong(5);

      // TODO sanity checks

      if (DEBUGSIMPLE) {
        Serial_Print_Line(pin);
        Serial_Print_Line(pin);
        Serial_Print_Line(wait);
        Serial_Print_Line(setting);
        Serial_Print_Line(freq);
      } // DEBUGSIMPLE

      pinMode(pin, OUTPUT);
      analogWriteFrequency(pin, freq);                                                           // set analog frequency
      analogWrite(pin, setting);
      delay(wait);
      analogWrite(pin, 0);
      //reset_freq();                                                                              // reset analog frequencies
    } // if
  } // for
}  //environmentals()

static void print_all () {
  // print every value saved in eeprom in valid json structure (even the undefined values which are still 0)
}

static void print_userdef () {
  // print only the userdef values which can be defined by the user
}

// ??  why
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


void print_calibrations() {
  Serial_Printf("{\n\"device_id\":\"d4:f5:%x:%x:%x:%x\",\n",
                (unsigned)eeprom->device_id >> 24,
                ((unsigned)eeprom->device_id & 0xff0000) >> 16,
                ((unsigned)eeprom->device_id & 0xff00) >> 8,
                (unsigned)eeprom->device_id & 0xff
               );
  Serial_Printf("\"device_manufacture\": \"%d\",\n", eeprom->device_manufacture);
  Serial_Printf("\"mag_bias\": [\"%f\",\"%f\",\"%f\"],\n", eeprom->mag_bias[0], eeprom->mag_bias[1], eeprom->mag_bias[2]);
  Serial_Printf("\"mag_cal\": [[\"%f\",\"%f\",\"%f\"],[\"%f\",\"%f\",\"%f\"],[\"%f\",\"%f\",\"%f\"]],\n", eeprom->mag_cal[0][0], eeprom->mag_cal[0][1], eeprom->mag_cal[0][2], eeprom->mag_cal[1][0], eeprom->mag_cal[1][1], eeprom->mag_cal[1][2], eeprom->mag_cal[2][0], eeprom->mag_cal[2][1], eeprom->mag_cal[2][2]);
  Serial_Printf("\"accel_bias\": [\"%f\",\"%f\",\"%f\"],\n", eeprom->accel_bias[0], eeprom->accel_bias[1], eeprom->accel_bias[2]);
  Serial_Printf("\"accel_cal\": [[\"%f\",\"%f\",\"%f\"],[\"%f\",\"%f\",\"%f\"],[\"%f\",\"%f\",\"%f\"]],\n", eeprom->accel_cal[0][0], eeprom->accel_cal[0][1], eeprom->accel_cal[0][2], eeprom->accel_cal[1][0], eeprom->accel_cal[1][1], eeprom->accel_cal[1][2], eeprom->accel_cal[2][0], eeprom->accel_cal[2][1], eeprom->accel_cal[2][2]);
  Serial_Printf("\"light_slope_all\": \"%f\",\n", eeprom->light_slope_all);
  Serial_Printf("\"light_slope_r\": \"%f\",\n", eeprom->light_slope_r);
  Serial_Printf("\"light_slope_g\": \"%f\",\n", eeprom->light_slope_g);
  Serial_Printf("\"light_slope_b\": \"%f\",\n", eeprom->light_slope_b);
  Serial_Printf("\"light_yint\": \"%f\",\n", eeprom->light_yint);
  Serial_Printf("\"light_slope_all\": \"%f\",\n", eeprom->light_slope_all);
  Serial_Printf("\"detector_offset_slope\": [\"%f\",\"%f\",\"%f\",\"%f\"],\n", eeprom->detector_offset_slope[0], eeprom->detector_offset_slope[1], eeprom->detector_offset_slope[2], eeprom->detector_offset_slope[3]);
  Serial_Printf("\"detector_offset_yint\": [\"%f\",\"%f\",\"%f\",\"%f\"],\n", eeprom->detector_offset_yint[0], eeprom->detector_offset_yint[1], eeprom->detector_offset_yint[2], eeprom->detector_offset_yint[3]);
  Serial_Printf("\"thickness_a\": \"%f\",\n", eeprom->thickness_a);
  Serial_Printf("\"thickness_b\": \"%f\",\n", eeprom->thickness_b);
  Serial_Printf("\"thickness_d\": \"%f\",\n", eeprom->thickness_d);
  Serial_Print("\"par_to_dac_slope\": [");
  for (unsigned i = 0; i < sizeof(eeprom->par_to_dac_slope) / sizeof(float); i++) {
    if (i != sizeof(eeprom->par_to_dac_slope) / sizeof(float) - 1) {
      Serial_Printf("\"%f\",", eeprom->par_to_dac_slope[i]);
    }
    else {
      Serial_Printf("\"%f\"],\n", eeprom->par_to_dac_slope[i]);
    }
  }

  Serial_Print("\"par_to_dac_yint\": [");
  for (unsigned i = 0; i < sizeof(eeprom->par_to_dac_yint) / sizeof(float); i++) {
    if (i != sizeof(eeprom->par_to_dac_yint) / sizeof(float) - 1) {
      Serial_Printf("\"%f\",", eeprom->par_to_dac_yint[i]);
    }
    else {
      Serial_Printf("\"%f\"],\n", eeprom->par_to_dac_yint[i]);
    }
  }

  Serial_Print("\"ir_baseline_slope\": [");
  for (unsigned i = 0; i < sizeof(eeprom->ir_baseline_slope) / sizeof(float); i++) {
    if (i != sizeof(eeprom->ir_baseline_slope) / sizeof(float) - 1) {
      Serial_Printf("\"%f\",", eeprom->ir_baseline_slope[i]);
    }
    else {
      Serial_Printf("\"%f\"],\n", eeprom->ir_baseline_slope[i]);
    }
  }

  Serial_Print("\"ir_baseline_slope\": [");
  for (unsigned i = 0; i < sizeof(eeprom->ir_baseline_yint) / sizeof(float); i++) {
    if (i != sizeof(eeprom->ir_baseline_yint) / sizeof(float) - 1) {
      Serial_Printf("\"%f\",", eeprom->ir_baseline_yint[i]);
    }
    else {
      Serial_Printf("\"%f\"],\n", eeprom->ir_baseline_yint[i]);
    }
  }

  Serial_Print("\"colorcal_intensity1_slope\": [");
  for (unsigned i = 0; i < sizeof(eeprom->colorcal_intensity1_slope) / sizeof(float); i++) {
    if (i != sizeof(eeprom->colorcal_intensity1_slope) / sizeof(float) - 1) {
      Serial_Printf("\"%f\",", eeprom->colorcal_intensity1_slope[i]);
    }
    else {
      Serial_Printf("\"%f\"],\n", eeprom->colorcal_intensity1_slope[i]);
    }
  }
  Serial_Print("\"colorcal_intensity1_yint\": [");
  for (unsigned i = 0; i < sizeof(eeprom->colorcal_intensity1_yint) / sizeof(float); i++) {
    if (i != sizeof(eeprom->colorcal_intensity1_yint) / sizeof(float) - 1) {
      Serial_Printf("\"%f\",", eeprom->colorcal_intensity1_yint[i]);
    }
    else {
      Serial_Printf("\"%f\"],\n", eeprom->colorcal_intensity1_yint[i]);
    }
  }
  Serial_Print("\"colorcal_intensity2_slope\": [");
  for (unsigned i = 0; i < sizeof(eeprom->colorcal_intensity2_slope) / sizeof(float); i++) {
    if (i != sizeof(eeprom->colorcal_intensity2_slope) / sizeof(float) - 1) {
      Serial_Printf("\"%f\",", eeprom->colorcal_intensity2_slope[i]);
    }
    else {
      Serial_Printf("\"%f\"],\n", eeprom->colorcal_intensity2_slope[i]);
    }
  }
  Serial_Print("\"colorcal_intensity2_yint\": [");
  for (unsigned i = 0; i < sizeof(eeprom->colorcal_intensity2_yint) / sizeof(float); i++) {
    if (i != sizeof(eeprom->colorcal_intensity2_yint) / sizeof(float) - 1) {
      Serial_Printf("\"%f\",", eeprom->colorcal_intensity2_yint[i]);
    }
    else {
      Serial_Printf("\"%f\"],\n", eeprom->colorcal_intensity2_yint[i]);
    }
  }
  Serial_Print("\"colorcal_intensity3_slope\": [");
  for (unsigned i = 0; i < sizeof(eeprom->colorcal_intensity3_slope) / sizeof(float); i++) {
    if (i != sizeof(eeprom->colorcal_intensity3_slope) / sizeof(float) - 1) {
      Serial_Printf("\"%f\",", eeprom->colorcal_intensity3_slope[i]);
    }
    else {
      Serial_Printf("\"%f\"],\n", eeprom->colorcal_intensity3_slope[i]);
    }
  }
  Serial_Print("\"colorcal_intensity3_yint\": [");
  for (unsigned i = 0; i < sizeof(eeprom->colorcal_intensity3_yint) / sizeof(float); i++) {
    if (i != sizeof(eeprom->colorcal_intensity3_yint) / sizeof(float) - 1) {
      Serial_Printf("\"%f\",", eeprom->colorcal_intensity3_yint[i]);
    }
    else {
      Serial_Printf("\"%f\"],\n", eeprom->colorcal_intensity3_yint[i]);
    }
  }
  Serial_Print("\"colorcal_blank1\": [");
  for (unsigned i = 0; i < sizeof(eeprom->colorcal_blank1) / sizeof(float); i++) {
    if (i != sizeof(eeprom->colorcal_blank1) / sizeof(float) - 1) {
      Serial_Printf("\"%f\",", eeprom->colorcal_blank1[i]);
    }
    else {
      Serial_Printf("\"%f\"],\n", eeprom->colorcal_blank1[i]);
    }
  }
  Serial_Print("\"colorcal_blank2\": [");
  for (unsigned i = 0; i < sizeof(eeprom->colorcal_blank2) / sizeof(float); i++) {
    if (i != sizeof(eeprom->colorcal_blank2) / sizeof(float) - 1) {
      Serial_Printf("\"%f\",", eeprom->colorcal_blank2[i]);
    }
    else {
      Serial_Printf("\"%f\"],\n", eeprom->colorcal_blank2[i]);
    }
  }
  Serial_Print("\"colorcal_blank3\": [");
  for (unsigned i = 0; i < sizeof(eeprom->colorcal_blank3) / sizeof(float); i++) {
    if (i != sizeof(eeprom->colorcal_blank3) / sizeof(float) - 1) {
      Serial_Printf("\"%f\",", eeprom->colorcal_blank3[i]);
    }
    else {
      Serial_Printf("\"%f\"],\n", eeprom->colorcal_blank3[i]);
    }
  }

  for (unsigned i = 0; i < NUM_USERDEFS; i++) {
    if (i != NUM_USERDEFS - 1) {
      Serial_Printf("\"userdef%d\": \"%f\",\n", i, eeprom->userdef[i]);
    }
    else {
      Serial_Printf("\"userdef%d\": \"%f\"\n}", i, eeprom->userdef[i]);
    }
  }
  Serial_Print_CRC();
}

