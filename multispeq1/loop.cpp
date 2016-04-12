
// main loop and some support routines

#include "defines.h"
#include <Time.h>                                                             // enable real time clock library
#include "utility/Adafruit_Sensor.h"
#include "json/JsonParser.h"
#include "utility/mcp4728.h"              // delete this once PAR is fixed
#include "DAC.h"
#include "AD7689.h"               // external ADC
#include "utility/Adafruit_BME280.h"      // temp/humidity/pressure sensor
#include "eeprom.h"
#include <ADC.h>                  // internal ADC
#include "serial.h"
#include "flasher.h"
#include "utility/crc32.h"

// local defines

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


// function declarations


void startTimers(uint16_t _pulsedistance, uint16_t _pulsesize);
void stopTimers(void);
void reset_freq(void);
void upgrade_firmware(void); // for over-the-air firmware updates
void boot_check(void);  // for over-the-air firmware updates
int Light_Intensity(int var1);
void recall_save(JsonArray _recall_eeprom, JsonArray _save_eeprom);
void set_device_info(const int _set);


// Globals (try to avoid)

// map LED to MCU pin
unsigned short LED_to_pin[NUM_LEDS + 1] = {0, PULSE1, PULSE2, PULSE3, PULSE4, PULSE5, PULSE6, PULSE7, PULSE8, PULSE9, PULSE10 }; // NOTE!  We skip the first element in the array so that the array lines up correctly (PULSE1 == 1, PULSE2 == 2 ... )

// ???
int averages = 1;
int _meas_light;           // measuring light to be used during the interrupt
int spec_on = 0;                                           // flag to indicate that spec is being used during this measurement

static const int serial_buffer_size = 5000;                                        // max size of the incoming jsons
static const int max_jsons = 15;                                                   // max number of protocols per measurement

static volatile int led_off = 0;
static uint16_t _pulsesize = 0;
//int analogresolutionvalue;
IntervalTimer timer0;
float data = 0;
float data_ref = 0;
int act_background_light = 0;
//extern float light_y_intercept;
//char* bt_response = "OKOKlinvorV1.8OKsetPINOKsetnameOK115200"; // Expected response from bt module after programming is done.
//float freqtimer0;
//float freqtimer1;
//float freqtimer2;

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


//////////////////////// MAIN LOOP /////////////////////////

// process ascii serial input commands of two forms:
// 1010+<parameter1>+<parameter2>+...  (a command)
// [...] (a json protocol to be executed)

void loop() {

  delay(50);  // ??

  int measurements = 1;                                      // the number of times to repeat the entire measurement (all protocols)
  unsigned long measurements_delay = 0;                      // number of seconds to wait between measurements
  unsigned long measurements_delay_ms = 0;                   // number of milliseconds to wait between measurements
  volatile unsigned long meas_number = 0;                    // counter to cycle through measurement lights 1 - 4 during the run
  //unsigned long end1;
  //unsigned long start1 = millis();

  // these variables could be pulled from the JSON at the time of use... however, because pulling from JSON is slow, it's better to create a int to save them into at the beginning of a protocol run and use the int instead of the raw hashTable.getLong type call
  int _a_lights [10] = {};
  int _a_intensities [10] = {};
  int _a_lights_prev [10] = {};
  int act_background_light_prev = 0;
  int cycle = 0;                                                                // current cycle number (start counting at 0!)
  int pulse = 0;                                                                // current pulse number
  //int total_cycles;                                                           // Total number of cycles - note first cycle is cycle 0
  int meas_array_size = 0;                                                      // measures the number of measurement lights in the current cycle (example: for meas_lights = [[15,15,16],[15],[16,16,20]], the meas_array_size's are [3,1,3].

  char* json = (char*)malloc(1);
  JsonHashTable hashTable;
  JsonParser<600> root;
  String json2 [max_jsons];

  //int end_flag = 0;
  unsigned long* data_raw_average = 0;                                          // buffer for ADC output data
  char serial_buffer [serial_buffer_size];                                      // large buffer for reading in a json protocol from serial port

  memset(serial_buffer, 0, serial_buffer_size);                                  // reset buffer to zero
  for (int i = 0; i < max_jsons; i++) {
    json2[i] = "";                                                              // reset all json2 char's to zero (ie reset all protocols)
  }

  /*NOTES*/  // REINSTATE THIS ONCE WE HAVE NEW CALIBRATIONS
  //  call_print_calibration(0);                                                                  // recall all data saved in eeprom


  // read and process n+ commands from the serial port until we see the start of a json

  for (;;) {
    int c = Serial_Peek();

    powerdown();            // power down if no activity for x seconds (could also be a timer interrupt)

    if (c == -1)
      continue;             // nothing available, try again

    activity();             // record fact that we have seen activity (used with powerdown())

    if (c == '[')
      break;                // start of json, exit this for loop to process it

    // received a non '[' char - processs n+ command

    char choose[50];
    Serial_Input_Chars(choose, "+", 500, sizeof(choose) - 1);

    if (strlen(choose) < 3) {        // short or null command, quietly ignore it
      continue;
    }

    if (isprint(choose[0]) && !isdigit(choose[0])) {
      Serial_Print("commands must be numbers or json\n");
      continue;                     // go read another command
    }

    crc32_init();                   // clear CRC value since below may print a json

    switch (atoi(choose)) {
      case 440:
        // set real-time clock (needed if battery is removed)
        //setTime(hours, minutes, seconds, days, months, years); // TODO finish this
        break;
      case 1000:                                                                    // print "Ready" to USB and Bluetooth
        Serial_Print(DEVICE_NAME);
        Serial_Print_Line(" Ready");
        break;
      case 666:
        Serial_Print("wake up");
        // TODO dac1.update();
        break;
      case 667:
        Serial_Print("get status  ");
        // TODO Serial_Print_Line(dac1.getId());
        break;
      case 1001:
        Serial_Print_Line("PULSE1");
        DAC_set(1, 50);
        DAC_change();
        digitalWriteFast(PULSE1, HIGH);
        delay(1000);
        digitalWriteFast(PULSE1, LOW);
        DAC_set(1, 0);
        DAC_change();
        break;
      case 1002:
        Serial_Print_Line("PULSE2");
        DAC_set(2, 50);
        DAC_change();
        digitalWriteFast(PULSE2, HIGH);
        delay(1000);
        digitalWriteFast(PULSE2, LOW);
        DAC_set(2, 0);
        DAC_change();
        break;
      case 1003:
        Serial_Print_Line("PULSE3");
        DAC_set(3, 50);
        DAC_change();
        digitalWriteFast(PULSE3, HIGH);
        delay(1000);
        digitalWriteFast(PULSE3, LOW);
        DAC_set(3, 0);
        DAC_change();
        break;
      case 1004:
        Serial_Print_Line("PULSE4");
        DAC_set(4, 50);
        DAC_change();
        digitalWriteFast(PULSE4, HIGH);
        delay(1000);
        digitalWriteFast(PULSE4, LOW);
        DAC_set(4, 0);
        DAC_change();
        break;
      case 1005:
        Serial_Print_Line("PULSE5");
        DAC_set(5, 50);
        DAC_change();
        digitalWriteFast(PULSE5, HIGH);
        delay(1000);
        digitalWriteFast(PULSE5, LOW);
        DAC_set(5, 0);
        DAC_change();
        break;
      case 1006:
        Serial_Print_Line("PULSE6");
        DAC_set(6, 50);
        DAC_change();
        digitalWriteFast(PULSE6, HIGH);
        delay(1000);
        digitalWriteFast(PULSE6, LOW);
        DAC_set(6, 0);
        DAC_change();
        break;
      case 1007:
        Serial_Print("{\"device_name\":\"MultispeQ\"");
        Serial_Print(",\"device_id\":\"1\"");
        Serial_Print(",\"device_firmware\":\"1.0\"");
        Serial_Print(",\"device_manufacture\":\"03112016\"");
        Serial_Print("}");
        Serial_Print_CRC();
        break;
      /*
        case 1007:
        Serial_Print_Line("PULSE7");
        DAC_set(7, 50);
        DAC_change();
        digitalWriteFast(PULSE7, HIGH);
        delay(1000);
        digitalWriteFast(PULSE7, LOW);
        DAC_set(7, 0);
        DAC_change();
        break;
      */
      case 1008:
        Serial_Print_Line("PULSE8");
        DAC_set(8, 50);
        DAC_change();
        digitalWriteFast(PULSE8, HIGH);
        delay(1000);
        digitalWriteFast(PULSE8, LOW);
        DAC_set(8, 0);
        DAC_change();
        break;
      case 1009:
        Serial_Print_Line("PULSE9");
        DAC_set(9, 50);
        DAC_change();
        digitalWriteFast(PULSE9, HIGH);
        delay(1000);
        digitalWriteFast(PULSE9, LOW);
        DAC_set(9, 0);
        DAC_change();
        break;

      case 1010:
        Serial_Print_Line("PULSE10");
        DAC_set(10, 50);
        DAC_change();
        digitalWriteFast(PULSE10, HIGH);
        delay(1000);
        digitalWriteFast(PULSE10, LOW);
        DAC_set(10, 0);
        DAC_change();
        break;

      case 1011: {                                                                         // continuously output until user enter -1+
          Serial_Print("{\"light_intensity\":[");
          int leave = 0;
          int sensor_value;
          while (leave != -1) {
            leave = Serial_Input_Long("+", 2000);
            sensor_value = 10;
            //          sensor_value = MLX90614_Read(0);
            Serial_Print(sensor_value);
            Serial_Print("`,");
          }
          Serial_Print("0]}");
          Serial_Print_CRC();
        }
        break;

      case 1019:                                                                          // test the Serial_Input_Chars() command
        char S[10];
        Serial_Print_Line(eeprom->userdef[0], 4);
        Serial_Input_Chars(S, "+", 20000, sizeof(S));
        Serial_Printf("output is %s \n", S);
        eeprom->userdef[0] = atof(S);
        break;

      case 1020:                                                                          // test the Serial_Input_Float command
        Serial_Print_Line(eeprom->userdef[1], 4);
        eeprom->userdef[1] = Serial_Input_Double("+", 20000);
        Serial_Printf("output is %f \n", (float) eeprom->userdef[1]);
        // so measure the size of the string and if it's > 5000 then tell the user that the protocol is too long
        break;

      case 1027:
        _reboot_Teensyduino_();                                                    // restart teensy
        break;

      case 1021:                                                                          // Compare the new AD read method from AD7689
        AD7689_set(0);
        //        AD7689_sample();                                                                               // start conversion
        uint16_t middle_data3[100];
        AD7689_read_array(middle_data3, 100);
        for (int i = 0; i < 100; i++) {
          Serial_Print(middle_data3[i]);
          Serial_Print(",");
        }
        Serial_Print_Line("");
        AD7689_set(0);
        AD7689_sample();                                                                   // start conversion
        //        middle_data2 = AD7689_read_sample();                                    // read value (could subtract off baseline)
        //        Serial_Print_Line(middle_data2);
        break;

      case 1022:                                                                          // Set DAC addresses to 1,2,3 assuming addresses are unset and all are factory (0,0,0)
        DAC_set_address(LDAC1, 0, 1);
        DAC_set_address(LDAC2, 0, 2);
        DAC_set_address(LDAC3, 0, 3);
        break;

      case 1024:   {
          Serial_Print("Enter 1/2/3/4+\n");
          long setserial = Serial_Input_Long();
          Serial_Printf("set serial to %d\n", (int)setserial);
          Serial_Set((int) setserial);
          Serial_Print_Line("test print");
        }
        break;
      case 1025:  {                                                                         // Test to make sure that savings parameters to EEPROM works
          Serial_Print("enter BLE baud rate (9600, 19200, 38400,57600) followed by +");
          long baudrate = Serial_Input_Long();
          Serial_Begin((int) baudrate);
        }
        break;
      case 1026:                                                                         // Test to make sure that savings parameters to EEPROM works
        //        Serial_Print_Line("q2asdf asd fasdfasdf asdfas fa sfda sfa sdf asdf asdfasdf asdfasfasdfa sdfasdf45 asdfasdf as dfasdfa sdf45a sdfa dfa sdfa dfasdfasdf45asd asdfasfd asdfa fdasdfa sfd the end!");
        //       Serial_Print_Line("q2asdf asd fasdfasdf asdfas fa sfda sfa sdf asdf asdfasdf asdfasfasdfa sdfasdf45 asdfasdf as dfasdfa sdf45a sdfa dfa sdfa dfasdfasdf45asd asdfasfd asdfa fdasdfa sfd the end!");

        for (int i = 0; i < 10; i++) {
          Serial_Print_Line("1234567890123456789");
        }

        /*
                Serial_Print_Line("This is the first really long sentence!!!");
                delay(50);
                Serial_Print_Line("And the second long sentence is also long, blah blah blah???");
                delay(50);
                Serial_Print_Line("We finally made it to the end of something that we can actually read what a pain in the ass().");
                Serial_Print("new new new");
                Serial_Print("new new new");
                Serial_Print("new new new");
                Serial_Print("new new new");
        */
        break;
      case 1028:
        Serial_Printf("%d\n", eeprom->manufacture_date);
        Serial_Print_Line(eeprom->mag_bias[1]);
        Serial_Print_Line(eeprom->mag_cal[1][1]);
        Serial_Printf("%ld\n", eeprom->device_id);
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
            Serial_Printf("%d\n", (int)val[i]);
          }
          Serial_Printf("single pulse stdev = %.2f AD counts\n", stdev16(val, SAMPLES));
          Serial_Printf("time = %d usec for %d samples\n", delta_time, SAMPLES);
        }
        break;

      case 4045:
        set_device_info(1);  //  input device info and write to eeprom
        break;

      case 4047:
        {
          // JZ test - do not remove
          // read multiple pulses with increasing intensity or pulse width for linearity test
          // with constant DAC value and pulse width, it is good for a pulse-to-pulse stdev test
          const int LED = 5;                              // 1 = green, 2 = red, 3 = yellow, 5 = IR (keep DAC < 100)
          Serial_Print_Line("using delay - wait...");
          AD7689_set(0);                                  // 0 is main detector
          DAC_set(LED, 30);                               // set initial LED intensity
          DAC_change();
          const int MAX = 200;                            // try a variety of intensities 0 up to 4095
          int count = 0;
          uint16_t data[100];

          for (int i = 1; i < MAX; i += MAX / 100) {
            //DAC_set(LED, i);                              // change LED intensity
            //DAC_change();
            digitalWriteFast(HOLDM, HIGH);                  // discharge cap
            delay(33);                                     // also allows LED to cool and DC filter to adjust
            noInterrupts();
            digitalWriteFast(LED_to_pin[LED], HIGH);        // turn on LED
            delayMicroseconds(10);                          // allow slow actopulser to stabilize
            digitalWriteFast(HOLDM, LOW);                   // start integrating
            delayMicroseconds(40);                          // pulse width (depends on sensitivity needed)
            digitalWriteFast(LED_to_pin[LED], LOW);         // turn off LED
            const int SAMPLES = 21;                         // reduce noise with multiple reads
            uint16_t val[SAMPLES];
            AD7689_read_array(val, SAMPLES);                // read values
            interrupts();
            data[count] = median16(val, SAMPLES);
            if (data[count] >= 65535) break;                 // saturated the ADC, no point in continuing
            //Serial_Printf("%d,%d\n", i, data[count]);
            ++count;
          } // for
          // results from each pulse are in data[]
          Serial_Printf("pulse to pulse stdev = %.2f AD counts, first = %d\n\n", stdev16(data, count), data[0]);
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
          uint16_t _pulsesize = 30 + 10;                  // account for actopulser delay
          uint16_t _pulsedistance = 2000;                 // lower has less jitter

          startTimers(_pulsedistance, _pulsesize);        // schedule continuous LED pulses

          for (int i = 1; i < MAX; i += MAX / 100) {
            //DAC_set(LED, i);                              // change LED intensity
            //DAC_change();
            digitalWriteFast(HOLDM, HIGH);                  // discharge cap

            led_off = 0;

            while (led_off != 1) {}                  // wait till pulse is done

            // a pulse completed
            noInterrupts();

            const int SAMPLES = 19;                         // reduce noise with multiple reads
            uint16_t val[SAMPLES];
            AD7689_read_array(val, SAMPLES);                // read values
            interrupts();

            data[count] = median16(val, SAMPLES);
            if (data[count] >= 65535) break;                 // saturated the ADC, no point in continuing
            //Serial_Printf("%d,%d\n", i, data[count]);
            ++count;
          } // for

          stopTimers();

          // results from each pulse are in data[]
          Serial_Printf("pulse to pulse stdev = %.2f AD counts, first = %d\n\n", stdev16(data, count), data[0]);
        }
        break;

      case 4049:
        {
          // JZ test - do not remove
          // read multiple pulses with increasing intensity or pulse width for linearity test
          // with constant DAC value and pulse width, it is good for a pulse-to-pulse stdev test
          const int LED = 5;                              // 1 = green, 2 = red, 3 = yellow, 5 = IR (keep DAC < 100)
          Serial_Print_Line("using combined timer - wait...");
          AD7689_set(0);                                  // 0 is main detector
          DAC_set(LED, 200);                               // set initial LED intensity
          DAC_change();
          const int MAX = 200;                            // try a variety of intensities 0 up to 4095
          int count = 0;
          uint16_t data[100];

          _meas_light = LED;
          uint16_t _pulsesize = 30;
          uint16_t _pulsedistance = 2000;                 // lower has less jitter

          startTimers(_pulsedistance, _pulsesize);        // schedule continuous LED pulses

          for (int i = 1; i < MAX; i += MAX / 100) {
            //DAC_set(LED, i);                              // change LED intensity
            //DAC_change();
            digitalWriteFast(HOLDM, HIGH);                  // discharge cap
            
            led_off = 0;

            while (led_off != 1) {}                  // wait till pulse is done in ISR

            // a pulse completed (note: interrupts are left off)

            const int SAMPLES = 19;                         // reduce noise with multiple reads
            uint16_t val[SAMPLES];
            AD7689_read_array(val, SAMPLES);                // read values
            interrupts();
            
            data[count] = median16(val, SAMPLES);
            if (data[count] >= 65535) break;                 // saturated the ADC, no point in continuing
            //Serial_Printf("%d,%d\n", i, data[count]);
            ++count;
          } // for

          stopTimers();

          // results from each pulse are in data[]
          Serial_Printf("pulse to pulse stdev = %.2f AD counts, first = %d\n\n", stdev16(data, count), data[0]);
        }
        break;
      default:
        Serial_Printf("bad command # %s\n", choose);
        break;
    }  // switch()

    Serial_Flush_Output();     // force all output to go out

  } // for

  // done reading commands

  // read in a protocol (starts with '[', ends with '!' or timeout)

  // example: [{"pulses": [150],"a_lights": [[3]],"a_intensities": [[50]],"pulsedistance": 1000,"m_intensities": [[125]],"pulsesize": 2,"detectors": [[3]],"meas_lights": [[1]],"protocols": 1}]!

  Serial_Input_Chars(serial_buffer, "\r\n", 500, serial_buffer_size);

  if (!check_protocol(serial_buffer)) {         // sanity check
    Serial_Print("bad json protocol\n");
    return;
  }

  // break up the protocol into individual jsons

  int number_of_protocols = 0;                                   // number of protocols

  for (unsigned i = 1; i < strlen(serial_buffer); i++) {         // increments through each char in incoming transmission - if it's open curly, it saves all chars until closed curly.  Any other char is ignored.
    if (serial_buffer[i] == '{') {                               // wait until you see a open curly bracket
      while (serial_buffer[i] != '}') {                          // once you see it, save incoming data to json2 until you see closed curly bracket
        json2[number_of_protocols] += serial_buffer[i];          // add single char to json
        i++;
      }
      json2[number_of_protocols] += serial_buffer[i];           // catch the last closed curly
      number_of_protocols++;
    }
  }

  //  Serial_Printf("got %d protocols\n", number_of_protocols);

#ifdef DEBUGSIMPLE
  // print each json
  for (int i = 0; i < number_of_protocols; i++) {
    Serial_Printf("Incoming JSON %d as received by Teensy: %s", i, json2[i]);
  } // for
#endif

  // discharge sample and hold in case the cap is currently charged (on add on and main board)
  digitalWriteFast(HOLDM, HIGH);
  digitalWriteFast(HOLDADD, HIGH);
  delay(10);

  crc32_init();          // reset CRC

  Serial_Printf("{\"device_id\":%ld", eeprom->device_id);
  Serial_Printf(",\"firmware_version\":%s", FIRMWARE_VERSION);
  Serial_Print(",\"sample\":[");

  // loop through the all measurements to create a measurement group
  for (int y = 0; y < measurements; y++) {

    Serial_Print("[");                                                                        // print brackets to define single measurement

    for (int q = 0; q < number_of_protocols; q++) {                                           // loop through all of the protocols to create a measurement

      free(json);                                                                             // free initial json malloc, make sure this is here! Free before resetting the size according to the serial input
      json = (char*)malloc((json2[q].length() + 1) * sizeof(char));
      strncpy(json, json2[q].c_str(), json2[q].length());
      json[json2[q].length()] = '\0';                                                         // Add closing character to char*
      hashTable = root.parseHashTable(json);
      if (!hashTable.success()) {                                                             // NOTE: if the incomign JSON is too long (>~5000 bytes) this tends to be where you see failure (no response from device)
        Serial_Print("{\"error\":\"JSON not recognized, or other failure with Json Parser\"}, the data JSON we received is: ");
        for (int i = 0; i < serial_buffer_size; i++) {
          Serial_Print(json2[i].c_str());
        }
        Serial_Flush_Output();
        return;
      }

      int protocols = 1;
      int quit = 0;
      for (int u = 0; u < protocols; u++) {                                                    // the number of times to repeat the current protocol
        JsonArray save_eeprom    = hashTable.getArray("save");
        JsonArray recall_eeprom  = hashTable.getArray("recall");
        JsonArray number_samples = hashTable.getArray("number_samples");                       // number of samples on the cap during sample + hold phase (default is 40);
        JsonArray reference =     hashTable.getArray("reference");                              // subtract reference value if set to 1 (also causes 1/2 the sample rate per detector) due to time constraints.  Default is no reference (== 0)
        uint16_t adc_show =       hashTable.getLong("adc_show");                                // this tells the MultispeQ to print the ADC values only instead of the normal data_raw (for signal quality debugging)
        uint16_t adc_only[150];                                                   // this and first_adc_ref are used to save the first set of ADC averages produced, so it can be optionally displayed instead of data_raw (for signal quality debugging).  USE ONLY WITH REFERENCE == 0 (ie reference is OFF!)
        JsonArray pulses =        hashTable.getArray("pulses");                                // the number of measuring pulses, as an array.  For example [50,10,50] means 50 pulses, followed by 10 pulses, follwed by 50 pulses.
        String protocol_id =      hashTable.getString("protocol_id");                          // used to determine what macro to apply
        int analog_averages =     hashTable.getLong("analog_averages");                          // DEPRECIATED IN NEWEST HARDWARE 10/14 # of measurements per measurement pulse to be internally averaged (min 1 measurement per 6us pulselengthon) - LEAVE THIS AT 1 for now
        if (analog_averages == 0) {                                                              // if averages don't exist, set it to 1 automatically.
          analog_averages = 1;
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
#ifdef CORAL_SPEQ
        JsonArray delay_time =    hashTable.getArray("delay_time");                                         // delay per half clock (in microseconds).  This ultimately conrols the integration time.
        JsonArray read_time =     hashTable.getArray("read_time");                                        // Amount of time that the analogRead() procedure takes (in microseconds)
        JsonArray intTime =       hashTable.getArray("intTime");                                         // delay per half clock (in microseconds).  This ultimately conrols the integration time.
        JsonArray accumulateMode = hashTable.getArray("accumulateMode");
#endif

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
        } // for

        if (data_raw_average)
          free(data_raw_average);                                                            // free calloc of data_raw_average
        data_raw_average = (unsigned long*)calloc(size_of_data_raw, sizeof(unsigned long));   // get some memory space for data_raw_average, initialize all at zero.

#ifdef DEBUGSIMPLE
        Serial_Print_Line("");
        Serial_Print("size of data raw:  ");
        Serial_Print_Line(size_of_data_raw);

        Serial_Print_Line("");
        Serial_Print("total number of pulses:  ");
        Serial_Print_Line(total_pulses);

        Serial_Print_Line("");
        Serial_Print("all data in data_raw_average:  ");
        for (int i = 0; i < size_of_data_raw; i++) {
          Serial_Print(data_raw_average[i]);
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
#endif
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

        // this should be an array, so I can reset it all......
        analog_read_average = 0;
        digital_read_average = 0;
        relative_humidity_average = 0;                                                                    // reset all of the environmental variables
        temperature_average = 0;
        lux_average = 0;
        r_average = 0;
        g_average = 0;
        b_average = 0;
        lux_average_forpar = 0;
        r_average_forpar = 0;
        g_average_forpar = 0;
        b_average_forpar = 0;
        //!!! when offset gets recalculated I need to reposition this later, since pulsesize is now an array
        //        calculate_offset(pulsesize);                                                                    // calculate the offset, based on the pulsesize and the calibration values (ax+b)

#ifdef DEBUGSIMPLE
        Serial_Print_Line("");
        Serial_Print("\"offsets\": ");
        Serial_Print(offset_34);
        Serial_Print(",");
        Serial_Print_Line(offset_35);
        // null or invalid string returns zero

#endif

        // perform the protocol
        for (int x = 0; x < averages; x++) {                                                 // Repeat the protocol this many times

          if (Serial_Available() && Serial_Input_Long("+", 1) == -1) {
            q = number_of_protocols - 1;
            y = measurements - 1;
            u = protocols;
            x = averages;
          }

          int background_on = 0;
          long data_count = 0;
          int message_flag = 0;                                                              // flags to indicate if an alert, prompt, or confirm have been called at least once (to print the object name to data JSON)
          uint16_t _pulsedistance = 0;                                                  // initialize variables for pulsesize and pulsedistance (as well as the previous cycle's pulsesize and pulsedistance).  We define these only once per cycle so we're not constantly calling the JSON (which is slow)
          uint16_t _pulsedistance_prev = 0;
          uint16_t _pulsesize_prev = 0;
          uint16_t _reference_flag = 0;                                                           // used to note if this is the first measurement
          float _reference_start = 0;                                                            // reference value at data point 0 - initial value for normalizing the reference (normalized based on the values from main and reference in the first point in the trace)
          float _main_start = 0;                                                               // main detector (sample) value at data point 0 - initial value for normalizing the reference (normalized based on the values from main and reference in the first point in the trace)
          uint16_t _number_samples = 0;                                                               // create the adc sampling rate number
          lux_local = 0;                                                                     // reset local (this measurement) light levels
          r_local = 0;
          g_local = 0;
          b_local = 0;

          //      options for relative humidity, temperature, contactless temperature. light_intensity,co2
          //           0 - take before spectroscopy measurements
          //           1 - take after spectroscopy measurements

          for (int i = 0; i < environmental.getLength(); i++) {                                   // call environmental measurements
#ifdef DEBUGSIMPLE
            Serial_Print_Line("Here's the environmental measurements called:    ");
            Serial_Print(environmental.getArray(i).getString(0));
            Serial_Print(", ");
            Serial_Print_Line(environmental.getArray(i).getLong(1));
#endif
            /*
                        if (environmental.getArray(i).getLong(1) == 0 \
                        && (String) environmental.getArray(i).getString(0) == "relative_humidity") {
                          Relative_Humidity((int) environmental.getArray(i).getLong(1));                        // if this string is in the JSON and the 2nd component in the array is == 0 (meaning they want this measurement taken prior to the spectroscopic measurement), then call the associated measurement (and so on for all if statements in this for loop)
                          if (x == averages-1) {                                                                // if it's the last measurement to average, then print the results
                            Serial_Print("\"relative_humidity\":");
                            Serial_Print(relative_humidity_average,2);
                            Serial_Print(",");
                          }
                        }
                        if (environmental.getArray(i).getLong(1) == 0 \
                        && (String) environmental.getArray(i).getString(0) == "temperature") {
                          Temperature((int) environmental.getArray(i).getLong(1));
                          if (x == averages-1) {
                            Serial_Print("\"temperature\":");
                            Serial_Print(temperature_average,2);
                            Serial_Print(",");
                          }
                        }
                        if (environmental.getArray(i).getLong(1) == 0 \
                        && (String) environmental.getArray(i).getString(0) == "contactless_temperature") {
                          Contactless_Temperature( environmental.getArray(i).getLong(1));
                          float c_temp = MLX90614_Read(0);
                          if (x == averages-1) {
                            Serial_Print("\"contactless_temperature\":");
                            Serial_Print(c_temp,2);
                            Serial_Print(",");
                          }
                        }
                        if (environmental.getArray(i).getLong(1) == 0 \
                        && (String) environmental.getArray(i).getString(0) == "co2") {
                          Co2( environmental.getArray(i).getLong(1));
                          if (x == averages-1) {                                                                // if it's the last measurement to average, then print the results
                            Serial_Print("\"co2\":");
                            Serial_Print(co2_value_average,2);
                            Serial_Print(",");
                          }
                        }
            */
            if (environmental.getArray(i).getLong(1) == 0 \
                && (String) environmental.getArray(i).getString(0) == "light_intensity") {
              Light_Intensity(1);
              if (x == averages - 1) {
                Serial_Print("\"light_intensity\":");
                /*
                                Serial_Print(lux_to_uE(lux_average_forpar), 2);
                                Serial_Print(",");
                                Serial_Print("\"r\":");
                                Serial_Print(lux_to_uE(r_average_forpar), 2);
                                Serial_Print(",");
                                Serial_Print("\"g\":");
                                Serial_Print(lux_to_uE(g_average_forpar), 2);
                                Serial_Print(",");
                                Serial_Print("\"b\":");
                                Serial_Print(lux_to_uE(b_average_forpar), 2);
                                Serial_Print(",");
                */
              }
            }
            if (environmental.getArray(i).getLong(1) == 0 \
                && (String) environmental.getArray(i).getString(0) == "light_intensity_raw") {
              Light_Intensity(0);
              if (x == averages - 1) {
                Serial_Print("\"light_intensity_raw\":");
                Serial_Print(lux_average, 2);
                Serial_Print(",");
                Serial_Print("\"r\":");
                Serial_Print(r_average, 2);
                Serial_Print(",");
                Serial_Print("\"g\":");
                Serial_Print(g_average, 2);
                Serial_Print(",");
                Serial_Print("\"b\":");
                Serial_Print(b_average, 2);
                Serial_Print(",");
              }
            }
            if (environmental.getArray(i).getLong(1) == 0 \
                && (String) environmental.getArray(i).getString(0) == "analog_read") {                      // perform analog reads
              int pin = environmental.getArray(i).getLong(2);
              pinMode(pin, INPUT);
              int analog_read = analogRead(pin);
              if (x == averages - 1) {
                Serial_Print("\"analog_read\":");
                Serial_Print(analog_read);
                Serial_Print(",");
              }
            }
            if (environmental.getArray(i).getLong(1) == 0 \
                && (String) environmental.getArray(i).getString(0) == "digital_read") {                      // perform digital reads
              int pin = environmental.getArray(i).getLong(2);
              pinMode(pin, INPUT);
              int digital_read = digitalRead(pin);
              if (x == averages - 1) {
                Serial_Print("\"digital_read\":");
                Serial_Print(digital_read);
                Serial_Print(",");
              }
            }
            if (environmental.getArray(i).getLong(1) == 0 \
                && (String) environmental.getArray(i).getString(0) == "digital_write") {                      // perform digital write
              int pin = environmental.getArray(i).getLong(2);
              int setting = environmental.getArray(i).getLong(3);
              pinMode(pin, OUTPUT);
              digitalWriteFast(pin, setting);
            }
            if (environmental.getArray(i).getLong(1) == 0 \
                && (String) environmental.getArray(i).getString(0) == "analog_write") {                      // perform analog write with length of time to apply the pwm
              int pin = environmental.getArray(i).getLong(2);
              int setting = environmental.getArray(i).getLong(3);
              int freq = environmental.getArray(i).getLong(4);
              int wait = environmental.getArray(i).getLong(5);
#ifdef DEBUGSIMPLE
              Serial_Print_Line(pin);
              Serial_Print_Line(pin);
              Serial_Print_Line(wait);
              Serial_Print_Line(setting);
              Serial_Print_Line(freq);
#endif
              pinMode(pin, OUTPUT);
              analogWriteFrequency(pin, freq);                                                           // set analog frequency
              analogWrite(pin, setting);
              delay(wait);
              analogWrite(pin, 0);
              reset_freq();                                                                              // reset analog frequencies
            }
          }

          //&&
          //          analogReadAveraging(analog_averages);                                      // set analog averaging (ie ADC takes one signal per ~3u)

          //////////////////////ADC SETUP////////////////////////

          //int actfull = 0;
          //          int _tcs_to_act = 0;

          //          float _light_intensity = 0;

          //          float _light_intensity = lux_to_uE(lux_local);
          //          _tcs_to_act = (uE_to_intensity(act_background_light,_light_intensity)*tcs_to_act)/100;  // set intensity of actinic background light
#ifdef DEBUGSIMPLE
          Serial_Print_Line("");
          Serial_Print("tcs to act: ");
          Serial_Print_Line(_tcs_to_act);
          Serial_Print("ambient light in uE: ");
          Serial_Print_Line(lux_to_uE(lux_local));
          Serial_Print("ue to intensity result:  ");
          Serial_Print_Line(uE_to_intensity(act_background_light, lux_to_uE(lux_local))); // NOTE: wait turn flip DAC switch until later.
#endif

          for (int z = 0; z < total_pulses; z++) {                                      // cycle through all of the pulses from all cycles
            int first_flag = 0;                                                           // flag to note the first pulse of a cycle
            int _spec = 0;                                                              // create the spec flag for the coralspeq
#ifdef CORAL_SPEQ
            int _intTime = 0;                                                           // create the _intTime flag for the coralspeq
            int _delay_time = 0;                                                        // create the _delay_time flag for the coralspeq
            int _read_time = 0;                                                         // create the _read_time flag for the coralspeq
            int _accumulateMode = 0;                                                    // create the _accumulateMode flag for the coralspeq
#endif
            if (pulse == 0) {                                                                                     // if it's the first pulse of a cycle, we need to set up the new set of lights and intensities...
              meas_array_size = meas_lights.getArray(cycle).getLength();                                          // get the number of measurement/detector subsets in the new cycle
#ifdef PULSERDEBUG
              Serial_Printf("\n _number_samples: %d \n", _number_samples);
#endif

              for (unsigned i = 0; i < sizeof(_a_lights) / sizeof(int); i++) {                                  // save the list of act lights in the previous pulse set to turn off later
                _a_lights_prev[i] = _a_lights[i];
#ifdef PULSERDEBUG
                Serial_Printf("\n all a_lights_prev: %d\n", _a_lights_prev[i]);
#endif
              }
              for (unsigned i = 0; i < sizeof(_a_lights) / sizeof(int); i++) {                                   // save the current list of act lights, determine if they should be on, and determine their intensity
                _a_lights[i] = a_lights.getArray(cycle).getLong(i);
                _a_intensities[i] = a_intensities.getArray(cycle).getLong(i);
#ifdef PULSERDEBUG
                Serial_Printf("\n all a_lights, intensities: %d,%d\n", _a_lights[i], _a_intensities[i]);
#endif
              }

#ifdef CORAL_SPEQ
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
#endif
              if (cycle != 0) {
                _pulsedistance_prev = _pulsedistance;
                _pulsesize_prev = _pulsesize;
              }
              _pulsedistance = pulsedistance.getLong(cycle);                                                    // initialize variables for pulsesize and pulsedistance (as well as the previous cycle's pulsesize and pulsedistance).  We define these only once per cycle so we're not constantly calling the JSON (which is slow)
              _pulsesize = pulsesize.getLong(cycle);
              first_flag = 1;                                                                                   // flip flag indicating that it's the 0th pulse and a new cycle
              if (cycle == 0) {                                                                                 // if it's the beginning of a measurement (cycle == 0 and pulse == 0), then...
                digitalWriteFast(act_background_light_prev, LOW);                                               // turn off actinic background light and...
                startTimers(_pulsedistance, _pulsesize);                                                        // Use the two interrupt service routines timers (pulse1 and pulse2) in order to turn on (pulse1) and off (pulse2) the measuring lights.
              }
              else if (cycle != 0 && (_pulsedistance != _pulsedistance_prev || _pulsesize != _pulsesize_prev)) {    // if it's not the 0th cycle and the last pulsesize or pulsedistance was different than the current one, then stop the old timers and set new ones.   If they were the same, avoid resetting the timers by skipping this part.
                //              stopTimers();                                                                                   // stop the old timers
                startTimers(_pulsedistance, _pulsesize);                                    // restart the measurement light timer
              }
            }


#ifdef PULSERDEBUG
            Serial_Printf("pulsedistance = %d, pulsesize = %d, cycle = %d, measurement number = %d, measurement array size = %d,total pulses = %d\n", (int) _pulsedistance, (int) _pulsesize, (int) cycle, (int) meas_number, (int) meas_array_size, (int) total_pulses);
#endif
            _number_samples = number_samples.getLong(cycle);                                               // set the _number_samples for this cycle
            _meas_light = meas_lights.getArray(cycle).getLong(meas_number % meas_array_size);             // move to next measurement light
            uint16_t _m_intensity = m_intensities.getArray(cycle).getLong(meas_number % meas_array_size);  // move to next measurement light intensity
            uint16_t detector = detectors.getArray(cycle).getLong(meas_number % meas_array_size);          // move to next detector
            uint16_t _reference = reference.getArray(cycle).getLong(meas_number % meas_array_size);
            if (_number_samples == 0) {                                                                    // if _number_samples wasn't set or == 0, set it automatically to 40 (default)
              _number_samples = 11;
            }
            if (_reference != 0) {                                                                      // if using the reference detector, make sure to half the sample rate (1/2 sample from main, 1/2 sample from detector)
              _number_samples = _number_samples / 2;
            }
            if (_reference == 0) {                                                                      // If the reference detector isn't turned on, then we need to set the ADC first
              AD7689_set (detector - 1);        // set ADC channel to main board, main detector
            }
#ifdef PULSERDEBUG
            Serial_Printf("measurement light, intensity, detector, reference:  %d, %d, %d, %d\n", _meas_light, _m_intensity, detector, _reference);
#endif

            if (pulse < meas_array_size) {                                                                // if it's the first pulse of a cycle, then change act 1,2,3,4 values as per array's set at beginning of the file
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
                    startTimers(_pulsedistance, _pulsesize);                                                // restart the measurement light timer
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
                    startTimers(_pulsedistance, _pulsesize);                                                // restart the measurement light timer
                  }
                  else if (_message_type == "prompt") {                                                    // wait for user to input information, followed by +
                    stopTimers();                                                                         // pause the timers (so the measuring light doesn't stay on
                    char response[100];
                    Serial_Input_Chars(response, "+", 0, sizeof(response));
                    Serial_Print("\"");
                    Serial_Print(response);
                    Serial_Print("\"]");
                    startTimers(_pulsedistance, _pulsesize);                                                // restart the measurement light timer
                  }
                  if (cycle != pulses.getLength() - 1) {                                                  // if it's not the last cycle, then add comma
                    Serial_Print(",");
                  }
                  else {                                                                                 // if it is the last cycle, then close out the array
                    Serial_Print("],");
                  }
                }
              }
              //              calculate_intensity(_meas_light, tcs_to_act, cycle, _light_intensity);                   // in addition, calculate the intensity of the current measuring light

              DAC_set(_meas_light, _m_intensity);
              DAC_change();

              for (unsigned i = 0; i < sizeof(_a_lights) / sizeof(int); i++) {                         // set the DAC lights for actinic lights in the current pulse set
                DAC_set(_a_lights[i], _a_intensities[i]);
#ifdef PULSERDEBUG
                Serial_Printf("actinic pin : %d \nactinic intensity %d \n", _a_lights[i], _a_intensities[i]);
                Serial_Printf("length of _a_lights : %d \n ", sizeof(_a_lights));
                Serial_Printf("\n _number_samples, _reference, adc_show: %d %d %d\n", _number_samples, _reference, adc_show);
#endif
              }
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

            while (led_off == 0) {                                                                     // if the measuring light turned on and off (pulse1 and pulse2 are background interrupt routines for on and off) happened, then...
            }

            //            startTimer = micros();
            noInterrupts();                                                                            // turn off interrupts because we're checking volatile variables set in the interrupts

            if (_reference != 0) {
              AD7689_read_arrays((detector - 1), sample_adc, (_reference - 1), sample_adc_ref, _number_samples); // also reads reference detector - note this function takes detectors 0 - 3, so must subtract detector value by 1
            }
            else {
              AD7689_read_array(sample_adc, _number_samples);                                              // just reads the detector defined by the user
            }

            interrupts();                                                                                // turn off interrupts because we're checking volatile variables set in the interrupts
            //            endTimer = micros();
            digitalWriteFast(HOLDM, HIGH);                                                            // turn off sample and hold, and turn on lights for next pulse set
            digitalWriteFast(HOLDADD, HIGH);                                                            // turn off sample and hold, and turn on lights for next pulse set

#ifdef PULSERDEBUG
            Serial_Print("time:   "); Serial_Print_Line(endTimer - startTimer);
#endif
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
#ifdef PULSERDEBUG
              Serial_Printf("reference_start = %d, reference_now = %d,       _main_start = %d, main_now = %d", (int) _reference_start, (int) data_ref, (int) _main_start, (int) data);
#endif
              data = data - _main_start * (data_ref - _reference_start) / _reference_start; // adjust main value according to the % change in the reference relative to main on the first pulse.  You adjust the main in the opposite direction of the reference (to compensate)
#ifdef PULSERDEBUG
              float changed = (data_ref - _reference_start) / _reference_start;
              Serial_Printf(",      main = %d, ref = %d, data_normalized = %f, percent_change = %f\n", detector, _reference, data, changed);
#endif
            }

#ifdef PULSERDEBUG        // use this to see all of the adc reads which are being averaged
            Serial_Printf("median + first value :%d,%d", data, sample_adc[5]);
            Serial_Printf("median + first value reference :%d,%d", data_ref, sample_adc_ref[5]);
#endif

            if (first_flag == 1) {                                                                    // if this is the 0th pulse and a therefore new cycle
              for (unsigned i = 0; i < sizeof(_a_lights_prev) / sizeof(int); i++) {                            // Turn off all of the previous actinic lights
                if (_a_lights_prev[i] != 0) {                                                                 // just skip it if it's zero
                  digitalWriteFast(LED_to_pin[_a_lights_prev[i]], LOW);
#ifdef PULSERDEBUG
                  Serial_Printf("turned off actinic light: %d\n", LED_to_pin[_a_lights_prev[i]]);
#endif
                }
              }
              DAC_change();                                                                               // initiate actinic lights which were set above
              for (unsigned i = 0; i < sizeof(_a_lights) / sizeof(int); i++) {                            // Turn on all the new actinic lights for this pulse set
                if (_a_lights[i] != 0) {                                                                 // just skip it if it's zero
                  digitalWriteFast(LED_to_pin[_a_lights[i]], HIGH);
#ifdef PULSERDEBUG
                  Serial_Printf("turned on new actinic light: %d\n", LED_to_pin[_a_lights[i]]);
#endif
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

#ifdef DEBUGSIMPLE
            Serial_Print("data count, size of raw data                                   ");
            Serial_Print(data_count);
            Serial_Print(",");
            Serial_Print_Line(size_of_data_raw);

#endif
            if (_spec != 1) {                                                    // if spec_on is not equal to 1, then coralspeq is off and proceed as per normal MultispeQ measurement.
              if (_meas_light  != 0) {                                                      // save the data, so long as the measurement light is not equal to zero.
                data_raw_average[data_count] += data - _offset;
                data_count++;
              }
            }
#ifdef CORAL_SPEQ
            else if (_spec == 1) {                                              // if spec_on is 1 for this cycle, then collect data from the coralspeq and save it to data_raw_average.
              readSpectrometer(_intTime, _delay_time, _read_time, _accumulateMode);                                                        // collect a reading from the spec
              for (int i = 0 ; i < SPEC_CHANNELS; i++) {
                data_raw_average[data_count] += spec_data[i];
                data_count++;
              }
            }
#endif
            noInterrupts();                                                              // turn off interrupts because we're checking volatile variables set in the interrupts
            led_off = 0;                                                                // reset pulse status flags
            pulse++;                                                                     // progress the pulse counter and measurement number counter

#ifdef DEBUGSIMPLE
            Serial_Print("data point average, current data                               ");
            Serial_Print(data_raw_average[meas_number]);
            Serial_Print("!");
            Serial_Print_Line(data);
#endif
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

          for (unsigned i = 0; i < sizeof(_a_lights) / sizeof(int); i++) {
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

          for (int i = 0; i < environmental.getLength(); i++) {                                       // call environmental measurements after the spectroscopic measurement
#ifdef DEBUGSIMPLE
            Serial_Print_Line("Here's the environmental measurements called:    ");
            Serial_Print(environmental.getArray(i).getString(0));
            Serial_Print(", ");
            Serial_Print_Line(environmental.getArray(i).getLong(1));
#endif
            /*
                        if (environmental.getArray(i).getLong(1) == 1 \
                        && (String) environmental.getArray(i).getString(0) == "relative_humidity") {
                          Relative_Humidity((int) environmental.getArray(i).getLong(1));                        // if this string is in the JSON and the 3rd component in the array is == 1 (meaning they want this measurement taken prior to the spectroscopic measurement), then call the associated measurement (and so on for all if statements in this for loop)
                          if (x == averages-1) {                                                                // if it's the last measurement to average, then print the results
                            Serial_Print("\"relative_humidity\":");
                            Serial_Print(relative_humidity_average,2);
                            Serial_Print(",");
                          }
                        }
                        if (environmental.getArray(i).getLong(1) == 1 \
                      && (String) environmental.getArray(i).getString(0) == "temperature") {
                          Temperature((int) environmental.getArray(i).getLong(1));
                          if (x == averages-1) {
                            Serial_Print("\"temperature\":");
                            Serial_Print(temperature_average,2);
                            Serial_Print(",");
                          }
                        }
                        if (environmental.getArray(i).getLong(1) == 1 \
                        && (String) environmental.getArray(i).getString(0) == "contactless_temperature") {
                          Contactless_Temperature( environmental.getArray(i).getLong(1));
                          float c_temp = MLX90614_Read(0);
                          if (x == averages-1) {
                            Serial_Print("\"contactless_temperature\":");
                            Serial_Print(c_temp,2);
                            Serial_Print(",");
                          }
                        }
                        if (environmental.getArray(i).getLong(1) == 1 \
                      && (String) environmental.getArray(i).getString(0) == "co2") {
                          Co2( environmental.getArray(i).getLong(1));
                          if (x == averages-1) {
                            Serial_Print("\"co2\":");
                            Serial_Print(co2_value_average,2);
                            Serial_Print(",");
                          }
                        }
            */
            if (environmental.getArray(i).getLong(1) == 1 \
                && (String) environmental.getArray(i).getString(0) == "light_intensity") {
              Light_Intensity(1);
              if (x == averages - 1) {
                Serial_Print("\"light_intensity\":");
                /*
                  Serial_Print(lux_to_uE(lux_average_forpar), 2);
                  Serial_Print(",");
                  Serial_Print("\"r\":");
                  Serial_Print(lux_to_uE(r_average_forpar), 2);
                  Serial_Print(",");
                  Serial_Print("\"g\":");
                  Serial_Print(lux_to_uE(g_average_forpar), 2);
                  Serial_Print(",");
                  Serial_Print("\"b\":");
                  Serial_Print(lux_to_uE(b_average_forpar), 2);
                  Serial_Print(",");
                */
              }
            }
            if (environmental.getArray(i).getLong(1) == 1 \
                && (String) environmental.getArray(i).getString(0) == "light_intensity_raw") {
              Light_Intensity(0);
              if (x == averages - 1) {
                Serial_Print("\"light_intensity_raw\":");
                Serial_Print(lux_average, 2);
                Serial_Print(",");
                Serial_Print("\"r\":");
                Serial_Print(r_average, 2);
                Serial_Print(",");
                Serial_Print("\"g\":");
                Serial_Print(g_average, 2);
                Serial_Print(",");
                Serial_Print("\"b\":");
                Serial_Print(b_average, 2);
                Serial_Print(",");
              }
            }
            if (environmental.getArray(i).getLong(1) == 1 \
                && (String) environmental.getArray(i).getString(0) == "analog_read") {                      // perform analog reads
              int pin = environmental.getArray(i).getLong(2);
              pinMode(pin, INPUT);
              int analog_read = analogRead(pin);
              if (x == averages - 1) {
                Serial_Print("\"analog_read\":");
                Serial_Print(analog_read);
                Serial_Print(",");
              }
            }
            if (environmental.getArray(i).getLong(1) == 1 \
                && (String) environmental.getArray(i).getString(0) == "digital_read") {                      // perform digital reads
              int pin = environmental.getArray(i).getLong(2);
              pinMode(pin, INPUT);
              int digital_read = digitalRead(pin);
              if (x == averages - 1) {
                Serial_Print("\"digital_read\":");
                Serial_Print(digital_read);
                Serial_Print(",");
              }
            }
            if (environmental.getArray(i).getLong(1) == 1 \
                && (String) environmental.getArray(i).getString(0) == "digital_write") {                      // perform digital write
              int pin = environmental.getArray(i).getLong(2);
              int setting = environmental.getArray(i).getLong(3);
              pinMode(pin, OUTPUT);
              digitalWriteFast(pin, setting);
            }
            if (environmental.getArray(i).getLong(1) == 1 \
                && (String) environmental.getArray(i).getString(0) == "analog_write") {                      // perform analog write with length of time to apply the pwm
              int pin = environmental.getArray(i).getLong(2);
              int setting = environmental.getArray(i).getLong(3);
              int freq = environmental.getArray(i).getLong(4);
              int wait = environmental.getArray(i).getLong(5);
#ifdef DEBUGSIMPLE
              Serial_Print_Line(pin);
              Serial_Print_Line(pin);
              Serial_Print_Line(wait);
              Serial_Print_Line(setting);
              Serial_Print_Line(freq);
#endif
              pinMode(pin, OUTPUT);
              analogWriteFrequency(pin, freq);                                                           // set analog frequency
              analogWrite(pin, setting);
              delay(wait);
              analogWrite(pin, 0);
              reset_freq();                                                                              // reset analog frequencies
            }
          }
          if (x + 1 < averages) {                                                             //  to next average, unless it's the end of the very last run
            if (averages_delay > 0) {
              Serial_Input_Long("+", averages_delay * 1000);
            }
            if (averages_delay_ms > 0) {
              Serial_Input_Long("+", averages_delay_ms);
            }
          }
        }

        /*
           Recall and save values to the eeprom
        */

        recall_save(recall_eeprom, save_eeprom);                                                    // Recall and save values to the eeprom.  If you save values to eeprom, you get those saved values returned to you to confirm that they have been saved successfully (so save automatically calls recall once complete)

#ifdef CORAL_SPEQ
        if (spec_on == 1) {                                                                    // if the spec is being used, then read it and print data_raw as spec values.  Otherwise, print data_raw as multispeq detector values as per normal
          Serial_Print("\"data_raw\":[");
          for (int i = 0; i < SPEC_CHANNELS; i++)
          {
            Serial_Print((unsigned)(spec_data_average[i] / averages));
            if (i != SPEC_CHANNELS - 1) {                                                     // if it's the last one in printed array, don't print comma
              Serial_Print(",");
            }
          }
          Serial_Print("]}");
        }
#endif
        if (spec_on == 0) {
          Serial_Print("\"data_raw\":[");
          if (adc_show == 0) {                                                             // normal condition - show data_raw as per usual
            for (int i = 0; i < size_of_data_raw; i++) {                                     // print data_raw, divided by the number of averages (median would be nice)
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

#ifdef DEBUGSIMPLE
        Serial_Print("# of protocols repeats, current protocol repeat, number of total protocols, current protocol      ");
        Serial_Print(protocols);
        Serial_Print(",");
        Serial_Print(u);
        Serial_Print(",");
        Serial_Print(number_of_protocols);
        Serial_Print(",");
        Serial_Print_Line(q);
#endif

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
        analog_averages = 1;                                                             // # of measurements per pulse to be averaged (min 1 measurement per 6us pulselengthon)
        for (unsigned i = 0; i < sizeof(_a_lights) / sizeof(int); i++) {
          _a_lights[i] = 0;
        }
        relative_humidity_average = 0;                                                // reset all environmental variables to zero
        temperature_average = 0;
        objt_average = 0;
        lux_average = 0;
        r_average = 0;
        g_average = 0;
        b_average = 0;
        lux_average_forpar = 0;
        r_average_forpar = 0;
        g_average_forpar = 0;
        b_average_forpar = 0;
#ifdef CORAL_SPEQ
        for (int i = 0; i < SPEC_CHANNELS; i++) {
          spec_data_average [i] = 0;
        }
#endif
        act_background_light_prev = act_background_light;                               // set current background as previous background for next protocol
        spec_on = 0;                                                                    // reset flag that spec is turned on for this measurement
#ifdef DEBUGSIMPLE
        Serial_Print_Line("previous light set to:   ");
        Serial_Print_Line(act_background_light_prev);
#endif
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
    }
  }
  /*
    // make sure one last time that all of the lights are turned off, including background light!
    for (unsigned i = 0; i < sizeof(LED_to_pin) / sizeof(unsigned short); i++) {
      digitalWriteFast(LED_to_pin[i], LOW);
      Serial_Print_Line(LED_to_pin[i]);
    }
  */

  Serial_Print("]}");                // terminate json
  Serial_Print_CRC();

  act_background_light = 0;          // ??

  if (data_raw_average)
    free(data_raw_average);            // free the calloc() of data_raw_average
  free(json);                        // free second json malloc

} // loop()


//  routines for LED pulsing
#define STABILIZE 10                      // this delay gives the LED current controller op amp the time needed to stabilize

void pulse3() {                           // ISR to turn on/off LED pulse
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

void startTimers(uint16_t _pulsedistance, uint16_t _pulsesize) {
  timer0.begin(pulse3, _pulsedistance);             // schedule pulses
}

void stopTimers() {
  timer0.end();                         // if it's the last cycle and last pulse, then... stop the timers
  //timer1.end();                       // old version used two timers      
}

#if 0
// interrupt service routine which turns the measuring light on

void pulse1() {
#ifdef PULSERDEBUG
  startTimer = micros();
#endif
  digitalWriteFast(LED_to_pin[_meas_light], HIGH);            // turn on measuring light
  delayMicroseconds(10);             // this delay gives the LED current controller op amp the time needed to turn
  // the light on completely + stabilize.  But the effect is seen even after 50 usec.
  // Very low intensity measuring pulses may require an even longer delay here.
  digitalWriteFast(HOLDM, LOW);          // turn off sample and hold discharge
  digitalWriteFast(HOLDADD, LOW);        // turn off sample and hold discharge
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
// schedule the turn on and off of the LED(s) via an ISR

void startTimers(uint16_t _pulsedistance, uint16_t _pulsesize) {
  timer0.begin(pulse1, _pulsedistance);                                      // schedule on - not clear why this can't be done with interrupts off
  noInterrupts();
  delayMicroseconds(_pulsesize);                                             // I don't think this accounts for the actopulser stabilization delay - JZ
  interrupts();
  timer1.begin(pulse2, _pulsedistance);                                      // schedule off
}

#endif



// read/write userdef[] values from/to eeprom
// example json: [{"save":[[1,3.43],[2,5545]]}]  for userdef[1] = 3.43 and userdef[2] = 5545

void recall_save(JsonArray _recall_eeprom, JsonArray _save_eeprom) {
  int number_saves = _save_eeprom.getLength();                                 // define these explicitly to make it easier to understand the logic
  int number_recalls = _recall_eeprom.getLength();                             // define these explicitly to make it easier to understand the logic
  if (number_saves > 0) {                                                          // if the user is saving eeprom values, then...
    for (int i = 0; i < number_saves; i++) {
      int location = _save_eeprom.getArray(i).getLong(0);
      float value_to_save = _save_eeprom.getArray(i).getDouble(1);
      if (location >= 0 && location < NUM_USERDEFS)
        eeprom->userdef[location] = value_to_save;                                                //  save new value in the defined eeprom location
      delay(1);                                                                     // delay to make sure it has time to save (min 1ms)
    }
  }
  if (number_recalls > 0) {                  // if the user is recalling any saved eeprom values or if they just saved some, then...
    Serial_Print("\"recall\":{");                                                       // then print the eeprom location number and the value located there
    for (int i = 0; i < number_recalls; i++) {
      int location = _recall_eeprom.getLong(i);
      if (location >= 0 && location < NUM_USERDEFS)
        Serial_Printf("\"%d\":%f", (int) location, eeprom->userdef[location]);
      if (i != number_recalls - 1) {
        Serial_Print(",");
      }
      else {
        Serial_Print("},");
      }
    } // for
  } // recall_save()
}

