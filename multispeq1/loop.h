
//////////////////////// MAIN LOOP /////////////////////////

// process ascii serial input commands of two forms:
// 1010+<parameter1>+<parameter2>+...  (a command)
// [...] (a json protocol to be executed)


// function declarations
uint16_t median16(uint16_t array[], const int n, const float percentile = .50);
int check_protocol(char *str);
void startTimers(uint16_t _pulsedistance, uint16_t _pulsesize);
void stopTimers();


void loop() {

  delay(50);
  int measurements = 1;                                                   // the number of times to repeat the entire measurement (all protocols)
  unsigned long measurements_delay = 0; //  dac1.vdd(2.0475); // set VDD(mV) of MCP4728 for correct conversion between LSB and Vout                              // number of seconds to wait between measurements
  unsigned long measurements_delay_ms = 0;                                    // number of milliseconds to wait between measurements
  volatile unsigned long meas_number = 0;                                       // counter to cycle through measurement lights 1 - 4 during the run
  //unsigned long end1;
  //unsigned long start1 = millis();

  // these variables could be pulled from the JSON at the time of use... however, because pulling from JSON is slow, it's better to create a int to save them into at the beginning of a protocol run and use the int instead of the raw hashTable.getLong type call
  int _a_lights [10] = {};
  int _a_intensities [10] = {};
  int _a_lights_prev [10] = {};
  int act_background_light_prev = BLANK;
  int cycle = 0;                                                                // current cycle number (start counting at 0!)
  int pulse = 0;                                                                // current pulse number
  //int total_cycles;                                                           // Total number of cycles - note first cycle is cycle 0
  int meas_array_size = 0;                                                      // measures the number of measurement lights in the current cycle (example: for meas_lights = [[15,15,16],[15],[16,16,20]], the meas_array_size's are [3,1,3].
  char* json = (char*)malloc(1);
  //char w;
  //char* name;
  JsonHashTable hashTable;
  JsonParser<600> root;

  //int end_flag = 0;
  unsigned long* data_raw_average = (unsigned long*)malloc(4);
  char serial_buffer [serial_buffer_size];
  String json2 [max_jsons];
  memset(serial_buffer, 0, serial_buffer_size);                                  // reset buffer to zero
  for (int i = 0; i < max_jsons; i++) {
    json2[i] = "";                                                              // reset all json2 char's to zero (ie reset all protocols)
  }

  /*NOTES*/  // REINSTATE THIS ONCE WE HAVE NEW CALIBRATIONS
  //  call_print_calibration(0);                                                                  // recall all data saved in eeprom

  // discharge sample and hold in case the cap is currently charged (on add on and main board)
  digitalWriteFast(HOLDM, LOW);
  delay(10);
  digitalWriteFast(HOLDM, HIGH);
  digitalWriteFast(HOLDADD, LOW);
  delay(10);
  digitalWriteFast(HOLDADD, HIGH);

  // read and process n+ commands until we see the start of a json

  for (;;) {
    int c = Serial_Peek();

    if (c == -1)
      continue;              // nothing available, try again

    if (c == '[')
      break;              // start of json, exit this for loop to process it

    // received a non '[' char - processs n+ command

    char choose[50];
    Serial_Input_Chars(choose, "+", 500, sizeof(choose) - 1);

    if (strlen(choose) < 3) {        // short or null command, quietly ignore it
      continue;
    }

    if (!isdigit(choose[0])) {
      //      Serial_Print("commands must be numbers\n");
      continue;                     // go read another command
    }

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
            leave = Serial_Input_Float("+", 2000);
            sensor_value = 10;
            //          sensor_value = MLX90614_Read(0);
            Serial_Print(sensor_value);
            Serial_Print("`,");
          }
          Serial_Print("0]}");
          Serial_Print_CRC();
        }
        break;
      case 1018:                                                                          // test the Serial_Input_Chars() command
        char S[10];
        Serial_Print_Line(userdef1[0], 4);
        Serial_Input_Chars(S, "+", 20000, sizeof(S));
        Serial_Printf("output is %s \n", S);
        userdef1[0] = atof(S);
        break;
      case 1019:                                                                          // test the Serial_Input_Float command
        Serial_Print_Line(userdef1[0], 4);
        userdef1[0] = Serial_Input_Float("+", 20000);
        Serial_Printf("output is %f \n", userdef1[0]);
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

      case 4044:
        {
          // JZ test - do not remove
          // read and analyze noise on ADC from a single LED pulse
          const int LED = 5;                              // 1 = green, 2 = red
          const int SAMPLES = 100;
          uint16_t val[SAMPLES];
          Serial_Print_Line("JZ test");
          DAC_set(LED, 40);                               // set LED intensity
          DAC_change();
          AD7689_set(0);                                  // select ADC channel
          digitalWriteFast(HOLDM, HIGH);                  // discharge cap
          delay(1000);
          noInterrupts();
          digitalWriteFast(LED_to_pin[LED], HIGH);        // turn on LED
          delayMicroseconds(30);                          // allow slow actopulser to stabilize
          digitalWriteFast(HOLDM, LOW);                   // start integrating (could take baseline value here)
          delayMicroseconds(20);                          // measuring width
          digitalWriteFast(LED_to_pin[LED], LOW);         // turn off LED
          uint32_t delta_time = micros();
          AD7689_read_array(val, SAMPLES);                // read values
          delta_time = micros() - delta_time;
          interrupts();
          // calc stats
          double mean = 0, delta = 0, m2 = 0, variance = 0, stdev = 0, n = 0;
          for (int i = 0; i < SAMPLES; i++) {
            val[i] += i;                     // EXPERIMENTAL - adjust for droop
            Serial_Printf("%d\n", val[i]);
            ++n;
            delta = val[i] - mean;
            mean += delta / n;
            m2 += (delta * (val[i] - mean));
          } // for
          variance = m2 / (SAMPLES - 1);  // (n-1):Sample Variance  (n): Population Variance
          stdev = sqrt(variance);         // Calculate standard deviation
          Serial_Printf("single pulse stdev = %.2f, mean = %.2f AD counts\n", stdev, mean);
          Serial_Printf("bits (95%%) = %.2f\n", (15 - log(stdev * 2) / log(2.0))); // 2 std dev from mean = 95%
          Serial_Printf("time = %d usec for %d samples\n", delta_time, SAMPLES);
        }
        break;

      case 4045:
        set_device_info(1);  // works
        break;

      case 4047:
        {
          // JZ test - do not remove
          // read multiple pulses with increasing intensity for linearity test
          const int LED = 3;                              // 1 = green, 2 = red, 3 = yellow, 5 = IR
          Serial_Print_Line("DAC,intensity);
          AD7689_set(0);
          const int MAX = 4095;                            // try a variety of intensities 0 up to 4095
          for (int i = 1; i < MAX; i += MAX / 100) {
            DAC_set(LED, i);                               // set LED intensity
            DAC_change();
            digitalWriteFast(HOLDM, HIGH);                  // discharge cap
            delay(100);
            noInterrupts();
            digitalWriteFast(LED_to_pin[LED], HIGH);        // turn on LED
            delayMicroseconds(30);                          // allow slow actopulser to stabilize
            digitalWriteFast(HOLDM, LOW);                   // start integrating
            delayMicroseconds(100);                          // pulse width (depends on sensitivity needed)
            digitalWriteFast(LED_to_pin[LED], LOW);         // turn off LED
            const int SAMPLES = 11;                         // reduce noise
            uint16_t val[SAMPLES];
            AD7689_read_array(val, SAMPLES);                // read values
            interrupts();
            int data = median16(val, SAMPLES);
            if (data > 65400) break;                         // saturated the ADC, no point in continuing
            Serial_Printf("%d,%d\n", i, data);               // AD offset was 698 for LED 5
          }
          Serial_Print_Line("done");
        }
        break;

      default:
        Serial_Printf("bad command # %s\n", choose);
        break;
    }  // switch()

    Serial_Flush_Output();     // force all output to go out
    crc32_init();              // reset CRC in case anything above printed

  } // for

  // done reading commands

  // read in a protocol (starts with '[', ends with '!')

  // example: [{"pulses": [150],"a_lights": [[3]],"a_intensities": [[50]],"pulsedistance": 1000,"m_intensities": [[125]],"pulsesize": 2,"detectors": [[3]],"meas_lights": [[1]],"protocols": 1}]!

  Serial_Input_Chars(serial_buffer, "\r\n", 500, serial_buffer_size);

  /*
    Serial_Print("got protocol of ");
    Serial_Print_Line(serial_buffer);
  */

  if (!check_protocol(serial_buffer)) {         // sanity check
    Serial_Print("bad protocol\n");
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

  crc32_init();          // reset CRC

  Serial_Print("{\"device_id\": ");
  Serial_Print(device_id, 0);
  Serial_Print(",\"firmware_version\":");
  Serial_Print((String) FIRMWARE_VERSION);
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
        JsonArray number_samples =   hashTable.getArray("number_samples");                       // number of samples on the cap during sample + hold phase (default is 40);
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
          act_background_light =  13;                                                            // change to new background actinic light
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
        int act_background_light_intensity = hashTable.getLong("act_background_light_intensity");  // sets intensity of background actinic.  Choose this OR tcs_to_act.
        int tcs_to_act =            hashTable.getLong("tcs_to_act");                               // sets the % of response from the tcs light sensor to act as actinic during the run (values 1 - 100).  If tcs_to_act is not defined (ie == 0), then the act_background_light intensity is set to actintensity1.
        //int offset_off =          hashTable.getLong("offset_off");                               // turn off detector offsets (default == 0 which is on, set == 1 to turn offsets off)

        ///*
//        long pulsedistance =      hashTable.getLong("pulsedistance");                            // distance between measuring pulses in us.  Minimum 1000 us.
        JsonArray pulsedistance =   hashTable.getArray("pulsedistance");                            // distance between measuring pulses in us.  Minimum 1000 us.
//        long pulsesize =          hashTable.getLong("pulsesize");                                // Size of the measuring pulse (5 - 100us).  This also acts as gain control setting - shorter pulse, small signal. Longer pulse, larger signal.
        JsonArray pulsesize =       hashTable.getArray("pulsesize");                            // distance between measuring pulses in us.  Minimum 1000 us.

        JsonArray a_lights =        hashTable.getArray("a_lights");
        JsonArray a_intensities =   hashTable.getArray("a_intensities");
        JsonArray m_intensities =   hashTable.getArray("m_intensities");

        int get_offset =          hashTable.getLong("get_offset");                               // include detector offset information in the output
        // NOTE: it takes about 50us to set a DAC channel via I2C at 2.4Mz.

#if 0
        JsonArray get_ir_baseline = hashTable.getArray("get_ir_baseline");                      // include the ir_baseline information from the device for the specified pins
        JsonArray get_tcs_cal =   hashTable.getArray("get_tcs_cal");                            // include the get_tcs_cal information from the device for the specified pins
        JsonArray get_lights_cal = hashTable.getArray("get_lights_cal");                        // include get_lights_cal information from the device for the specified pins
        JsonArray get_blank_cal = hashTable.getArray("get_blank_cal");                          // include the get_blank_cal information from the device for the specified pins
        JsonArray get_other_cal = hashTable.getArray("get_other_cal");                        // include the get_other_cal information from the device for the specified pins

        int get_userdef0 =  hashTable.getLong("get_userdef0");                        // include the saved userdef0 information from the device
        int get_userdef1 =  hashTable.getLong("get_userdef1");                        // include the saved userdef1 information from the device
        int get_userdef2 =  hashTable.getLong("get_userdef2");                        // include the saved userdef2 information from the device
        int get_userdef3 =  hashTable.getLong("get_userdef3");                        // include the saved userdef3 information from the device
        int get_userdef4 =  hashTable.getLong("get_userdef4");                        // include the saved userdef4 information from the device
        int get_userdef5 =  hashTable.getLong("get_userdef5");                        // include the saved userdef5 information from the device
        int get_userdef6 =  hashTable.getLong("get_userdef6");                        // include the saved userdef6 information from the device
        int get_userdef7 =  hashTable.getLong("get_userdef7");                        // include the saved userdef7 information from the device
        int get_userdef8 =  hashTable.getLong("get_userdef8");                        // include the saved userdef8 information from the device
        int get_userdef9 =  hashTable.getLong("get_userdef9");                        // include the saved userdef9 information from the device
        int get_userdef10 =  hashTable.getLong("get_userdef10");                        // include the saved userdef10 information from the device
        int get_userdef11 =  hashTable.getLong("get_userdef11");                        // include the saved userdef11 information from the device
        int get_userdef12 =  hashTable.getLong("get_userdef12");                        // include the saved userdef12 information from the device
        int get_userdef13 =  hashTable.getLong("get_userdef13");                        // include the saved userdef13 information from the device
        int get_userdef14 =  hashTable.getLong("get_userdef14");                        // include the saved userdef14 information from the device
        int get_userdef15 =  hashTable.getLong("get_userdef15");                        // include the saved userdef15 information from the device
        int get_userdef16 =  hashTable.getLong("get_userdef16");                        // include the saved userdef16 information from the device
        int get_userdef17 =  hashTable.getLong("get_userdef17");                        // include the saved userdef17 information from the device
        int get_userdef18 =  hashTable.getLong("get_userdef18");                        // include the saved userdef18 information from the device
        int get_userdef19 =  hashTable.getLong("get_userdef19");                        // include the saved userdef19 information from the device
        int get_userdef20 =  hashTable.getLong("get_userdef20");                        // include the saved userdef20 information from the device
        int get_userdef21 =  hashTable.getLong("get_userdef21");                        // include the saved userdef21 information from the device
        int get_userdef22 =  hashTable.getLong("get_userdef22");                        // include the saved userdef22 information from the device
        int get_userdef23 =  hashTable.getLong("get_userdef23");                        // include the saved userdef23 information from the device
        int get_userdef24 =  hashTable.getLong("get_userdef24");                        // include the saved userdef24 information from the device
        int get_userdef25 =  hashTable.getLong("get_userdef25");                        // include the saved userdef25 information from the device
        int get_userdef26 =  hashTable.getLong("get_userdef26");                        // include the saved userdef26 information from the device
        int get_userdef27 =  hashTable.getLong("get_userdef27");                        // include the saved userdef27 information from the device
        int get_userdef28 =  hashTable.getLong("get_userdef28");                        // include the saved userdef28 information from the device
        int get_userdef29 =  hashTable.getLong("get_userdef29");                        // include the saved userdef29 information from the device
        int get_userdef30 =  hashTable.getLong("get_userdef30");                        // include the saved userdef30 information from the device
        int get_userdef31 =  hashTable.getLong("get_userdef31");                        // include the saved userdef31 information from the device
        int get_userdef32 =  hashTable.getLong("get_userdef32");                        // include the saved userdef32 information from the device
        int get_userdef33 =  hashTable.getLong("get_userdef33");                        // include the saved userdef33 information from the device
        int get_userdef34 =  hashTable.getLong("get_userdef34");                        // include the saved userdef34 information from the device
        int get_userdef35 =  hashTable.getLong("get_userdef35");                        // include the saved userdef35 information from the device
        int get_userdef36 =  hashTable.getLong("get_userdef36");                        // include the saved userdef36 information from the device
        int get_userdef37 =  hashTable.getLong("get_userdef37");                        // include the saved userdef37 information from the device
        int get_userdef38 =  hashTable.getLong("get_userdef38");                        // include the saved userdef38 information from the device
        int get_userdef39 =  hashTable.getLong("get_userdef39");                        // include the saved userdef39 information from the device
        int get_userdef40 =  hashTable.getLong("get_userdef40");                        // include the saved userdef40 information from the device
        int get_userdef41 =  hashTable.getLong("get_userdef41");                        // include the saved userdef41 information from the device
        int get_userdef42 =  hashTable.getLong("get_userdef42");                        // include the saved userdef42 information from the device
        int get_userdef43 =  hashTable.getLong("get_userdef43");                        // include the saved userdef43 information from the device
        int get_userdef44 =  hashTable.getLong("get_userdef44");                        // include the saved userdef44 information from the device
        int get_userdef45 =  hashTable.getLong("get_userdef45");                        // include the saved userdef45 information from the device
        int get_userdef46 =  hashTable.getLong("get_userdef46");                        // include the saved userdef46 information from the device
        int get_userdef47 =  hashTable.getLong("get_userdef47");                        // include the saved userdef47 information from the device
        int get_userdef48 =  hashTable.getLong("get_userdef48");                        // include the saved userdef48 information from the device
        int get_userdef49 =  hashTable.getLong("get_userdef49");                        // include the saved userdef49 information from the device
        int get_userdef50 =  hashTable.getLong("get_userdef50");                        // include the saved userdef50 information from the device
#endif

        ///*
        //JsonArray act1_lights =   hashTable.getArray("act1_lights");
        //JsonArray act2_lights =   hashTable.getArray("act2_lights");
        //JsonArray act3_lights =   hashTable.getArray("act3_lights");
        //JsonArray act4_lights =   hashTable.getArray("act4_lights");
        act_intensities =         hashTable.getArray("act_intensities");                         // write to input register of a dac1. channel 0 for low (actinic).  1 step = +3.69uE (271 == 1000uE, 135 == 500uE, 27 == 100uE)
        meas_intensities =        hashTable.getArray("meas_intensities");                        // write to input register of a dac1. channel 3 measuring light.  0 (high) - 4095 (low).  2092 = 0.  From 2092 to zero, 1 step = +.2611uE
        cal_intensities =         hashTable.getArray("cal_intensities");                         // write to input register of a dac1. channel 2 calibrating light.  0 (low) - 4095 (high).
        JsonArray detectors =     hashTable.getArray("detectors");                               // the Teensy pin # of the detectors used during those pulses, as an array of array.  For example, if pulses = [5,2] and detectors = [[34,35],[34,35]] .
        JsonArray meas_lights =   hashTable.getArray("meas_lights");
        JsonArray message =       hashTable.getArray("message");                                // sends the user a message to which they must reply <answer>+ to continue
        //*/
        JsonArray environmental = hashTable.getArray("environmental");

        // ********************INPUT DATA FOR CORALSPEQ*******************
        JsonArray spec =          hashTable.getArray("spec");                                // defines whether the spec will be called during each array.  note for each single plus, the spec will call and add 256 values to data_raw!
        JsonArray delay_time =    hashTable.getArray("delay_time");                                         // delay per half clock (in microseconds).  This ultimately conrols the integration time.
        JsonArray read_time =     hashTable.getArray("read_time");                                        // Amount of time that the analogRead() procedure takes (in microseconds)
        JsonArray intTime =       hashTable.getArray("intTime");                                         // delay per half clock (in microseconds).  This ultimately conrols the integration time.
        JsonArray accumulateMode = hashTable.getArray("accumulateMode");
        //total_cycles =            pulses.getLength() - 1;                                        // (start counting at 0!)

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
        }
        free(data_raw_average);                                                            // free malloc of data_raw_average
        //        data_raw_average = (long*)calloc(size_of_data_raw,sizeof(long));                   // get some memory space for data_raw_average, initialize all at zero.
        data_raw_average = (unsigned long*)calloc(size_of_data_raw, sizeof(unsigned long));                  // get some memory space for data_raw_average, initialize all at zero.

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

        if (get_offset == 1) {
          print_offset(1);
        }
        /*
                print_get_userdef0(get_userdef0,get_userdef1,get_userdef2,get_userdef3,get_userdef4,get_userdef5,get_userdef6,get_userdef7,get_userdef8,get_userdef9,get_userdef10,get_userdef11,get_userdef12,get_userdef13,get_userdef14,get_userdef15,get_userdef16,get_userdef17,get_userdef18,get_userdef19,get_userdef20,get_userdef21,get_userdef22,get_userdef23,get_userdef24,get_userdef25,get_userdef26,get_userdef27,get_userdef28,get_userdef29,get_userdef30,get_userdef31,get_userdef32,get_userdef33,get_userdef34,get_userdef35,get_userdef36,get_userdef37,get_userdef38,get_userdef39,get_userdef40,get_userdef41,get_userdef42,get_userdef43,get_userdef44,get_userdef45,get_userdef46,get_userdef47,get_userdef48,get_userdef49,get_userdef50); // check to see if we need to print any of the user defined calibrations

                get_calibration(calibration_baseline_slope,calibration_baseline_yint,0,0,get_ir_baseline ,"get_ir_baseline");
                get_calibration(calibration_slope,calibration_yint,0,0,get_lights_cal ,"get_lights_cal");
                get_calibration(calibration_blank1,calibration_blank2,0,0,get_blank_cal,"get_blank_cal");
                get_calibration(calibration_other1,calibration_other2,0,0,get_other_cal,"get_other_cal");
                get_calibration(0,0,light_slope,light_y_intercept,get_tcs_cal,"get_tcs_cal");
          //        get_calibration_userdef();
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

          if (Serial_Available() && Serial_Input_Float("+", 1) == -1) {
            q = number_of_protocols - 1;
            y = measurements - 1;
            u = protocols;
            x = averages;
          }

          int background_on = 0;
          long data_count = 0;
          int message_flag = 0;                                                              // flags to indicate if an alert, prompt, or confirm have been called at least once (to print the object name to data JSON)
          uint16_t _pulsedistance;                                                    // initialize variables for pulsesize and pulsedistance (as well as the previous cycle's pulsesize and pulsedistance).  We define these only once per cycle so we're not constantly calling the JSON (which is slow)
          uint16_t _pulsesize;
          uint16_t _pulsedistance_prev;
          uint16_t _pulsesize_prev;
          uint16_t _reference = 0;                                                                 // create the reference flag
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
          float _light_intensity = lux_to_uE(lux_local);
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
            int _intTime = 0;                                                           // create the _intTime flag for the coralspeq
            int _delay_time = 0;                                                        // create the _delay_time flag for the coralspeq
            int _read_time = 0;                                                         // create the _read_time flag for the coralspeq
            int _accumulateMode = 0;                                                    // create the _accumulateMode flag for the coralspeq            

            if (pulse == 0) {                                                                                     // if it's the first pulse of a cycle, we need to set up the new set of lights and intensities...
              meas_array_size = meas_lights.getArray(cycle).getLength();                                          // get the number of measurement/detector subsets in the new cycle
#ifdef PULSERDEBUG
              Serial_Printf("\n _number_samples and _reference: %d %d \n", _number_samples, _reference);
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
            Serial_Printf("pulsedistance = %d, pulsesize = %d, cycle = %d, measurement number = %d, measurement array size = %d,total pulses = %d\n",(int) _pulsedistance, (int) _pulsesize,(int) cycle,(int) meas_number,(int) meas_array_size,(int) total_pulses);
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
            Serial_Printf("measurement light, intensity, detector, reference:  %d, %d, %d, %d\n", _meas_light, _m_intensity, detector, reference);
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
                      long response = Serial_Input_Long();
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
                      long response = Serial_Input_Long();
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
              calculate_intensity(_meas_light, tcs_to_act, cycle, _light_intensity);                   // in addition, calculate the intensity of the current measuring light

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

            if (Serial_Available() && Serial_Input_Float("+", 1) == -1) {                                      // exit protocol completely if user enters -1+
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






            while (on == 0 || off == 0) {                                                                     // if the measuring light turned on and off (pulse1 and pulse2 are background interrupt routines for on and off) happened, then...
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
            on = 0;                                                                      // reset pulse counters
            off = 0;
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
          background_on = calculate_intensity_background(act_background_light, tcs_to_act, cycle, _light_intensity, act_background_light_intensity); // figure out background light intensity and state

          for (unsigned i = 0; i < sizeof(_a_lights) / sizeof(int); i++) {
            if (_a_lights[i] != act_background_light) {                                  // turn off all lights unless they are the actinic background light
              digitalWriteFast(LED_to_pin[_a_lights[i]], LOW);
            }
          }



          /*
                    if (_act1_light != act_background_light) {                                  // turn off all lights unless they are the actinic background light
                      digitalWriteFast(_act1_light, LOW);
                    }
                    if (_act2_light != act_background_light) {
                      digitalWriteFast(_act2_light, LOW);
                    }
                    if (_act3_light != act_background_light) {
                      digitalWriteFast(_act3_light, LOW);
                    }
                    if (_act4_light != act_background_light) {
                      digitalWriteFast(_act4_light, LOW);
                    }
          */

          if (background_on == 1) {
            digitalWriteFast(LDAC1, LOW);
            delayMicroseconds(1);
            digitalWriteFast(LDAC1, HIGH);
            digitalWriteFast(act_background_light, HIGH);                                // turn on actinic background light in case it was off previously.
          }
          else {
            digitalWriteFast(act_background_light, LOW);                                // turn on actinic background light in case it was off previously.
          }

          timer0.end();                                                                  // if it's the last cycle and last pulse, then... stop the timers
          timer1.end();
          cycle = 0;                                                                     // ...and reset counters
          pulse = 0;
          on = 0;
          off = 0;
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


        // skip to the end of the protocol
skipPart:

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

  Serial_Print("]}");
  act_background_light = 13;                                                      // reset background light to teensy pin 13
  free(data_raw_average);                                                         // free the calloc() of data_raw_average
  free(json);                                                                     // free second json malloc
  Serial_Print_CRC();
} // loop()

// check a protocol for validity (matching [] and {})
// return 1 if OK, otherwise 0
// should also check CRC value if present

int check_protocol(char *str)
{
  int bracket = 0, curly = 0;
  char *start = str;

  while (*str != 0) {
    switch (*str) {
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
    ++str;
  } // while

  if (bracket != 0 || curly != 0)  // unbalanced - can't be correct
    return 0;

  // check CRC - 8 hex digits immediately after the closing }
  char *ptr = strrchr(start, '}'); // find last }
  if (!ptr)                        // no } found - how can that be?
    return 0;

  if (!isxdigit(*(ptr + 1)))      // hex digit follows last }
    return 1;                    // no CRC so report OK

  // CRC is there - check it

  crc32_init();     // find crc of json (from first { to last })
  crc32_buf (start, 1 + ptr - start);

  // note: must be exactly 8 upper case hex digits
  if (strncmp(int32_to_hex (crc32_value()), ptr + 1, 8) != 0) {
    return 0;                 // bad CRC
  }

  return 1;
} // check_protocol()

// schedule the turn on and off of the LED(s) via an ISR
void startTimers(uint16_t _pulsedistance, uint16_t _pulsesize) {
  timer0.begin(pulse1, _pulsedistance);                                      // schedule on - not clear why this can't be done with interrupts off
  noInterrupts();
  delayMicroseconds(_pulsesize);                                             // I don't this accounts for the actopulser stabilization delay - JZ
  interrupts();
  timer1.begin(pulse2, _pulsedistance);                                      // schedule off
}
void stopTimers() {
  timer0.end();                                                                  // if it's the last cycle and last pulse, then... stop the timers
  timer1.end();
}


