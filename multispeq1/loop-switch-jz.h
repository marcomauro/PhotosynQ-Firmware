
// If your name isn't Jon, don't touch this file

{

case hash("set_date"):
  {
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
case hash("print_date"):
  // example: 2004-02-12T15:19:21.000Z
  if (year() >= 2016) {
    Serial_Printf("{\"device_time\":\"%d-%d-%dT%d:%d:%d.000Z\"}\n", year(), month(), day(), hour(), minute(), second());
    Serial_Printf("{\"device_time\":%u}\n", now()); // since 1970 format
  }
  break;

case hash("powerdown"):
  pinMode(POWERDOWN_REQUEST, OUTPUT);     //  bring P0.6 (2nd pin) low
  digitalWrite(POWERDOWN_REQUEST, LOW);
  delay(11000);                  // device should power off here - P0.5 (third pin) should go low
  digitalWrite(POWERDOWN_REQUEST, HIGH); // put it back
  break;

case hash("battery"):
  battery_low(1);  // test battery with LEDs on
  break;

case hash("scan_i2c"):
  scan_i2c();
  break;

case hash("sleep"):
  sleep_mode(5000);
  Serial_Print_Line("done sleeping");
  break;

case hash("packet_test"):
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

case hash("compiled"):
  Serial_Printf("Compiled on: %s %s\n", __DATE__, __TIME__);
  break;

case hash("temp"):
  Serial_Printf("BME2801 Temp = %fC, Humidity = %fC\n", bme1.readTempC(), bme1.readHumidity());
  Serial_Printf("BME2802 Temp = %fC, Humidity = %fC\n", bme2.readTempC(), bme2.readHumidity());
  break;

case hash("memory"):
  {
    uint32_t stackTop;
    uint32_t heapTop;

    // current position of the stack.
    stackTop = (uint32_t) & stackTop;

    // current position of heap.
    void* hTop = malloc(1);
    heapTop = (uint32_t) hTop;
    free(hTop);

    // The difference is the free, available ram.
    int FreeRam = stackTop - heapTop;

    Serial_Printf("heap used = %d, heap free = %d, stack + heap free = %d\n",mallinfo().uordblks ,mallinfo().fordblks, FreeRam);
  }
  break;

#ifdef PULSE_TEST

case hash("single_pulse"):
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

case hash("p2p"):
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
    Serial_Print_Line("wait...");
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
#endif

}
