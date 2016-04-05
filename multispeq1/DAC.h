
//  Routines to set multispeq DAC values.
//  Do not write to the DACs or ldac lines anywhere else.
//  Any calls from outside this file assume that pins are numbered 1-x (vs 0-x).
//  *** Important: assume that the DAC ICs have addresses of 1,2,3 that were set with other software
//  Jon Zeeff, March, 2016
// test

#define NUM_DACS 3
#define NUM_LED_PINS 10         // number of LEDs

// which MCU pin is attached to each dac
#define LDAC1    15
#define LDAC2    16
#define LDAC3    17

// DAC routines
int DAC_init(void);
void DAC_change(void);
void DAC_set(unsigned int pin, unsigned int value);
int DAC_set_address(int ldac_pin, unsigned oldAddress, unsigned newAddress); 

