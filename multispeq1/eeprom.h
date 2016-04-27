
// Many variables used have to be stored/retrieved from eeprom.  All such variables are stored in a structure, with #defines for
// historical and convenience reasons.
// This structure is stored directly in eeprom memory
// Assume that there was a calibration process that initialized the eeprom.

// DANGER:  do not do too many writes to eeprom variables - cycles are limited
// WARNING: max size is 2K
// WARNING: do a delay(1) after every write.  If not, program may freeze.

// Jon Zeeff 2016

// this structure overlays eeprom ram

#include <stdint.h>

const unsigned NUM_USERDEFS=50;

class eeprom_class
{
  public:
    // each of these needs a comment
     long device_id;                   // lower 4 bytes of BLE mac address
     long device_manufacture;        // month and year, eg 12016
     float mag_bias[3];       // magnetometer/compass calibration
     float mag_cal[3][3];
     float accel_bias[3];
     float accel_cal[3][3];   // accelerometer calibration
    /*
       now we calculate par based on rgb components of the incoming light and a yintercept
    */
    //     float light_slope;
     float light_yint;         // y intercept for par calibration from tcs
     float light_slope_all;    // slope for the total intensity value for par calibration from tcs
     float light_slope_r;      // slope for just the red portion of the part calibration from tcs
     float light_slope_g;      // " " just green
     float light_slope_b;      // " " just blue
    //     float slope_34;
    //     float yintercept_34;
    //     float slope_35;
    //     float yintercept_35;
     float detector_offset_slope[4];      // slope and yint for detector offsets (detector 1 - 4 as array positions 1 - 4)
     float detector_offset_yint[4];
    /*
       calibration for leaf thickness (3 parameters - a, b, and d)
    */
     float thickness_a;
     float thickness_b;
     float thickness_d;
    /*
       replace calibration_slope and all that just with par_to_dac (and create a simple function for conversion)...- if the relationships is more complicated than just mx+b then add more variables as needed
    */
     float par_to_dac_slope[NUM_LEDS+1];
     float par_to_dac_yint[NUM_LEDS+1];    
    /*
       IR baseline estimation based on the reflectivity of the object.  This helps address any residual IR in the measuring light itself (used in chlorophyll fluorescence).
    */
     float ir_baseline_slope[NUM_LEDS+1];
     float ir_baseline_yint[NUM_LEDS+1];
    /*
       values for color calibrations (used for SPAD, anthocyanin, etc.) at 3 intensity levels (allows for simpler protocols as thickness doesn't need to be adjusted or on the fly).
    */
     float colorcal_intensity1_slope[NUM_LEDS+1];
     float colorcal_intensity1_yint[NUM_LEDS+1];
     float colorcal_intensity2_slope[NUM_LEDS+1];
     float colorcal_intensity2_yint[NUM_LEDS+1];
     float colorcal_intensity3_slope[NUM_LEDS+1];
     float colorcal_intensity3_yint[NUM_LEDS+1];

    // user set values
     float userdef[NUM_USERDEFS];  

    /*
         float calibration_slope;
         float calibration_yint;
         float calibration_slope_factory;
         float calibration_yint_factory;
         float calibration_baseline_slope;
         float calibration_baseline_yint;
         float calibration_blank1;
         float calibration_blank2;
         float calibration_other1;
         float calibration_other2;
         float pwr_off_ms[2];               // number of milliseconds before unit auto powers down (why two values?)
    */
 
    // add more here, also add a #define as below
};

// where to store permanent data (teensy 3 specific)
#define FlexRAM ((eeprom_class *)0x14000000)

#ifndef EXTERN
extern class eeprom_class * eeprom;
#else
class eeprom_class * eeprom = FlexRAM;
#endif

#undef EXTERN


