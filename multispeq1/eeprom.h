
// Many variables used have to be stored/retrieved from eeprom.  All such variables are stored in a structure, with #defines for
// historical and convenience reasons.
// This structure is stored directly in eeprom memory
// Assume that there was a calibration process that initialized the eeprom.

// DANGER:  do not do too many writes to eeprom variables - cycles are limited
// WARNING: max size is 2K
// WARNING: do a delay(1) after every write.  If not, program may freeze.

// Jon Zeeff 2016

// this structure overlays eeprom ram

// todo:  use placement new to initialize (until then, methods/contructor are not possible)

#include <stdint.h>

#define NUM_USERDEFS 50

class eeprom_class
{
  public:
    // each of these needs a comment
    long device_id;                   // lower 4 bytes of BLE mac address
    uint16_t manufacture_date;
    float mag_bias[3];       // magnetometer/compass calibration
    float mag_cal[3][3];
    float accel_bias[3];
    float accel_cal[3][3];   // accelerometer calibration
    /*
       now we calculate par based on rgb components of the incoming light and a yintercept
    */
    //    float light_slope;
    float light_yint;
    float light_slope_r;
    float light_slope_g;
    float light_slope_b;
    //    float slope_34;
    //    float yintercept_34;
    //    float slope_35;
    //    float yintercept_35;
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
    float par_to_dac_slope[11];
    float par_to_dac_yint[11];
    /*
       IR baseline estimation based on the reflectivity of the object.  This helps address any residual IR in the measuring light itself (used in chlorophyll fluorescence).
    */
    float ir_baseline_slope[11];
    float ir_baseline_yint[11];
    /*
       values for color calibrations (used for SPAD, anthocyanin, etc.) at 3 intensity levels (allows for simpler protocols as thickness doesn't need to be adjusted or on the fly).
    */
    float colorcal_intensity1_slope[11];
    float colorcal_intensity1_yint[11];
    float colorcal_intensity2_slope[11];
    float colorcal_intensity2_yint[11];
    float colorcal_intensity3_slope[11];
    float colorcal_intensity3_yint[11];

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

// these make the code that uses eeprom stored variables cleaner
//#define userdef[x] eeprom->userdef[x]


// since eeprom requires a delay(1) after write, it's better to not use these.  This makes the delay() requirement clearer.
// consider mapping many of these to userdef values

/*
#define mag_bias eeprom->mag_bias
#define mag_cal eeprom->mag_cal
#define accel_bias eeprom->accel_bias
#define accel_cal eeprom->accel_cal
#define light_yint eeprom->light_yint
#define light_slope_r eeprom->light_slope_r
#define light_slope_g eeprom->light_slope_g
#define light_slope_b eeprom->light_slope_b
#define detector_offset_slope eeprom->detector_offset_slope
#define detector_offset_yint eeprom->detector_offset_yint
#define thickness_a eeprom->thickness_a
#define thickness_b eeprom->thickness_b
#define thickness_d eeprom->thickness_d
#define par_to_dac_slope eeprom->par_to_dac_slope
#define par_to_dac_yint eeprom->par_to_dac_yint
#define ir_baseline_slope eeprom->ir_baseline_slope
#define ir_baseline_yint eeprom->ir_baseline_yint
#define colorcal_intensity1_slope eeprom->colorcal_intensity1_slope
#define colorcal_intensity1_yint eeprom->colorcal_intensity1_yint
#define colorcal_intensity2_slope eeprom->colorcal_intensity2_slope
#define colorcal_intensity2_yint eeprom->colorcal_intensity2_yint
#define colorcal_intensity3_slope eeprom->colorcal_intensity3_slope
#define colorcal_intensity3_yint eeprom->colorcal_intensity3_yint
#define userdef eeprom->userdef
*/



