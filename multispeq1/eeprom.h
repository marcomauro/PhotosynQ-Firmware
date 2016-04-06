
// Many variables used have to be stored/retrieved from eeprom.  All such variables are stored in a structure, with #defines for
// historical and convenience reasons.
// This structure is stored directly in eeprom memory (but it could be mirrored in regular ram).  Max size is 2K.
// Assume that there was a calibration process that initialized the eeprom.

// DANGER:  do not do too many writes to eeprom variables - cycles are limited
// Jon Zeeff 2016

// this structure overlays eeprom ram

// todo:  use placement new to initialize (until then, methods/contructor are not possible)

class eeprom_class
{
  public:
    float device_id;
    float manufacture_date;
    float mag_bias[3];       // magnetometer/compass calibration
    float mag_cal[3][3];
    float accel_bias[3];
    float accel_cal[3][3];   // accelerometer calibration

    // each of these needs a comment
// --> so these are admin defined, and each as a unique name (can be type, string, float, etc.)
    
    float light_slope;
    float light_y_intercept;
    float slope_34;
    float yintercept_34;
    float slope_35;
    float yintercept_35;

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

//    float userdef[50];


    float userdef0[2];
    float userdef1[2];
    float userdef2[2];
    float userdef3[2];
    float userdef4[2];
    float userdef5[2];
    float userdef6[2];
    float userdef7[2];
    float userdef8[2];
    float userdef9[2];
    float userdef10[2];
    float userdef11[2];
    float userdef12[2];
    float userdef13[2];
    float userdef14[2];
    float userdef15[2];
    float userdef16[2];
    float userdef17[2];
    float userdef18[2];
    float userdef19[2];
    float userdef20[2];
    float userdef21[2];
    float userdef22[2];
    float userdef23[2];
    float userdef24[2];
    float userdef25[2];
    float userdef26[2];
    float userdef27[2];
    float userdef28[2];
    float userdef29[2];
    float userdef30[2];
    float userdef31[2];
    float userdef32[2];
    float userdef33[2];
    float userdef34[2];
    float userdef35[2];
    float userdef36[2];
    float userdef37[2];
    float userdef38[2];
    float userdef39[2];
    float userdef40[2];
    float userdef41[2];
    float userdef42[2];
    float userdef43[2];
    float userdef44[2];
    float userdef45[2];
    float userdef46[3];
    float userdef47[3];
    float userdef48[3];
    float userdef49[3];
    float userdef50[3];

    // add more here, also add a #define as below
};

// where to store permanent data (teensy 3 specific)
#define FlexRAM ((eeprom_class *)0x14000000)

#ifndef EXTERN
extern class eeprom_class * const eeprom;
#else
class eeprom_class * const eeprom = FlexRAM;
#endif

#undef EXTERN

// these make the code that uses eeprom stored variables cleaner
//#define userdef[x] eeprom->userdef[x]

#define userdef0 eeprom->userdef0
#define userdef1 eeprom->userdef1
#define userdef2 eeprom->userdef2
#define userdef3 eeprom->userdef3
#define userdef4 eeprom->userdef4
#define userdef5 eeprom->userdef5
#define userdef6 eeprom->userdef6
#define userdef7 eeprom->userdef7
#define userdef8 eeprom->userdef8
#define userdef9 eeprom->userdef9
#define userdef10 eeprom->userdef10
#define userdef11 eeprom->userdef11
#define userdef12 eeprom->userdef12
#define userdef13 eeprom->userdef13
#define userdef14 eeprom->userdef14
#define userdef15 eeprom->userdef15
#define userdef16 eeprom->userdef16
#define userdef17 eeprom->userdef17
#define userdef18 eeprom->userdef18
#define userdef19 eeprom->userdef19
#define userdef20 eeprom->userdef20
#define userdef21 eeprom->userdef21
#define userdef22 eeprom->userdef22
#define userdef23 eeprom->userdef23
#define userdef24 eeprom->userdef24
#define userdef25 eeprom->userdef25
#define userdef26 eeprom->userdef26
#define userdef27 eeprom->userdef27
#define userdef28 eeprom->userdef28
#define userdef29 eeprom->userdef29
#define userdef30 eeprom->userdef30
#define userdef31 eeprom->userdef31
#define userdef32 eeprom->userdef32
#define userdef33 eeprom->userdef33
#define userdef34 eeprom->userdef34
#define userdef35 eeprom->userdef35
#define userdef36 eeprom->userdef36
#define userdef37 eeprom->userdef37
#define userdef38 eeprom->userdef38
#define userdef39 eeprom->userdef39
#define userdef40 eeprom->userdef40
#define userdef41 eeprom->userdef41
#define userdef42 eeprom->userdef42
#define userdef43 eeprom->userdef43
#define userdef44 eeprom->userdef44
#define userdef45 eeprom->userdef45
#define userdef46 eeprom->userdef46
#define userdef47 eeprom->userdef47
#define userdef48 eeprom->userdef48
#define userdef49 eeprom->userdef49
#define userdef50 eeprom->userdef50
