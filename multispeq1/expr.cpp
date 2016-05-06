// original author James Gregson (james.gregson@gmail.com)
// includes enhancements for PhotoSynQ

#include <stdio.h>
#include <string.h>
#include "defines.h"
#include "eeprom.h"
#include "serial.h"

extern "C" {
#include "expr/expression_parser.h"
}

// any reference to a variable must be processed here

int variable_callback( void *user_data, const char *name, double *value ) {
  // look up the variables by name
  // set return value, return true

  // could make a function that given a variable name string, return the address
  // only useful if writes are needed
  
  // Userdef commands
  if (strncmp( name, "userdef[", 8) == 0) {  // handles all userdef[x] references
    unsigned index = atoi(name + 8);
    if (index < NUM_USERDEFS)
      *value = eeprom->userdef[index];
    else
      *value = NAN;
    return PARSER_TRUE;
    
    // Responses from sensors
  } else if (strcmp( name, "light_intensity" ) == 0 ) {
    // set return value, return true
    *value = light_intensity;
    return PARSER_TRUE;
  } else if (strcmp( name, "light_intensity_averaged" ) == 0 ) {
    // set return value, return true
    *value = light_intensity_averaged;
    return PARSER_TRUE;
  } else if (strcmp( name, "light_intensity_raw" ) == 0 ) {
    // set return value, return true
    *value = light_intensity_raw;
    return PARSER_TRUE;
  } else if (strcmp( name, "light_intensity_raw_averaged" ) == 0 ) {
    // set return value, return true
    *value = light_intensity_raw_averaged;
    return PARSER_TRUE;
  } else if (strcmp( name, "r" ) == 0 ) {
    // set return value, return true
    *value = r;
    return PARSER_TRUE;
  } else if (strcmp( name, "r_averaged" ) == 0 ) {
    // set return value, return true
    *value = r_averaged;
    return PARSER_TRUE;
  } else if (strcmp( name, "g" ) == 0 ) {
    // set return value, return true
    *value = g;
    return PARSER_TRUE;
  } else if (strcmp( name, "g_averaged" ) == 0 ) {
    // set return value, return true
    *value = g_averaged;
    return PARSER_TRUE;
  } else if (strcmp( name, "b" ) == 0 ) {
    // set return value, return true
    *value = b;
    return PARSER_TRUE;
  } else if (strcmp( name, "b_averaged" ) == 0 ) {
    // set return value, return true
    *value = b_averaged;
    return PARSER_TRUE;
  } else if (strcmp( name, "thickness" ) == 0 ) {
    // set return value, return true
    *value = thickness;
    return PARSER_TRUE;
  } else if (strcmp( name, "thickness_averaged" ) == 0 ) {
    // set return value, return true
    *value = thickness_averaged;
    return PARSER_TRUE;
  } else if (strcmp( name, "thickness_raw" ) == 0 ) {
    // set return value, return true
    *value = thickness_raw;
    return PARSER_TRUE;
  } else if (strcmp( name, "thickness_raw_averaged" ) == 0 ) {
    // set return value, return true
    *value = thickness_raw_averaged;
    return PARSER_TRUE;
  } else if (strcmp( name, "contactless_temp" ) == 0 ) {
    // set return value, return true
    *value = contactless_temp;
    return PARSER_TRUE;
  } else if (strcmp( name, "contactless_temp_averaged" ) == 0 ) {
    // set return value, return true
    *value = contactless_temp_averaged;
    return PARSER_TRUE;
  } else if (strcmp( name, "x_tilt" ) == 0 ) {
    // set return value, return true
    *value = x_tilt;
    return PARSER_TRUE;
  } else if (strcmp( name, "y_tilt" ) == 0 ) {
    // set return value, return true
    *value = y_tilt;
    return PARSER_TRUE;
  } else if (strcmp( name, "z_tilt" ) == 0 ) {
    // set return value, return true
    *value = z_tilt;
    return PARSER_TRUE;
  } else if (strcmp( name, "x_tilt_averaged" ) == 0 ) {
    // set return value, return true
    *value = x_tilt_averaged;
    return PARSER_TRUE;
  } else if (strcmp( name, "y_tilt_averaged" ) == 0 ) {
    // set return value, return true
    *value = y_tilt_averaged;
    return PARSER_TRUE;
  } else if (strcmp( name, "z_tilt_averaged" ) == 0 ) {
    // set return value, return true
    *value = z_tilt_averaged;
    return PARSER_TRUE;
  } else if (strcmp( name, "x_tilt_raw" ) == 0 ) {
    // set return value, return true
    *value = x_tilt_raw;
    return PARSER_TRUE;
  } else if (strcmp( name, "y_tilt_raw" ) == 0 ) {
    // set return value, return true
    *value = y_tilt_raw;
    return PARSER_TRUE;
  } else if (strcmp( name, "z_tilt_raw" ) == 0 ) {
    // set return value, return true
    *value = z_tilt_raw;
    return PARSER_TRUE;
  } else if (strcmp( name, "x_tilt_raw_averaged" ) == 0 ) {
    // set return value, return true
    *value = x_tilt_raw_averaged;
    return PARSER_TRUE;
  } else if (strcmp( name, "y_tilt_raw_averaged" ) == 0 ) {
    // set return value, return true
    *value = y_tilt_raw_averaged;
    return PARSER_TRUE;
  } else if (strcmp( name, "z_tilt_raw_averaged" ) == 0 ) {
    // set return value, return true
    *value = z_tilt_raw_averaged;
    return PARSER_TRUE;
  } else if (strcmp( name, "temperature" ) == 0 ) {
    // set return value, return true
    *value = temperature;
    return PARSER_TRUE;
  } else if (strcmp( name, "humidity" ) == 0 ) {
    // set return value, return true
    *value = humidity;
    return PARSER_TRUE;
  } else if (strcmp( name, "pressure" ) == 0 ) {
    // set return value, return true
    *value = pressure;
    return PARSER_TRUE;
  } else if (strcmp( name, "temperature_averaged" ) == 0 ) {
    // set return value, return true
    *value = temperature_averaged;
    return PARSER_TRUE;
  } else if (strcmp( name, "humidity_averaged" ) == 0 ) {
    // set return value, return true
    *value = humidity_averaged;
    return PARSER_TRUE;
  } else if (strcmp( name, "pressure_averaged" ) == 0 ) {
    // set return value, return true
    *value = pressure_averaged;
    return PARSER_TRUE;
  } else if (strcmp( name, "temperature2" ) == 0 ) {
    // set return value, return true
    *value = temperature2;
    return PARSER_TRUE;
  } else if (strcmp( name, "humidity2" ) == 0 ) {
    // set return value, return true
    *value = humidity2;
    return PARSER_TRUE;
  } else if (strcmp( name, "pressure2" ) == 0 ) {
    // set return value, return true
    *value = pressure2;
    return PARSER_TRUE;
  } else if (strcmp( name, "temperature2_averaged" ) == 0 ) {
    // set return value, return true
    *value = temperature2_averaged;
    return PARSER_TRUE;
  } else if (strcmp( name, "humidity2_averaged" ) == 0 ) {
    // set return value, return true
    *value = humidity2_averaged;
    return PARSER_TRUE;
  } else if (strcmp( name, "pressure2_averaged" ) == 0 ) {
    // set return value, return true
    *value = pressure2_averaged;
    return PARSER_TRUE;
    
    // Admin defined values
  } else if (strcmp( name, "light_yint" ) == 0 ) {
    // set return value, return true
    *value = eeprom->light_yint;
    return PARSER_TRUE;
  } else if (strcmp( name, "light_slope_all" ) == 0 ) {
    // set return value, return true
    *value = eeprom->light_slope_all;
    return PARSER_TRUE;
  } else if (strcmp( name, "light_slope_r" ) == 0 ) {
    // set return value, return true
    *value = eeprom->light_slope_r;
    return PARSER_TRUE;
  } else if (strcmp( name, "light_slope_g" ) == 0 ) {
    // set return value, return true
    *value = eeprom->light_slope_g;
    return PARSER_TRUE;
  } else if (strcmp( name, "light_slope_b" ) == 0 ) {
    // set return value, return true
    *value = eeprom->light_slope_b;
    return PARSER_TRUE;
  } else if (strcmp( name, "thickness_a" ) == 0 ) {
    // set return value, return true
    *value = eeprom->thickness_a;
    return PARSER_TRUE;
  } else if (strcmp( name, "thickness_b" ) == 0 ) {
    // set return value, return true
    *value = eeprom->thickness_b;
    return PARSER_TRUE;
  } else if (strcmp( name, "thickness_d" ) == 0 ) {
    // set return value, return true
    *value = eeprom->thickness_d;
    return PARSER_TRUE;
  } else  if (strncmp( name, "detector_offset_slope[", 22) == 0 ) {
    unsigned index = atoi(name + 22);
    if (index < 4)
      *value = eeprom->detector_offset_slope[index];
    else
      *value = NAN;
    return PARSER_TRUE;
  } else  if (strncmp( name, "detector_offset_yint[", 21) == 0 ) {
    unsigned index = atoi(name + 21);
    if (index < 4)
      *value = eeprom->detector_offset_yint[index];
    else
      *value = NAN;
    return PARSER_TRUE;
  } else  if (strncmp( name, "mag_bias[", 9) == 0 ) {
    unsigned index = atoi(name + 9);
    if (index < 3)
      *value = eeprom->mag_bias[index];
    else
      *value = NAN;
    return PARSER_TRUE;
  } else  if (strncmp( name, "accel_bias[", 11) == 0 ) {
    unsigned index = atoi(name + 11);
    if (index < 3)
      *value = eeprom->accel_bias[index];
    else
      *value = NAN;
    return PARSER_TRUE;
  } else  if (strncmp( name, "par_to_dac_slope[", 17) == 0 ) {
    unsigned index = atoi(name + 17);
    if (index < NUM_LEDS + 1)
      *value = eeprom->par_to_dac_slope[index];
    else
      *value = NAN;
    return PARSER_TRUE;
  } else  if (strncmp( name, "par_to_dac_yint[", 16) == 0 ) {
    unsigned index = atoi(name + 16);
    if (index < NUM_LEDS + 1)
      *value = eeprom->par_to_dac_yint[index];
    else
      *value = NAN;
    return PARSER_TRUE;
  } else  if (strncmp( name, "ir_baseline_slope[", 18) == 0 ) {
    unsigned index = atoi(name + 18);
    if (index < NUM_LEDS + 1)
      *value = eeprom->ir_baseline_slope[index];
    else
      *value = NAN;
    return PARSER_TRUE;
  } else  if (strncmp( name, "ir_baseline_yint[", 17) == 0 ) {
    unsigned index = atoi(name + 17);
    if (index < NUM_LEDS + 1)
      *value = eeprom->ir_baseline_yint[index];
    else
      *value = NAN;
    return PARSER_TRUE;
  } else  if (strncmp( name, "colorcal_intensity1_slope[", 26) == 0 ) {
    unsigned index = atoi(name + 26);
    if (index < NUM_LEDS + 1)
      *value = eeprom->colorcal_intensity1_slope[index];
    else
      *value = NAN;
    return PARSER_TRUE;
  } else  if (strncmp( name, "colorcal_intensity2_slope[", 26) == 0 ) {
    unsigned index = atoi(name + 26);
    if (index < NUM_LEDS + 1)
      *value = eeprom->colorcal_intensity2_slope[index];
    else
      *value = NAN;
    return PARSER_TRUE;
  } else  if (strncmp( name, "colorcal_intensity3_slope[", 26) == 0 ) {
    unsigned index = atoi(name + 26);
    if (index < NUM_LEDS + 1)
      *value = eeprom->colorcal_intensity3_slope[index];
    else
      *value = NAN;
    return PARSER_TRUE;
  } else  if (strncmp( name, "colorcal_intensity1_yint[", 25) == 0 ) {
    unsigned index = atoi(name + 25);
    if (index < NUM_LEDS + 1)
      *value = eeprom->colorcal_intensity1_yint[index];
    else
      *value = NAN;
    return PARSER_TRUE;
  } else  if (strncmp( name, "colorcal_intensity2_yint[", 25) == 0 ) {
    unsigned index = atoi(name + 25);
    if (index < NUM_LEDS + 1)
      *value = eeprom->colorcal_intensity2_yint[index];
    else
      *value = NAN;
    return PARSER_TRUE;
  } else  if (strncmp( name, "colorcal_intensity3_yint[", 25) == 0 ) {
    unsigned index = atoi(name + 25);
    if (index < NUM_LEDS + 1)
      *value = eeprom->colorcal_intensity3_yint[index];
    else
      *value = NAN;
    return PARSER_TRUE;
  } else  if (strncmp( name, "colorcal_blank1[", 16) == 0 ) {
    unsigned index = atoi(name + 16);
    if (index < NUM_LEDS + 1)
      *value = eeprom->colorcal_blank1[index];
    else
      *value = NAN;
    return PARSER_TRUE;
  } else  if (strncmp( name, "colorcal_blank2[", 16) == 0 ) {
    unsigned index = atoi(name + 16);
    if (index < NUM_LEDS + 1)
      *value = eeprom->colorcal_blank2[index];
    else
      *value = NAN;
    return PARSER_TRUE;
  } else  if (strncmp( name, "colorcal_blank3[", 16) == 0 ) {
    unsigned index = atoi(name + 16);
    if (index < NUM_LEDS + 1)
      *value = eeprom->colorcal_blank3[index];
    else
      *value = NAN;
    return PARSER_TRUE;
  }

  // failed to find variable, return false
  return PARSER_FALSE;
}

/**
  @brief user-defined function callback. see expression_parser.h for more details.
  @param[in] user_data input pointer to any user-defined state variables needed.  in this case, this pointer is the maximum number of arguments allowed to the functions (as a contrived example usage).
  @param[in] name name of the function to evaluate
  @param[in] num_args number of arguments that were parsed in the function call
  @param[in] args list of parsed arguments
  @param[out] value output evaluated result of the function call
  @return true if the function exists and was evaluated successfully with the result stored in value, false otherwise.
*/
int function_callback( void *user_data, const char *name, const int num_args, const double *args, double *value ) {
  int i, max_args;
  double tmp;

  // example to show the user-data parameter, sets the maximum number of
  // arguments allowed for the following functions from the user-data function
  max_args = *((int*)user_data);

  if ( strcmp( name, "max_value") == 0 && num_args >= 2 && num_args <= max_args ) {
    // example 'maximum' function, returns the largest of the arguments, this and
    // the min_value function implementation below allow arbitrary number of arguments
    tmp = args[0];
    for ( i = 1; i < num_args; i++ ) {
      tmp = args[i] >= tmp ? args[i] : tmp;
    }
    // set return value and return true
    *value = tmp;
    return PARSER_TRUE;
  } else if ( strcmp( name, "min_value" ) == 0 && num_args >= 2 && num_args <= max_args ) {
    // example 'minimum' function, returns the smallest of the arguments
    tmp = args[0];
    for ( i = 1; i < num_args; i++ ) {
      tmp = args[i] <= tmp ? args[i] : tmp;
    }
    // set return value and return true
    *value = tmp;
    return PARSER_TRUE;
  }

  // failed to evaluate function, return false
  return PARSER_FALSE;
}

// given an expression, evaluate it

double expr(const char str[])
{
  int num_arguments = 3;

  if (str == 0 || str[0] == 0)
    return NAN;

  Serial_Flush_Output();

  return parse_expression_with_callbacks( str, variable_callback, function_callback, &num_arguments );
}  // expr()
