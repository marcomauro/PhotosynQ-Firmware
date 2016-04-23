// original author James Gregson (james.gregson@gmail.com)
// includes enhancements for PhotoSynQ

#include<stdio.h>
#include<string.h>
#include "defines.h"
#include "eeprom.h"

extern "C" {
#include "expr/expression_parser.h"
}

// any reference to a variable must be processed here

int variable_callback( void *user_data, const char *name, double *value ) {
  // look up the variables by name
  // set return value, return true

  if (strncmp( name, "userdef", 7) == 0 ) {  // handles all userdefx references
    unsigned index = atoi(name + 7);
    if (index < NUM_USERDEFS)
      *value = eeprom->userdef[index];
    else 
      *value = NAN;
    return PARSER_TRUE;
  } else if (strcmp( name, "light_intensity" ) == 0 ) {
    // set return value, return true
    *value = light_intensity;
    return PARSER_TRUE;
  } else if ( strcmp( name, "var2" ) == 0 ) {
    // set return value, return true
    *value = 2.0;
    return PARSER_TRUE;
  } else if ( strcmp( name, "var3" ) == 0 ) {
    // set return value, return true
    *value = 3.0;
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

  return parse_expression_with_callbacks( str, variable_callback, function_callback, &num_arguments );
}  // expr()
