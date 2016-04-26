
// Serial routines that can read/write to either or both Serial devices (Serial (USB) and Serial1 (BLE)) based on a setting
// It also maintains and prints a CRC value
// Jon Zeeff 2016

// WARNING: Serial_Printf() is limited to 200 characters per call
// Zero is used as a string terminator - it can't be sent/received

// Should change this to C++ and a class

#include <Arduino.h>
#include "serial.h"
#include "utility/crc32.h"

static void Serial_Print_BLE(const char *str);

// select which serial port to print to
static int Serial_Port = 3;   // 1 == Serial, 2 == Serial1, 3 = both  (ignored during automatic mode)
static int automatic = 0;     // automatic means that writes will only go to the serial port that last had a byte read (Serial_Port is ignored)
static int last_read = 0;     // where last incoming byte was from, 0 = Serial, 1 = Serial1

// set baud rates
// TODO - verify that multiple calls are OK

void Serial_Begin(int rate)
{
  //  assert(rate >= 9600 && rate <= 115200);

  Serial.begin(115200);   // USB serial port, baud rate is irrelevant
  Serial1.begin(rate);    // for BLE, 57600 is standard
}

// discard all available input
void Serial_Flush_Input(void)
{
  while (Serial_Available())
    Serial_Read();
}

const int MAX_RESEND_SIZE = 5000;
static char resend_buffer[MAX_RESEND_SIZE + 1];      // allow re-transmission of previous output
static int  resend_count = 0;                        // number of chars in the resend buffer

// re-send everything since the last Serial_Start()
void Serial_Resend()
{
  Serial_Print(resend_buffer);
}

// specify which ports to send output to
void Serial_Set(int s)
{
  //  assert(s > 0 && s <= 4);

  if (s == 4) {                 // automatic means that writes will only go to the serial port that last had a byte available
    automatic = 1;
    Serial_Port = 3;
  } else {
    automatic = 0;
    Serial_Port = s;
  }
}

// Do a printf to either port
#define SIZE 200

void Serial_Printf(const char * format, ... )
{
  char string[SIZE + 1];        // Warning: fixed buffer size
  va_list v_List;
  va_start( v_List, format );
  vsnprintf( string, SIZE, format, v_List );

  //  assert(strlen(string) < SIZE);

  string[SIZE] = 0;
  Serial_Print(string);
  va_end( v_List );
}

int Serial_Read()
{
  for (;;) {
    if (Serial && Serial.available())  {
      last_read = 0;
      return Serial.read();
    }
    if (Serial1.available()) {
      last_read = 1;
      return Serial1.read();
    }
  } // for
}  // Serial_Read()


unsigned Serial_Available()
{
  unsigned count = 0;

  if (Serial)
    count += Serial.available();

  count += Serial1.available();

  return count;
}  // Serial_Available()


int Serial_Peek()
{
  if (Serial && Serial.available())
    return Serial.peek();

  if (Serial1.available())
    return Serial1.peek();

  return -1;
}  // Serial_Available()


// routines to print to two serial ports with a CRC at the end

void
Serial_Print (const char *str)    // other Serial_Print() routines call this one
{
  // output to specified ports
  // Caution:  may be buffered and require a Serial_Flush_Output() to see the entire string

  if (automatic) {             // automatic mode takes precedence over Serial_Port setting
    if (last_read == 0) {
      if (Serial)
        Serial.print(str);
    } else
      Serial_Print_BLE(str);
  } else {
    // non-automatic mode
    if (Serial && (Serial_Port & 1))
      Serial.print(str);

    if (Serial_Port & 2)
      Serial_Print_BLE(str);      // send with time delays to rate limit
  } // if

  // add to crc value
  crc32_string ((char *)str);

  // add to resend buffer
  int count = min((int)strlen(str), MAX_RESEND_SIZE - resend_count);  // check for enough remaining space
  strncpy(resend_buffer + resend_count, (char *)str, count);
  resend_count += count;
  resend_buffer[resend_count] = 0;    // null terminate
}

#define BLE_DELAY 20                    // milli seconds between packets
#define BLE_PACKET_SIZE 20

static char buffer[BLE_PACKET_SIZE + 1];
static int count = 0;                   // how many bytes in the above buffer

// output to the BLE serial port, but buffer into 20 byte packets and put delays between packets to limit to 1Kbytes/sec
// does not add to CRC

static void Serial_Print_BLE(const char *str)
{
  // copy to buffer, sending whenever it reaches n bytes
  while (*str != 0) {                  // until end of string

    buffer[count] = *str;
    ++count;
    ++str;

    if (count == BLE_PACKET_SIZE) { // full buffer - send it
      buffer[count] = 0;            // null terminate the string
      Serial1.print(buffer);        // send it
      Serial1.flush();              // make sure it goes out
      delay(BLE_DELAY);             // wait for transmission to avoid overrunning buffers
      count = 0;
    }
  } // while
}

// send any buffered characters
void Serial_Flush_Output()
{
  // for BLE, where we have a buffer
  if (count > 0) {
    buffer[count] = 0;            // null terminate the string
    Serial1.print(buffer);        // send it
    Serial1.flush();              // make sure it goes out
    delay(BLE_DELAY);             // wait for transmission to avoid overrunning buffers
    count = 0;
  }

  // for normal serial (not sure what this does)
  //if (Serial)
  //   Serial.flush();
}


void
Serial_Print(const float x, int places)
{
  char str[50 + 1];
  snprintf(str, 50,"%.*f", places, x);
  // output to both ports
  Serial_Print ((char *)str);
}

void
Serial_Print(const double xx, int places)
{
  char str[50 + 1];
  snprintf(str, 50,"%.*f", places, xx);
  // output to both ports
  Serial_Print ((char *)str);
}

void
Serial_Print(const int i)
{
  char str[50+1];
  snprintf(str, 50,"%d", i);
  // output to both ports
  Serial_Print ((char *)str);
}

void
Serial_Print(const unsigned u)
{
  char str[50+1];
  snprintf(str, 50,"%u", u);
  // output to both ports
  Serial_Print ((char *)str);
}

void
Serial_Print(const String string)
{
  // output to both ports
  Serial_Print (string.c_str());
}


void
Serial_Print_Line (const char *str)
{
  // output to both ports
  Serial_Print(str);
  Serial_Print("\n");
}

void
Serial_Print_Line(String string)
{
  // output to both ports
  Serial_Print (string.c_str());
  Serial_Print("\r\n");
}


void
Serial_Print_Line (const int i)
{
  char str[50+1];
  snprintf(str, 50, "%d", i);
  // output to both ports
  Serial_Print_Line ((char *)str);
}

void
Serial_Print_Line (const long i)
{
  char str[50+1];
  snprintf(str, 50, "%ld", i);
  // output to both ports
  Serial_Print_Line ((char *)str);
}

void
Serial_Print_Line (const float x, const int places)
{
  char str[50 + 1];
  snprintf(str, 50, "%.*f", places, x);
  //  assert(strlen(str) < 20);
  // output to both ports
  Serial_Print_Line ((char *)str);
}

void
Serial_Print_Line (const double xx, const int places)
{
  char str[50 + 1];
  snprintf(str, 50, "%.*f", places, xx);
  //  assert(strlen(str) < 20);
  // output to both ports
  Serial_Print_Line ((char *)str);
}

// print CRC (8 capital hex digits) and a newline and clear the crc value for the next time
void
Serial_Print_CRC (void)
{
  char *p = int32_to_hex (crc32_value ());

  Serial_Print_Line (p);
  Serial_Flush_Output();          // force it to go out

  Serial_Start();                 // new packet
}

// start an output packet (used to resend output)
void
Serial_Start(void)
{
  crc32_init ();          // reset for next time
  resend_count = 0;       // empty resend buffer
}

#include <string.h>

// read a string, terminating on any of:
//   1) time (in ms) runs out
//   2) a terminating character is received
//   3) input length = max_length (string should be one larger to hold the terminating null)

char *Serial_Input_Chars(char *string, const char *terminators, long unsigned int timeout, unsigned int max_length)  {       // terminating characters must be single characters or a list of single characters (so "+" will terminal on +, while "+-" will terminal on either + or -).
  unsigned long start = millis();
  unsigned count = 0;

  // loop forever or until a condition is met
  for (;;) {
    int c = Serial_Peek();

    if (timeout > 0 && (millis() - start) > timeout)  // timeout
      break;                                          // done

    if (c == -1) continue;    // nothing available

    char b = Serial_Read();
    if (strchr(terminators, b))                     // terminator char seen - throw it away
      break;

    string[count++] = b;                            // add to string

    if (max_length > 0 && count >= max_length)        // too long
      break;

  } // for ever

  string[count] = 0;   // always null terminate the string

  return string;

}  // Serial_Input_Chars()


// read a float value (see above)
// empty string returns NAN

double Serial_Input_Double(const char *terminators, long unsigned int timeout) {
  char S[25];
  Serial_Input_Chars(S, terminators, timeout, sizeof(S) - 1);
  if (strlen(S) == 0)
    return NAN;
  return strtod(S,0);
}  // Serial_Input_Double()


// read a long value (see above)
// invalid strings return 0

long Serial_Input_Long(const char *terminators, long unsigned int timeout) {
  char S[25];
  Serial_Input_Chars(S, terminators, timeout, sizeof(S) - 1);
  return atol(S);
}  // Serial_Input_Long()



String Serial_Input_String(const char *terminators, long unsigned int timeout)
{
  static String serial_string;
  char str[100 + 1]; // caution - fixed size buffer

  Serial_Input_Chars(str, terminators, timeout, sizeof(str) - 1);

  serial_string = str;

  //  assert(strlen(str) < 100);

  return serial_string;

}  // user_enter_str()


