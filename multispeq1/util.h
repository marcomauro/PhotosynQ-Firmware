

// functions in util.cpp

void applyMagCal(float* arr);
void applyAccCal(int* arr);
void rad_to_deg(float* roll, float* pitch, float* yaw);
float getCompass(const float magX, const float magY, const float magZ, const float& pitch, const float& roll);
float getRoll(const int accelY, const int accelZ);
float getPitch(const int accelX, const int accelY, const int accelZ, const float& roll);
const char *getDirection(int compass);
Tilt calculateTilt(float roll, float pitch, float compass);
int compass_segment(float angle);
void scan_i2c(void);
void sleep_mode(int n);
int battery_low(int leds);
float sine_internal(float angle);
float cosine_internal(float angle);
//removed because doesn't work and probably unnecessary - see util.cpp comments for more details
//float arctan_internal(float x, float y);
//float float_max(float x, float y);
//float float_min(float x, float y);
void timefromcompiler(void);

