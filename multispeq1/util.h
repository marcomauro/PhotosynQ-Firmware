

// functions in util.cpp

void applyMagCal(float* arr);
float getCompass(const float magX, const float magY, const float magZ, const float& pitch, const float& roll);
float getRoll(const int accelY, const int accelZ);
float getPitch(const int accelX, const int accelY, const int accelZ, const float& roll);
String getDirection(int compass);
Tilt calculateTilt(const float& roll, const float& pitch, float compass);
int compass_segment(float angle);
void scan_i2c(void);
void sleep_mode(void);


