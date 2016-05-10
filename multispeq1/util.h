

// functions in util.cpp

void applyMagCal(float* arr);
double getCompass(const float magX, const float magY, const float magZ, const double& pitch, const double& roll);
double getRoll(const int accelY, const int accelZ);
double getPitch(const int accelX, const int accelY, const int accelZ, const double& roll);
String getDirection(int compass);
Tilt calculateTilt(const double& roll, const double& pitch, double compass);
void scan_i2c(void);
void sleep_mode(void);


