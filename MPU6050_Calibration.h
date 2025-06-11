#ifndef MPU6050_CALIBRATION_H
#define MPU6050_CALIBRATION_H

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

class MPU6050Calibration {
private:
    MPU6050 accelgyro;
    int buffersize = 1000;
    int acel_deadzone = 8;
    int gyro_deadzone = 1;
    

    int16_t ax, ay, az, gx, gy, gz;
    int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
    int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

    void meansensors();
    void calibration();
    void runCalibrationWithLED(int ledPin = 13);


public:
    MPU6050Calibration(uint8_t address = 0x68);
    void begin();
    void runCalibration();

    int getAccelXOffset();
    int getAccelYOffset();
    int getAccelZOffset();
    int getGyroXOffset();
    int getGyroYOffset();
    int getGyroZOffset();
};

#endif
