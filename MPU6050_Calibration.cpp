#include "MPU6050_Calibration.h"

MPU6050Calibration::MPU6050Calibration(uint8_t address)
: accelgyro(address) {}

void MPU6050Calibration::begin() {
    Wire.begin();
    TWBR = 24; // Set I2C speed
    accelgyro.initialize();
}

void MPU6050Calibration::runCalibration() {
    accelgyro.setXAccelOffset(0);
    accelgyro.setYAccelOffset(0);
    accelgyro.setZAccelOffset(0);
    accelgyro.setXGyroOffset(0);
    accelgyro.setYGyroOffset(0);
    accelgyro.setZGyroOffset(0);

    meansensors();
    calibration();
    meansensors();
}

void MPU6050Calibration::meansensors() {
    long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

    while (i < (buffersize + 101)) {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        if (i > 100 && i <= (buffersize + 100)) {
            buff_ax += ax;
            buff_ay += ay;
            buff_az += az;
            buff_gx += gx;
            buff_gy += gy;
            buff_gz += gz;
        }

        if (i == (buffersize + 100)) {
            mean_ax = buff_ax / buffersize;
            mean_ay = buff_ay / buffersize;
            mean_az = buff_az / buffersize;
            mean_gx = buff_gx / buffersize;
            mean_gy = buff_gy / buffersize;
            mean_gz = buff_gz / buffersize;
        }
        i++;
        delay(2);
    }
}

void MPU6050Calibration::calibration() {
    ax_offset = -mean_ax / 8;
    ay_offset = -mean_ay / 8;
    az_offset = (16384 - mean_az) / 8;

    gx_offset = -mean_gx / 4;
    gy_offset = -mean_gy / 4;
    gz_offset = -mean_gz / 4;

    while (true) {
        int ready = 0;
        accelgyro.setXAccelOffset(ax_offset);
        accelgyro.setYAccelOffset(ay_offset);
        accelgyro.setZAccelOffset(az_offset);
        accelgyro.setXGyroOffset(gx_offset);
        accelgyro.setYGyroOffset(gy_offset);
        accelgyro.setZGyroOffset(gz_offset);

        meansensors();

        if (abs(mean_ax) <= acel_deadzone) ready++;
        else ax_offset -= mean_ax / acel_deadzone;

        if (abs(mean_ay) <= acel_deadzone) ready++;
        else ay_offset -= mean_ay / acel_deadzone;

        if (abs(16384 - mean_az) <= acel_deadzone) ready++;
        else az_offset += (16384 - mean_az) / acel_deadzone;

        if (abs(mean_gx) <= gyro_deadzone) ready++;
        else gx_offset -= mean_gx / (gyro_deadzone + 1);

        if (abs(mean_gy) <= gyro_deadzone) ready++;
        else gy_offset -= mean_gy / (gyro_deadzone + 1);

        if (abs(mean_gz) <= gyro_deadzone) ready++;
        else gz_offset -= mean_gz / (gyro_deadzone + 1);

        if (ready == 6) break;
    }
}

void MPU6050Calibration::runCalibrationWithLED(int ledPin) {
    pinMode(ledPin, OUTPUT);

    // Reset offsets before starting
    accelgyro.setXAccelOffset(0);
    accelgyro.setYAccelOffset(0);
    accelgyro.setZAccelOffset(0);
    accelgyro.setXGyroOffset(0);
    accelgyro.setYGyroOffset(0);
    accelgyro.setZGyroOffset(0);

    // Initial sensor average
    meansensors();

    // Calculate initial offsets
    ax_offset = -mean_ax / 8;
    ay_offset = -mean_ay / 8;
    az_offset = (16384 - mean_az) / 8;
    gx_offset = -mean_gx / 4;
    gy_offset = -mean_gy / 4;
    gz_offset = -mean_gz / 4;

    while (true) {
        digitalWrite(ledPin, HIGH); delay(200);
        digitalWrite(ledPin, LOW);  delay(200);

        int ready = 0;

        accelgyro.setXAccelOffset(ax_offset);
        accelgyro.setYAccelOffset(ay_offset);
        accelgyro.setZAccelOffset(az_offset);
        accelgyro.setXGyroOffset(gx_offset);
        accelgyro.setYGyroOffset(gy_offset);
        accelgyro.setZGyroOffset(gz_offset);

        meansensors();

        if (abs(mean_ax) <= acel_deadzone) ready++;
        else ax_offset = ax_offset - mean_ax / acel_deadzone;

        if (abs(mean_ay) <= acel_deadzone) ready++;
        else ay_offset = ay_offset - mean_ay / acel_deadzone;

        if (abs(16384 - mean_az) <= acel_deadzone) ready++;
        else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

        if (abs(mean_gx) <= gyro_deadzone) ready++;
        else gx_offset = gx_offset - mean_gx / (gyro_deadzone + 1);

        if (abs(mean_gy) <= gyro_deadzone) ready++;
        else gy_offset = gy_offset - mean_gy / (gyro_deadzone + 1);

        if (abs(mean_gz) <= gyro_deadzone) ready++;
        else gz_offset = gz_offset - mean_gz / (gyro_deadzone + 1);

        if (ready == 6) break;
    }

    // Turn LED off when finished
    digitalWrite(ledPin, LOW);
}


// Getters
int MPU6050Calibration::getAccelXOffset() { return ax_offset; }
int MPU6050Calibration::getAccelYOffset() { return ay_offset; }
int MPU6050Calibration::getAccelZOffset() { return az_offset; }
int MPU6050Calibration::getGyroXOffset()  { return gx_offset; }
int MPU6050Calibration::getGyroYOffset()  { return gy_offset; }
int MPU6050Calibration::getGyroZOffset()  { return gz_offset; }
