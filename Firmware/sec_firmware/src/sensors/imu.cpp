#include <Arduino.h>
#include <Wire.h>
#include "imu.h"

IMU::IMU(int SCLPIN, int SDAPIN)
    : SCLPIN(SCLPIN),
      SDAPIN(SDAPIN),
      roll_deg(0.0f),
      pitch_deg(0.0f),
      yaw_deg(0.0f),
      bias_x(0.0f),
      bias_y(0.0f),
      bias_z(0.0f),
      last_update(0)
    {}

void IMU::initialize() {
    Wire.begin(SDAPIN, SCLPIN);
    Wire.setClock(400000);

    // MPU-6500 boots in sleep, clearing PWR_MGMT_1 wakes it and selects 8MHz oscillator.
    wakeReg(MPU_POWER_REG, 0x00);
    delay(100);

    Wire.beginTransmission(MPU_I2C_ADDR);

    calibrateGyro();
    last_update = micros();
}

void IMU::calibrateGyro(int samples) {
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    int16_t ax, ay, az, gx, gy, gz;

    for (int i = 0; i < samples; i++) {
        if (readRaw(&ax, &ay, &az, &gx, &gy, &gz)) {
            sum_x += gx;
            sum_y += gy;
            sum_z += gz;
        }
        delay(3);
    }

    bias_x = sum_x / samples;
    bias_y = sum_y / samples;
    bias_z = sum_z / samples;
}

void IMU::update() {
    int16_t ax, ay, az, gx, gy, gz;

    // If readraw returns prematurely, do not update anything
    if (!readRaw(&ax, &ay, &az, &gx, &gy, &gz)) { return; }

    unsigned long now = micros();
    float dt = (now - last_update) / 1000000.0f;
    last_update = now;

    // If first call or a long stall (e.g. after recalibration), do not integrate
    if (dt <= 0.0f || dt > 0.5f) {
        return;
    }

    // Accelerometer is absolute but cannot be trusted when accelerating
    float axg = ax / MPU_ACCEL_LSB_PER_G;
    float ayg = ay / MPU_ACCEL_LSB_PER_G;
    float azg = az / MPU_ACCEL_LSB_PER_G;

    float acc_roll = atan2f(ayg, azg) * RAD_TO_DEG;
    float acc_pitch = atan2f(-axg, sqrtf(ayg * ayg + azg * azg)) * RAD_TO_DEG;

    // Gyroscope turn rate in deg/s without bias
    float rate_x = (gx - bias_x) / MPU_GYRO_LSB_PER_DPS;
    float rate_y = (gy - bias_y) / MPU_GYRO_LSB_PER_DPS;
    float rate_z = (gz - bias_z) / MPU_GYRO_LSB_PER_DPS;

    // Complementary filter for roll and pitch
    roll_deg = COMP_FILTER_ALPHA * (roll_deg + rate_x * dt) + (1.0f - COMP_FILTER_ALPHA) * acc_roll;
    pitch_deg = COMP_FILTER_ALPHA * (pitch_deg + rate_y * dt) + (1.0f - COMP_FILTER_ALPHA) * acc_pitch;

    // Raw yaw, since accelerometer has no absolute reference
    yaw_deg += rate_z * dt;

    // Change yaw range to -180 to 180 degrees
    if (yaw_deg > 180.0f){ yaw_deg -= 360.0f; }
    else if (yaw_deg < -180.0f){ yaw_deg += 360.0f; }
}

float IMU::roll() { return roll_deg * FLIP_ROLL; }

float IMU::pitch() { return pitch_deg * FLIP_PITCH; }

float IMU::yaw() { return yaw_deg * FLIP_YAW; }

void IMU::zeroYaw() { yaw_deg = 0.0f; }
 
void IMU::wakeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

bool IMU::readRaw(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) {
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(MPU_ACCEL_REG);
    // Return prematurely if no connection or incomplete data.
    if (Wire.endTransmission(false) != 0) { return false; }
    if (Wire.requestFrom((uint8_t)MPU_I2C_ADDR, (uint8_t)14) != 14) { return false; }

    // Read into a buffer to prevent race conditions.
    uint8_t buffer[14];
    for (int i = 0; i < 14; i++) {
        buffer[i] = Wire.read();
    }

    *ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    *ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    *az = (int16_t)((buffer[4] << 8) | buffer[5]);
    // b[6], b[7] are the die temperature which are unused
    *gx = (int16_t)((buffer[8] << 8) | buffer[9]);
    *gy = (int16_t)((buffer[10] << 8) | buffer[11]);
    *gz = (int16_t)((buffer[12] << 8) | buffer[13]);

    return true;
}