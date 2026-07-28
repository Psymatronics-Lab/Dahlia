#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Joystick, KY-023
constexpr int JOY_X_PIN = 4;
constexpr int JOY_Y_PIN = 5;
constexpr int JOY_SW_PIN = 6;

// Rotary Encoder, KY-040
constexpr int ENC_A_PIN = 9;
constexpr int ENC_B_PIN = 10;
constexpr int ENC_SW_PIN = 11;

// IMU, MPU6050 (I2C)
constexpr int IMU_SDA_PIN = 13;
constexpr int IMU_SCL_PIN = 14;

// Display, MAX7219 (SPI)
constexpr int DSPLY_DIN_PIN = 40;
constexpr int DSPLY_CS_PIN = 41;
constexpr int DSPLY_CLK_PIN = 42;

#endif /* CONFIG_H */