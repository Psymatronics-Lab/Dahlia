/**
 * @file imu.h
 * @brief Basic driver for the MPU-6500 6-axis IMU module on the SEC
 * 
 * @details Extracts gyroscope and accelerometer data from the sensor,
 *          using a complementary filter to adjust roll and pitch values. Uses
 *          a non-blocking update() to adjust state each tick in the main loop.
 */

#ifndef IMU_H
#define IMU_H

constexpr int MPU_I2C_ADDR = 0x68;  // I2C address with AD0 low, 0x69 if AD0 is high

constexpr int MPU_POWER_REG = 0x6B;   // GPIO register to wake the imu
constexpr int MPU_ACCEL_REG = 0x3B;   // GPIO register to collect acceleration / gyro reads from
constexpr float MPU_ACCEL_LSB_PER_G = 16384.0f;   // Accelerometer sensitivity at default 2g full scale
constexpr float MPU_GYRO_LSB_PER_DPS = 131.0f;   // Gyroscope sensitivity at default 250deg/s full scale

/**
 * Complementary filter weight on the gyro and accelerometer between 0 and 1
 * Higher trusts the gyroscope for a longer time but potentially allows drift persistence
 */
constexpr float COMP_FILTER_ALPHA = 0.98f;

constexpr int FLIP_ROLL = -1;   // Set to -1 to flip positive and negative degrees of roll
constexpr int FLIP_PITCH = 1;   // Set to -1 to flip positive and negative degrees of pitch
constexpr int FLIP_YAW = 1;   // Set to -1 to flip positive and negative degrees of yaw

class IMU{
    public:
        /**
         * @brief Create a new IMU object given I2C pins.
         * @param SCLPIN GPIO pin connected to SCL on the sensor
         * @param SDAPIN GPIO pin connected to SDA on the sensor
         */
        IMU(int SCLPIN, int SDAPIN);

        /**
         * @brief Starts the I2C bus, wakes the sensor, and conducts calibration, takes around 1.0s.
         */
        void initialize();

        /**
         * @brief Calibrate the gyroscope by sampling and setting bias.
         * @param samples Number of samples to take for calibration
         */
        void calibrateGyro(int samples = 200);

        /**
         * @brief Read the IMU and update the rotation estimate.
         */
        void update();

        /** 
         * @brief Roll in degrees, gravity-referenced, does not drift.
         * */
        float roll();

        /**
         * @brief Pitch in degrees, gravity-referenced, does not drift.
         * */
        float pitch();

        /**
         * @brief Yaw in degrees, drifts without bound, relative to last zeroYaw() only.
         */
        float yaw();

        /**
         * @brief Set the current yaw as the zeroed position.
         * */
        void zeroYaw();

    private:
        /**
         * @brief Wakes the IMU by writing to the associated register.
         */
        void wakeReg(uint8_t reg, uint8_t value);

        /**
         * @brief Burst read the acceleration and gyroscope registers.
         */
        bool readRaw(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);

        int SDAPIN;
        int SCLPIN;

        float roll_deg;   // Fused roll estimate
        float pitch_deg;   // Fused pitch estimate
        float yaw_deg;   // Gyroscope-only yaw estimate

        float bias_x;   // Gyroscope zero-rate offset in raw counts
        float bias_y;
        float bias_z;

        unsigned long last_update;   // Time of last update call in microseconds.
};

#endif /* IMU_H */