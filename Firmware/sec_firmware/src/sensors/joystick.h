/**
 * @file joystick.h
 * @brief Basic driver for the KY-023 2-axis joystick module on the SEC
 * 
 * @details Polls both ADC axes and the switch from a non-blocking update()
 *          that the main loop calls every control tick. Raw counts are
 *          sampled to reduce noise, filtered through a radial deadzone, and
 *          normalized to a signed output range.
 */

#ifndef JOYSTICK_H
#define JOYSTICK_H

constexpr int JOY_ADC_MAX = 4095;   // Highest raw reading at 12-bit ADC resolution
constexpr int JOY_AXIS_RANGE = 1000;   // Magnitude of corrected joystick output
constexpr int JOY_DEADZONE_RAD = 120;   // Raw reading radius of radial deadzone
constexpr int JOY_SWITCH_LATENCY = 5;   // Time that switch status needs to change before it is believed (ms)
constexpr int JOY_AVG_READS = 4;   // Number of reads performed per update to average ADC noise

constexpr int FLIP_X = 1;   // Set to -1 to flip the x coordinate positive and negative
constexpr int FLIP_Y = -1;   // Set to -1 to flip the y coordinate positive negative

class Joystick{
    public:
        /**
         * @brief Constructs a new Joystick object using input pins.
         * @param XPIN GPIO pin connected to VRx on the joystick, must be an ADC pin
         * @param YPIN GPIO pin connected to VRy on the joystick, must be an ADC pin
         * @param SWPIN GPIO pin connected to SW on the joystick
         */
        Joystick(int XPIN, int YPIN, int SWPIN);

        /**
         * @brief Initializes the joystick by powering the pins and calibrating the center.
         */
        void initialize();

        /**
         * @brief Calibrates the joystick center point using repeated data polls.
         * @param samples The number of ADC raw data samples to calibrate with
         */
        void calibrateCenter(int samples = 64);

        /**
         * @brief Updates the exposed polling values of the joystick using raw readings.
         */
        void update();

        /** 
         * @brief Returns normalized and centered x counts.
         * */
        int16_t x();
        /** 
         * @brief Returns normalized and centered y counts.
         * */
        int16_t y();

        /** 
         * @brief Returns switch state.
         * */
        bool pressed();

    private:

        /** 
         * @brief Average several raw reads to reduce ADC measurement noise.
         * @param pin The ADC pin to take readings from
         */
        int readAveraged(int pin);

        /**
         * @brief Scale a raw offset from center into the normalized range. Forward and backward
         * scale factors for each axis may be different.
         * @param offset The centered and scaled offset of the raw readings from the calibrated center
         * @param center The center position of the axis measured
         */
        int16_t normalizeCentered(float offset, int center);

        int XPIN;
        int YPIN;
        int SWPIN;

        int center_x;   // Calibrated center value of x axis
        int center_y;   // Calibrated center value of y axis
        int16_t poll_x;   // Normalized and centered output x value
        int16_t poll_y;   // Normalized and centered output y value

        bool switch_sample;   // Raw switch status
        bool switch_pressed;   // If the switch is considered pressed or not
        unsigned long switch_last_changed;   // Last timestamp of raw switch change
};

#endif /* JOYSTICK_H */