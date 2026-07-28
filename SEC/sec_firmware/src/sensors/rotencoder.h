/**
 * @file rotencoder.h
 * @brief Basic driver for the KY-040 rotary encoder module on the SEC
 * 
 * @details Uses a IRAM resident ISR to track quadrature encoder counts and the switch
 *          using a nonblocking update() that the main loop calls each tick. A
 *          reduction factor can be applied to adjust the raw count output at low level.
 */

#ifndef ROTENCODER_H
#define ROTENCODER_H

#include <Arduino.h>

constexpr int ENC_SWITCH_LATENCY = 5;   // Time that switch status needs to change before it is believed (ms)

constexpr int FLIP_DIRECTION = -1;   // Set to -1 to flip the positive negative output ticks of rotation

class RotEncoder {
    public:
        /**
         * @brief Constructs a new RotEncoder object using input pins.
         * @param APIN GPIO pin connected to Channel A (CLK) on the encoder.
         * @param BPIN GPIO pin connected to Channel B (DT) on the encoder.
         * @param SWPIN GPIO pin connected to SW on the encoder.
         * @param reduction_factor Integer reduction factor of raw count output.
         */
        RotEncoder(int APIN, int BPIN, int SWPIN, int reduction_factor);

        /**
         * @brief Initializes rotary encoder and assigns GPIO registers for quadrature channels.
         */
        void initialize();

        /**
         * @brief Per tick update function for switch state.
         */
        void update();

        /**
         * @brief Returns encoder counts number, potentially divided by reduction factor.
         * @param reduce If true, reduce the raw counts by the reduction factor
         */
        int32_t pos(bool reduce = true);

        /** 
         * @brief Returns switch state.
         * */
        bool pressed();

    private:

        /**
         * @brief Interrupt Service Routine on IRAM for quadrature updates.
         */
        static void IRAM_ATTR isr(void *arg);

        int APIN;
        int BPIN;
        int SWPIN;
        int reduction_factor;   // Value to divide the raw count number by

        volatile int32_t counts;   // Edge count, updated by the ISR
        volatile uint8_t quad_state;   // Current (A, B) channel state

        volatile uint32_t *a_reg;    // GPIO input register for channel A
        volatile uint32_t *b_reg;    // GPIO input register for channel B
        uint8_t a_shift;   // Bit position of channel A in a_reg
        uint8_t b_shift;   // Bit position of channel B in b_reg

        bool switch_sample;   // Raw switch status
        bool switch_pressed;   // If the switch is considered pressed or not
        unsigned long switch_last_changed;   // Last timestamp of raw switch change
};

#endif /* ROTENCODER_H */