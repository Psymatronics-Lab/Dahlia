/**
 * @file led_matrix.h
 * @brief Basic driver for the MAX7219 8x8 LED Matrix on the SEC
 * 
 * @details Writes singleton, row, column, and range bar LED inputs to the
 *          LED matrix through an SPI connection.
 */

#ifndef LED_MATRIX_H
#define LED_MATRIX_H

/* MAX7219 register map. */
constexpr int MAX7219_REG_DIGIT0 = 0x01;
constexpr int MAX7219_REG_DECODE_MODE = 0x09;
constexpr int MAX7219_REG_INTENSITY = 0x0A;
constexpr int MAX7219_REG_SCAN_LIMIT = 0x0B;
constexpr int MAX7219_REG_SHUTDOWN = 0x0C;
constexpr int MAX7219_REG_DISPLAY_TEST = 0x0F;

constexpr int MATRIX_SPI_HZ = 8000000;   // Tolerates up to 10MHz, 8 leaves margin

class LedMatrix {
    public:
        /**
         * @brief Create a new LedMatrix object with SPI wire pins.
         * @param DINPIN GPIO pin corresponding with DIN on the matrix
         * @param CLKPIN GPIO pin corresponding with CLK on the matrix
         * @param CSPIN GPIO pin corresponding with CS on the matrix
         */
        LedMatrix(int DINPIN, int CLKPIN, int CSPIN);

        /**
         * @brief Initialize the led matrix by binding pins, starting the SPI link, and setting modes.
         * @param intensity Initial intensity of the matrix
         */
        void initialize(uint8_t intensity = 2);

        /**
         * @brief Set the brightness of the matrix, takes effect immediately.
         * @param level Intensity level from 0 to 15
         */
        void setIntensity(uint8_t level);

        /**
         * @brief Clear all on LEDs in the matrix
         */
        void clear();

        /**
         * @brief Turn an (r, c) indexed LED on or off, invalid indices are ignored.
         * @param row Row index of the LED, from 0 to 7
         * @param col Col index of the LED, from 0 to 7
         * @param on If true, turns the LED on
         */
        void setLed(int row, int col, bool on);

        /**
         * @brief Set an entire row to specific byte value.
         * @param row Row index of the LED row, from 0 to 7
         * @param bits Byte of new row value, from 0 (clear) to 255 (fill)
         */
        void setRow(int row, uint8_t bits);

        /**
         * @brief Set an entire column to specific byte value.
         * @param col Column index of the LED column, from 0 to 7
         * @param bits Byte of new row value, from 0 (clear) to 255 (fill)
         */
        void setCol(int col, uint8_t bits);

        /**
         * @brief Set an entire row to display a normalized value range.
         * @param row Row index of the LED row, from 0 to 7
         * @param value Value of the data to display in the range
         * @param high Upper bound of the range
         * @param low Lower bound of the range
         * @param centered Whether to display a centered or left/right aligned bar
         */
        void barRow(int row, float value, float low, float high, bool centered = false);

        /**
         * @brief Set an entire column to display a normalized value range.
         * @param col Column index of the LED column, from 0 to 7
         * @param value Value of the data to display in the range
         * @param high Upper bound of the range
         * @param low Lower bound of the range
         * @param centered Whether to display a centered or left/right aligned bar
         */
        void barCol(int col, float value, float low, float high, bool centered = false);

        /**
         * @brief Display the frame buffer on the LED panel
         */
        void show();

    private:
        /**
         * @brief Calculate the bit state of a matrix row/column to display a value bar
         * @param value Value of the data to display in the range
         * @param high Upper bound of the range
         * @param low Lower bound of the range
         * @param centered Whether to display a centered or left/right aligned bar
         */
        int calcBar(float value, float high, float low, bool centered);

        /**
         * @brief Clock one 16-bit address and data frame out to the matrix
         * @param reg Register address of the data
         * @param value Value of the data
         */
        void sendRegister(uint8_t reg, uint8_t value);

        int DINPIN;
        int CLKPIN;
        int CSPIN;

        uint8_t buf[8];   // Frame buffer, stores one byte per row
};

#endif /* LED_MATRIX_H */