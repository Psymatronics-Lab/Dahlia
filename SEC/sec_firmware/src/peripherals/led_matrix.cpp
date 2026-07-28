#include <Arduino.h>
#include <SPI.h>
#include "led_matrix.h"

LedMatrix::LedMatrix(int DINPIN, int CLKPIN, int CSPIN)
    : DINPIN(DINPIN),
      CLKPIN(CLKPIN),
      CSPIN(CSPIN)
    {
        for (int r = 0; r < 8; r++) { buf[r] = 0; }
    }

void LedMatrix::initialize(uint8_t intensity){
    pinMode(CSPIN, OUTPUT);
    digitalWrite(CSPIN, HIGH);

    // MISO is -1 because MAX7219 has no feedback, SS is -1 since CS is driven manually
    SPI.begin(CLKPIN, -1, DINPIN, -1);

    sendRegister(MAX7219_REG_DISPLAY_TEST, 0x00);   // Turn test mode off
    sendRegister(MAX7219_REG_DECODE_MODE, 0x00);   // Use raw LEDs, no BCD
    sendRegister(MAX7219_REG_SCAN_LIMIT, 0x07);   // Scan all eight rows
    setIntensity(intensity);
    sendRegister(MAX7219_REG_SHUTDOWN, 0x01);  // Leave shutdown

    clear();
    show();
}

void LedMatrix::setIntensity(uint8_t level){
    if (level > 0x0F) { level = 0x0F; }
    sendRegister(MAX7219_REG_INTENSITY, level);
}

void LedMatrix::clear(){
    for (int r = 0; r < 8; r++) {
        buf[r] = 0;
    }
}

void LedMatrix::setLed(int row, int col, bool on) {
    // Filter out invalid indices
    if (row < 0 || row >= 8 || col < 0 || col >= 8) { return; }

    if (on) { buf[row] |= (uint8_t)(1 << col); }
    else { buf[row] &= (uint8_t)~(1 << col); }
}

void LedMatrix::setRow(int row, uint8_t bits){
    if (row < 0 || row >= 8) { return; }
    buf[row] = bits;
}

void LedMatrix::setCol(int col, uint8_t bits){
    if (col < 0 || col >= 8) { return; }
    for (int r = 0; r < 8; r++) {
        if (bits & (1 << r)) { buf[r] |= (uint8_t)(1 << col); }
        else { buf[r] &= (uint8_t)~(1 << col); }
    }
}

void LedMatrix::barRow(int row, float value, float low, float high, bool centered){
    setRow(row, calcBar(value, low, high, centered));
}

void LedMatrix::barCol(int col, float value, float low, float high, bool centered){
    setCol(col, calcBar(value, low, high, centered));
}

void LedMatrix::show(){
    for (int r = 0; r < 8; r++) {
        sendRegister((uint8_t)(MAX7219_REG_DIGIT0 + r), buf[r]);
    }
}

int LedMatrix::calcBar(float value, float high, float low, bool centered){
    float span = high - low;
    if (span == 0.0f) { return 0; }

    if (centered){
        float normalized = 2.0f * (value - low) / span - 1.0f;
        normalized = constrain(normalized, -1.0f, 1.0f);
        int n = lroundf(fabsf(normalized) * 4.0f);

        uint8_t lit = 0;
        if (normalized > 0.0f) {
            for (int i = 0; i < n; i++) {
                lit |= (1 << (3 - i));
            }
        }
        else if (normalized < 0.0f) {
            for (int i = 0; i < n; i++) {
                lit |= (1 << (4 + i));
            }
        }
        return lit;
    }

    float normalized = (value - low) / span;
    int lit = lroundf(normalized * 8.0f);

    if (lit < 0) { lit = 0; }
    else if (lit > 8) { lit = 8; }

    // Converts to correct display format
    if (lit == 8){ return 0xFF; }
    return (1 << lit) - 1;
}

void LedMatrix::sendRegister(uint8_t reg, uint8_t value){
    SPI.beginTransaction(SPISettings(MATRIX_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(CSPIN, LOW);
    SPI.transfer(reg);
    SPI.transfer(value);
    digitalWrite(CSPIN, HIGH);
    SPI.endTransaction();
}
