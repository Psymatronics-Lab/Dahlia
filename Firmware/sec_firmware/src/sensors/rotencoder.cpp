#include <Arduino.h>
#include "rotencoder.h"

RotEncoder::RotEncoder(int APIN, int BPIN, int SWPIN, int reduction_factor)
    : APIN(APIN),
      BPIN(BPIN),
      SWPIN(SWPIN),
      counts(0),
      quad_state(0),
      a_reg(nullptr),
      b_reg(nullptr),
      a_shift(0),
      b_shift(0),
      switch_sample(0),
      switch_pressed(0),
      switch_last_changed(0)
    {
        if (reduction_factor < 1) { reduction_factor = 1; }
        this->reduction_factor = reduction_factor;
    }


void RotEncoder::initialize(){
    pinMode(APIN, INPUT_PULLUP);
    pinMode(BPIN, INPUT_PULLUP);
    pinMode(SWPIN, INPUT_PULLUP);

    // Assign each channel to a GPIO register and bit position.
    // The ISR then only dereferences pointers it already holds.
    a_reg = (volatile uint32_t *)((APIN < 32) ? GPIO_IN_REG : GPIO_IN1_REG);
    b_reg = (volatile uint32_t *)((BPIN < 32) ? GPIO_IN_REG : GPIO_IN1_REG);
    a_shift = (uint8_t)((APIN < 32) ? APIN : (APIN - 32));
    b_shift = (uint8_t)((BPIN < 32) ? BPIN : (BPIN - 32));

    // Seed the quadrature encoder state with its current state.
    quad_state = (uint8_t)((digitalRead(APIN) << 1) | digitalRead(BPIN));
    counts = 0;

    // By passing the encoder to the ISR, it can stay static and not follow the instance.
    attachInterruptArg(APIN, isr, this, CHANGE);
    attachInterruptArg(BPIN, isr, this, CHANGE);

    switch_sample = (digitalRead(SWPIN) == LOW);
    switch_pressed = switch_sample;
    switch_last_changed = millis();
}

void RotEncoder::update(){
    unsigned long now = millis();

    bool new_sample = (digitalRead(SWPIN) == LOW);
    // Changes restart switch clock, and until clock is above latency, then press is registered
    if (new_sample != switch_sample){
        switch_sample = new_sample;
        switch_last_changed = now;
    }

    if (now - switch_last_changed >= ENC_SWITCH_LATENCY){
        switch_pressed = switch_sample;
    }
}


int32_t RotEncoder::pos(bool reduce) {
    if (reduce){
        int32_t raw = counts;
        if (raw >= 0) {
            return raw / reduction_factor * FLIP_DIRECTION;
        }
        // Floor division on negatives to prevent rounding skipping counts
        return -((-raw + reduction_factor - 1) / reduction_factor) * FLIP_DIRECTION;
    }
    return counts * FLIP_DIRECTION;
} 

bool RotEncoder::pressed() { return switch_pressed; }

void IRAM_ATTR RotEncoder::isr(void *arg) {
    RotEncoder *self = (RotEncoder *)arg;

    uint8_t a = (uint8_t)((*self->a_reg >> self->a_shift) & 0x1);
    uint8_t b = (uint8_t)((*self->b_reg >> self->b_shift) & 0x1);

    uint8_t now  = (uint8_t)((a << 1) | b);
    uint8_t step = (uint8_t)(((self->quad_state << 2) | now) & 0x0F);

    self->quad_state = now;

    if (step == 1 || step == 7 || step == 14 || step == 8) {
        self->counts++;
    } else if (step == 2 || step == 11 || step == 13 || step == 4) {
        self->counts--;
    }
}