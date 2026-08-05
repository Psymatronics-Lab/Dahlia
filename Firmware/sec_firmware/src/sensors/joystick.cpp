#include <Arduino.h>
#include "joystick.h"

Joystick::Joystick(int XPIN, int YPIN, int SWPIN)
    : XPIN(XPIN),
      YPIN(YPIN),
      SWPIN(SWPIN),
      center_x(0),
      center_y(0),
      poll_x(0),
      poll_y(0),
      switch_sample(false),
      switch_pressed(false),
      switch_last_changed(0)
    {}

void Joystick::initialize(){
    analogReadResolution(12);
    analogSetPinAttenuation(XPIN, ADC_11db);
    analogSetPinAttenuation(YPIN, ADC_11db);

    pinMode(SWPIN, INPUT_PULLUP);

    switch_sample = (digitalRead(SWPIN) == LOW);
    switch_pressed = switch_sample;
    switch_last_changed = millis();

    calibrateCenter();
}

void Joystick::calibrateCenter(int samples) {
    long sum_x = 0;
    long sum_y = 0;

    for (int i = 0; i < samples; i++) {
        sum_x += analogRead(XPIN);
        sum_y += analogRead(YPIN);
        delayMicroseconds(200);
    }

    center_x = sum_x / samples;
    center_y = sum_y / samples;

    poll_x = 0;
    poll_y = 0;
}

void Joystick::update() {
    unsigned long now = millis();

    int raw_x = readAveraged(XPIN);
    int raw_y = readAveraged(YPIN);

    float off_x = raw_x - center_x;
    float off_y = raw_y - center_y;
    float distance = sqrtf(off_x * off_x + off_y * off_y);

    if (distance <= JOY_DEADZONE_RAD){
        off_x = 0.0f;
        off_y = 0.0f;
    }
    else{
        // Prevent jumping power on edge of deadzone
        float scale = (distance - JOY_DEADZONE_RAD) / distance;
        off_x = off_x * scale;
        off_y = off_y * scale;
    }

    poll_x = normalizeCentered(off_x, center_x);
    poll_y = normalizeCentered(off_y, center_y);

    bool new_sample = (digitalRead(SWPIN) == LOW);
    // Changes restart switch clock, and until clock is above latency, then press is registered
    if (new_sample != switch_sample){
        switch_sample = new_sample;
        switch_last_changed = now;
    }

    if (now - switch_last_changed >= JOY_SWITCH_LATENCY){
        switch_pressed = switch_sample;
    }
}

int16_t Joystick::x() { return poll_x * FLIP_X; }

int16_t Joystick::y() { return poll_y * FLIP_Y; }

bool Joystick::pressed() { return switch_pressed; }

int Joystick::readAveraged(int pin){
    long sum = 0;
    for (int i = 0; i < JOY_AVG_READS; i++) {
        sum += analogRead(pin);
    }
    return sum / JOY_AVG_READS;
}

int16_t Joystick::normalizeCentered(float offset, int center) {
    if (offset == 0.0f) {
        return 0;
    }

    float span;
    if (offset > 0.0f) {
        span = JOY_ADC_MAX - center;
    }
    else {
        span = center;
    }

    float scaled = offset * JOY_AXIS_RANGE / span;

    if (scaled > JOY_AXIS_RANGE) {
        scaled = JOY_AXIS_RANGE;
    }
    if (scaled < -JOY_AXIS_RANGE) {
        scaled = -JOY_AXIS_RANGE;
    }

    return (int16_t)lroundf(scaled);
}