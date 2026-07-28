#include <Arduino.h>
#include <NimBLEDevice.h>
#include "ble.h"

BLEPublisher::BLEPublisher()
    : state_char(nullptr),
      seq(0)
    {}

void BLEPublisher::initialize(){
    NimBLEDevice::init("SEC Controller");
    NimBLEServer* server = NimBLEDevice::createServer();
    NimBLEService* service = server->createService(SERVICE_UUID);

    state_char = service->createCharacteristic(STATE_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    service->start();

    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
    advertising->setName("SEC Controller");
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->start();
}

void BLEPublisher::publish_data(int32_t enc_pos, int16_t joy_x, int16_t joy_y, float imu_roll,
                    float imu_pitch, float imu_yaw, bool joy_sw, bool enc_sw){
    SECState state;
    state.seq = seq;
    state.timestamp = millis();
    state.enc_pos = enc_pos;
    state.joy_x = joy_x;
    state.joy_y = joy_y;
    state.imu_roll = (int16_t)lround(imu_roll);
    state.imu_pitch = (int16_t)lround(imu_pitch);
    state.imu_yaw = (int16_t)lround(imu_yaw);
    state.joy_pressed = (uint8_t)joy_sw;
    state.enc_pressed = (uint8_t)enc_sw;
    publish((uint8_t*)&state, sizeof(state));
    seq = seq + 1;
}

bool BLEPublisher::connected(){
    return NimBLEDevice::getServer()->getConnectedCount() > 0;
}

void BLEPublisher::publish(const uint8_t* state_pointer, size_t size){
    state_char->setValue(state_pointer, size);
    state_char->notify();
}