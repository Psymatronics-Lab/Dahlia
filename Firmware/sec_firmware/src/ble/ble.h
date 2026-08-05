#ifndef BLE_H
#define BLE_H

#include <NimBLEDevice.h>

#define SERVICE_UUID  "12345678-1234-5678-1234-123456789abc"
#define STATE_CHAR_UUID  "87654321-4321-6789-4321-cba987654321"

struct SECState {
    uint32_t seq;
    uint32_t timestamp;

    int32_t enc_pos;

    int16_t joy_x;
    int16_t joy_y;

    int16_t imu_roll;
    int16_t imu_yaw;
    int16_t imu_pitch;

    uint8_t joy_pressed;
    uint8_t enc_pressed;
} __attribute__((packed));

class BLEPublisher{
    public:
        /**
         * @brief Create a new BLEPublisher instance.
         */
        BLEPublisher();

        /**
         * @brief Initialize the NimBLE broadcaster.
         */
        void initialize();

        /**
         * @brief Publish controller polled fields of an SECState.
         * @param enc_pos Encoder position
         * @param joy_x Joystick x position
         * @param joy_y Joystick y position
         * @param imu_roll IMU roll degrees
         * @param imu_pitch IMU pitch degrees
         * @param imu_yaw IMU yaw degrees
         * @param joy_sw Joystick switch state
         * @param enc_sw Encoder switch state
         */
        void publish_data(int32_t enc_pos, int16_t joy_x, int16_t joy_y, float imu_roll,
                          float imu_pitch, float imu_yaw, bool joy_sw, bool enc_sw);

        /**
         * @brief Whether if one or more devices is connected to the controller.
         */
        bool connected();

    private:
        /**
         * @brief Publish an SECState on NimBLE, using notify().
         * @param state_pointer Pointer to a SECState struct
         * @param size Size of the SECState struct
         */
        void publish(const uint8_t* state_pointer, size_t size);

        NimBLECharacteristic* state_char;   // NimBLE state characteristic object
        uint32_t seq;   // Packet sequence number, for ease of use
};

#endif /* BLE_H */