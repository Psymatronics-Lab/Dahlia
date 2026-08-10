#ifndef SPI_SERVICE_H
#define SPI_SERVICE_H

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/spi.h>

constexpr int SPI_JOINT_COUNT = 5;
constexpr uint8_t COMMAND_MAGIC = 0xA5;   // MPU to MCU
constexpr uint8_t FEEDBACK_MAGIC = 0x5A;   // MCU to MPU

/**
 * @brief Mode the MCU uses to interpret the given commands.
 */
enum ControlMode : uint8_t {
    MODE_HOLD = 0,    // Ignore the targets and hold the present pose
    MODE_ANGLE = 1,   // Follow target_position / target_velocity, in radians
    MODE_RAW = 2      // Follow raw_position / raw_velocity / raw_acceleration, in ticks
};

// Set in CommandPacket::torque indicate torque command validity
constexpr uint8_t TORQUE_APPLY = 0x80;

// FeedbackPacket::status bits
constexpr uint8_t STATUS_TORQUE = 0x01;   // At least one joint is holding
constexpr uint8_t STATUS_MOVING = 0x02;   // At least one joint is moving
constexpr uint8_t STATUS_STALE = 0x04;    // No fresh command within the watchdog window

/**
 * @brief Newest joint targets from the MPU.
 */
struct CommandPacket {
    uint8_t magic;
    uint8_t sequence;
    uint8_t mode;     // ControlMode
    uint8_t torque;   // bits 0..4 per joint, only honoured when TORQUE_APPLY is set

    float target_position[SPI_JOINT_COUNT];   // Radians
    float target_velocity[SPI_JOINT_COUNT];   // Radians per second

    int16_t raw_position[SPI_JOINT_COUNT];   // Servo ticks, 0 is the calibrated centre
    int16_t raw_velocity[SPI_JOINT_COUNT];   // Ticks per second, also caps the slew rate
    uint8_t raw_acceleration[SPI_JOINT_COUNT];   // Servo acceleration units, 0..254

    uint8_t gripper;   // Clamp closure, 0 = open, 255 = closed
} __attribute__((packed));

/**
 * @brief Newest measured arm state from the MCU.
 */
struct FeedbackPacket {
    uint8_t magic;
    uint8_t sequence;   // Echo of the last accepted command sequence
    uint8_t mode;       // The mode actually in effect, which is MODE_HOLD while stale
    uint8_t status;     // STATUS_* bits

    float current_position[SPI_JOINT_COUNT];   // Radians
    float current_velocity[SPI_JOINT_COUNT];   // Radians per second

    int16_t raw_position[SPI_JOINT_COUNT];   // Servo ticks
    int16_t raw_velocity[SPI_JOINT_COUNT];   // Ticks per second
    int16_t raw_load[SPI_JOINT_COUNT];       // Signed load, servo units

    uint8_t voltage[SPI_JOINT_COUNT];       // 0.1 V per count
    uint8_t temperature[SPI_JOINT_COUNT];   // Degrees Celsius

    uint8_t torque;        // Actual per-joint torque state, bits 0..4
    uint8_t comm_errors;   // Bit i set if joint i missed the last telemetry read
    uint8_t gripper;       // Echo of the commanded closure; the clamp has no encoder

    // Servo loop iteration count, constant iterations means wedged MCU
    uint16_t loop_count;
} __attribute__((packed));

// MPU packs structs, so layout changes must be mirrored in `bricks/spi_service/spi_service.py`.
static_assert(sizeof(CommandPacket) == 5 + 13 * SPI_JOINT_COUNT,
              "CommandPacket layout changed: update PACKET_FORMAT in spi_service.py");
static_assert(sizeof(FeedbackPacket) == 9 + 16 * SPI_JOINT_COUNT,
              "FeedbackPacket layout changed: update FEEDBACK_FORMAT in spi_service.py");

/**
 * @brief Full duplex SPI service, every transfer sends to TX and receives from RX.
 */
template <typename TX, typename RX>
class SPIService {
    public:
        TX tx;
        RX rx;

        SPIService(){
            tx = {};
            rx = {};
            spi_cfg = {};

            spi_cfg.frequency = 1000000;
            spi_cfg.operation = SPI_WORD_SET(8) | SPI_OP_MODE_SLAVE;

            tx_buf.buf = txmsg;
            tx_buf.len = FRAME_SIZE;
            tx_bufs.buffers = &tx_buf;
            tx_bufs.count = 1;

            rx_buf.buf = rxmsg;
            rx_buf.len = FRAME_SIZE;
            rx_bufs.buffers = &rx_buf;
            rx_bufs.count = 1;
        }

        int initialize(){ return device_init(spi_device); }

        /**
         * @brief Exchange tx for rx. Blocks until the host clocks out a frame.
         * @return True if a frame was exchanged.
         */
        bool transfer(){
            memset(txmsg, 0, FRAME_SIZE);
            memcpy(txmsg, &tx, sizeof(TX));

            if (spi_transceive(spi_device, &spi_cfg, &tx_bufs, &rx_bufs) < 0){
                // Clear since data is kept async by host and client, and verified by magic
                rx = {};
                return false;
            }

            memcpy(&rx, rxmsg, sizeof(RX));
            return true;
        }

        /** @brief Bytes exchanged per transfer. The host must clock exactly this many. */
        static constexpr size_t frame_size(){ return FRAME_SIZE; }

    private:
        static constexpr size_t FRAME_SIZE = (sizeof(TX) > sizeof(RX)) ? sizeof(TX) : sizeof(RX);

        const struct device *const spi_device = DEVICE_DT_GET(
            DT_BUS(DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_spi_slave)));
        struct spi_config spi_cfg;

        uint8_t txmsg[FRAME_SIZE];
        struct spi_buf tx_buf;
        struct spi_buf_set tx_bufs;

        uint8_t rxmsg[FRAME_SIZE];
        struct spi_buf rx_buf;
        struct spi_buf_set rx_bufs;
};

#endif
