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
 * @brief Newest joint targets from the MPU.
 */
struct CommandPacket {
    uint8_t magic;
    uint8_t sequence;
    float target_position[SPI_JOINT_COUNT];   // Radians
    float target_velocity[SPI_JOINT_COUNT];   // Radians per second
    uint8_t gripper;   // Clamp closure, 0 = open, 255 = closed
} __attribute__((packed));

/**
 * @brief Newest measured arm state from the MCU.
 */
struct FeedbackPacket {
    uint8_t magic;
    uint8_t sequence;   // Echo of the last accepted command sequence
    float current_position[SPI_JOINT_COUNT];   // Radians
    float current_velocity[SPI_JOINT_COUNT];   // Radians per second
    uint8_t status;   // bit0 = torque on, bit1 = any joint moving
} __attribute__((packed));

/**
 * @brief Full duplex SPI service, every transfer sends to TX and received from RX.
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
                return false;
            }

            memcpy(&rx, rxmsg, sizeof(RX));
            return true;
        }

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
