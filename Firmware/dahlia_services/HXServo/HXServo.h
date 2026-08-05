/**
 * @file    HXServo.h
 * @brief   Arduino driver for Hiwonder Magnetic-Encoder intelligent bus servos
 *          (HX "HM" family: HX-10HM, HX-30HM, HX-65HM, ...).
 *
 * @details Speaks the native 0xFF-header servo protocol, so it drives the
 *          servos through a transparent bus adapter such as the Hiwonder
 *          BusLinker V3.0, or directly on a half-duplex TTL bus with suitable
 *          direction-control hardware. See README.md for wiring.
 *
 *          Ported from Hiwonder's ESP32-only "HX_30HM" reference to standard
 *          Arduino (AVR / STM32 / etc.). The ESP32-specific serial calls have
 *          been removed and serial start-up moved into begin(). See the
 *          "Changes from the original" section of the README.
 *
 * @version 2.0  (Arduino port)
 * @note    Original protocol implementation (c) 2025 Hiwonder ("Min").
 */
#ifndef HXSERVO_H
#define HXSERVO_H

#include <Arduino.h>
#include "HXServo_Registers.h"

/**
 * @def HX_DEBUG
 * @brief Set to 1 to print every TX/RX frame (in hex) to the debug stream.
 *        Very useful during first bring-up; set back to 0 for normal running,
 *        as the prints slow down high-rate control loops.
 */
#ifndef HX_DEBUG
#define HX_DEBUG 0
#endif

/**
 * @def HX_DEBUG_STREAM
 * @brief Stream the debug frames are printed to (defaults to USB Serial).
 */
#ifndef HX_DEBUG_STREAM
#define HX_DEBUG_STREAM Serial
#endif

/** Largest parameter payload handled by the RX packet buffer. */
#define HX_MAX_FRAME_SIZE 250

/**
 * @brief Servo status / result returned by every command.
 *
 * @c id holds the responding servo ID (or the ID a command was sent to).
 * The bit-field decodes hardware error flags reported by the servo, plus two
 * driver-side transport flags (bit_tx / bit_rx) raised when a frame could not
 * be sent or no valid reply arrived. A fully successful call has
 * error_byte == 0.
 */
typedef struct {
    uint8_t id;                         /**< Servo ID that responded          */
    union {
        uint8_t error_byte;             /**< All flags as one byte            */
        struct {
            uint8_t bit_voltage  : 1;   /**< Supply voltage out of range      */
            uint8_t bit_sensor   : 1;   /**< Angle/magnetic sensor error      */
            uint8_t bit_overheat : 1;   /**< Over-temperature                 */
            uint8_t bit_current  : 1;   /**< Over-current                     */
            uint8_t bit_angle    : 1;   /**< Commanded angle out of range     */
            uint8_t bit_overload : 1;   /**< Sustained overload               */
            uint8_t bit_tx       : 1;   /**< Driver: frame failed to send     */
            uint8_t bit_rx       : 1;   /**< Driver: no/invalid reply (timeout)*/
        } error_bits;
    };
} ServoStatus_t;

/** Internal state machine used while parsing an incoming frame. */
typedef enum {
    PACKET_HEADER_1 = 0,
    PACKET_HEADER_2,
    PACKET_ID,
    PACKET_DATA_LENGTH,
    PACKET_CMD,
    PACKET_PARAMETERS,
    PACKET_CHECKSUM,
    PACKET_FINISH
} PacketStatus;

/** Raw framed packet, overlaid on a flat byte buffer for checksum walking. */
#pragma pack(1)
typedef struct {
    uint8_t header_1;
    uint8_t header_2;
    union {
        struct {
            uint8_t id;
            uint8_t length;
            uint8_t cmd;
            uint8_t args[HX_MAX_FRAME_SIZE];
        } elements;
        uint8_t data_raw[HX_MAX_FRAME_SIZE + 3];
    };
} PacketTypeDef;
#pragma pack()

/**
 * @class HXServo
 * @brief Bus master for Hiwonder magnetic-encoder (HX "HM") serial bus servos.
 *
 * Bind the driver to the hardware serial port that is wired to the servo bus
 * (or to the BusLinker), then call begin() from setup():
 * @code
 *   HXServo servo(Serial1);        // 1,000,000 bps by default
 *   void setup() { servo.begin(); servo.ping(BROADCAST_ID); }
 * @endcode
 */
class HXServo {
public:
    /**
     * @brief Construct a driver bound to a hardware serial port.
     * @param serial Hardware UART wired to the servo bus / BusLinker.
     * @param baud   Bus baud rate (must match the servos and adapter).
     *               Factory default for HX-HM servos is 1,000,000.
     * @note Does NOT open the port -- call begin() from setup().
     */
    HXServo(HardwareSerial &serial, uint32_t baud = 1000000);

    /** @brief Open the serial port at the configured baud rate (8N1). */
    void begin();

    /** @brief Open the serial port, overriding the baud rate. */
    void begin(uint32_t baud);

    /**
     * @brief Ping a servo to check that it is online.
     * @param id Servo ID, or BROADCAST_ID (0xFE) to discover a single servo.
     * @return Status; on success @c id holds the responding servo's ID.
     */
    ServoStatus_t ping(uint8_t id);

    /**
     * @brief Write raw bytes to a register (synchronous, waits for ACK).
     * @param id       Servo ID (BROADCAST_ID skips the ACK wait).
     * @param addr     First register address.
     * @param data     Bytes to write.
     * @param data_len Number of bytes.
     */
    ServoStatus_t general_write(uint8_t id, uint8_t addr, uint8_t *data, uint8_t data_len);

    /**
     * @brief Read raw bytes from a register.
     * @param id       Servo ID.
     * @param addr     First register address.
     * @param data     Destination buffer (>= data_len bytes).
     * @param data_len Number of bytes to read.
     */
    ServoStatus_t general_read(uint8_t id, uint8_t addr, uint8_t *data, uint8_t data_len);

    /**
     * @brief Buffered (deferred) register write; applied later by reg_action().
     * @see reg_action
     */
    ServoStatus_t reg_write(uint8_t id, uint8_t addr, uint8_t *data, uint8_t data_len);

    /**
     * @brief Trigger execution of a previously buffered reg_write().
     * @param id Servo ID, or BROADCAST_ID to trigger all servos simultaneously.
     */
    ServoStatus_t reg_action(uint8_t id);

    /**
     * @brief Synchronous write of a register block to multiple servos at once.
     * @param addr          First register address.
     * @param data          Interleaved payload:
     *                      [id1, p1..pn, id2, p1..pn, ...].
     * @param data_len      Total length of @p data.
     * @param parameter_len Bytes written per servo (excluding the ID byte).
     * @note Broadcast; servos do not reply.
     */
    ServoStatus_t sync_write(uint8_t addr, uint8_t *data, uint8_t data_len, uint8_t parameter_len);

    /**
     * @brief Synchronous read of the same register block from multiple servos.
     * @param addr     First register address.
     * @param byte_num Bytes to read from each servo.
     * @param id       Array of servo IDs to read.
     * @param id_num   Number of IDs.
     * @param data     Destination, laid out as id_num * byte_num bytes.
     */
    ServoStatus_t sync_read(uint8_t addr, uint8_t byte_num, uint8_t *id, uint8_t id_num, uint8_t *data);

    /** @brief Enable torque output (servo holds position / is powered). */
    ServoStatus_t enable_torque(uint8_t id);

    /** @brief Disable torque output (servo goes limp / free-wheeling). */
    ServoStatus_t disable_torque(uint8_t id);

    /**
     * @brief Calibrate the current shaft position as the new center (2048).
     * @note Writes the special value 128 to the torque-enable register.
     */
    ServoStatus_t cali_pos(uint8_t id);

    /**
     * @brief Select the operating mode.
     * @param mode POSITION_MODE, CLOSED_LOOP_MOTOR_MODE, or OPEN_LOOP_MOTOR_MODE.
     */
    ServoStatus_t select_mode(uint8_t id, uint8_t mode);

    /**
     * @brief Set target position (position mode).
     * @param pos Target, clamped to [-30719, 30719]. 0 is center.
     */
    ServoStatus_t write_pos(uint8_t id, int16_t pos);

    /**
     * @brief Set the stored position offset (zero trim).
     * @param offset Clamped to [-2047, 2047].
     */
    ServoStatus_t write_pos_offset(uint8_t id, int16_t offset);

    /**
     * @brief Set acceleration.
     * @param acc Clamped to [0, 254].
     */
    ServoStatus_t write_acc(uint8_t id, uint8_t acc);

    /**
     * @brief Set target speed (closed-loop speed mode).
     * @param speed Clamped to [-3400, 3400]; sign selects direction.
     */
    ServoStatus_t write_speed(uint8_t id, int16_t speed);

    /**
     * @brief One-shot position move with acceleration and speed limit.
     * @param acc   Acceleration [0, 254].
     * @param speed Speed limit [-3400, 3400].
     * @param pos   Target position [-30719, 30719].
     */
    ServoStatus_t write_pos_ex(uint8_t id, uint8_t acc, int16_t speed, int16_t pos);

    /**
     * @brief Set PWM duty (open-loop motor mode).
     * @param speed Clamped to [-1000, 1000]; sign selects direction.
     */
    ServoStatus_t write_pwm_speed(uint8_t id, int16_t speed);

    /**
     * @brief Set the maximum torque limit.
     * @param torque Clamped to [0, 1000].
     */
    ServoStatus_t write_max_torque(uint8_t id, uint16_t torque);

    /**
     * @brief Buffered version of write_pos_ex(); applied by reg_action().
     * @see reg_action
     */
    ServoStatus_t write_reg_pos_ex(uint8_t id, uint8_t acc, int16_t speed, int16_t pos);

    /**
     * @brief Synchronously command position+acc+speed to several servos.
     * @param data   Array of rows, each {id, acc, speed, pos}.
     * @param id_num Number of rows (max 30).
     */
    ServoStatus_t sync_write_pos_ex(int16_t (*data)[4], uint8_t id_num);

    /**
     * @brief Synchronously read telemetry from several servos.
     * @param id     Array of servo IDs.
     * @param id_num Number of IDs.
     * @param data   Output rows, each {position, speed, load, voltage, temp}.
     */
    ServoStatus_t sync_read_cur_pos_ex(uint8_t *id, uint8_t id_num, int16_t (*data)[5]);

    /** @brief Read the stored position offset into @p offset. */
    ServoStatus_t read_pos_offset(uint8_t id, int16_t *offset);

    /** @brief Read the present position into @p pos. */
    ServoStatus_t read_pos(uint8_t id, int16_t *pos);

    /** @brief Read the present speed into @p speed. */
    ServoStatus_t read_speed(uint8_t id, int16_t *speed);

    /** @brief Read present position and speed in a single transaction. */
    ServoStatus_t read_pos_speed(uint8_t id, int16_t *pos, int16_t *speed);

    /** @brief Read present temperature (degrees C) into @p temp. */
    ServoStatus_t read_temperature(uint8_t id, uint8_t *temp);

    /** @brief Read present supply voltage (0.1 V per count) into @p vol. */
    ServoStatus_t read_voltage(uint8_t id, uint8_t *vol);

    /** @brief Read present current into @p cur. */
    ServoStatus_t read_current(uint8_t id, uint16_t *cur);

    /** @brief Read present load into @p load. */
    ServoStatus_t read_load(uint8_t id, int16_t *load);

    /** @brief Read moving status (0 = stopped, 1 = moving) into @p status. */
    ServoStatus_t read_moving_status(uint8_t id, uint8_t *status);

private:
    HardwareSerial *uart;   /**< Bus serial port. */
    uint32_t baudrate;      /**< Configured baud rate. */

    uint8_t      rx_skip;         /**< When set, do not wait for a reply. */
    uint8_t      endianness;      /**< 0 = little-endian on the wire. */
    uint8_t      rx_frame_length; /**< Bytes expected in the next reply. */
    uint32_t     rx_timeout;      /**< Reply timeout, milliseconds. */
    PacketStatus rx_status;       /**< Parser state. */
    PacketTypeDef rx_packet;      /**< Last parsed reply. */

    uint8_t  unpack(void);
    void     word2bytes(uint16_t word, uint8_t *bytes_l, uint8_t *bytes_h);
    uint16_t bytes2word(uint8_t *bytes_l, uint8_t *bytes_h);
    uint8_t  data_check(const uint8_t buf[], uint8_t len);
    uint8_t  tx_frame_write(uint8_t id, uint8_t cmd, const uint8_t *data, uint8_t data_len);
    ServoStatus_t ack(void);
};

#endif /* HXSERVO_H */
