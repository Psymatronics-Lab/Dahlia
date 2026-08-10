/**
 * @file    HXServo.cpp
 * @brief   Implementation of the HXServo Arduino driver.
 *
 * Ported from Hiwonder's ESP32-only "HX_30HM" reference implementation.
 * Protocol logic is unchanged; the port removes ESP32-specific serial calls,
 * moves port start-up into begin(), and fixes the goal-speed field in the
 * "_ex" position helpers. See README.md > "Changes from the original".
 */
#include "HXServo.h"

/* Convert a signed host value to the servo's sign-magnitude wire form: the
 * magnitude in the low bits and the sign in @c bit. */
#define MASK_HOST(x, bit)  ((x) < 0 ? (-(x)) | ((1U) << (bit)) : (x))

/* Inverse of MASK_HOST: recover a signed value from sign-magnitude form. */
#define MASK_SERVO(x, bit) (((x) & ((1U) << (bit))) != 0 ? -((x) & ~((1U) << (bit))) : (x))

/* Clamp x to [min, max]. */
#define LIMIT(x, min, max) (((x) < (min)) ? (min) : ((x) > (max)) ? (max) : (x))

/* -------------------------------------------------------------------------
 * Construction / start-up
 * ---------------------------------------------------------------------- */
HXServo::HXServo(HardwareSerial &serial, uint32_t baud)
{
    uart       = &serial;
    baudrate   = baud;
    rx_skip    = 1;
    rx_timeout = 20;   /* ms */
    endianness = 0;    /* little-endian on the wire */
    rx_status  = PACKET_HEADER_1;
}

void HXServo::begin()
{
    uart->begin(baudrate, SERIAL_8N1);
}

void HXServo::begin(uint32_t baud)
{
    baudrate = baud;
    begin();
}

/* -------------------------------------------------------------------------
 * Byte / word helpers
 * ---------------------------------------------------------------------- */
void HXServo::word2bytes(uint16_t word, uint8_t *bytes_l, uint8_t *bytes_h)
{
    if (endianness) {
        *bytes_l = (word >> 8);
        *bytes_h = (word & 0xff);
    } else {
        *bytes_h = (word >> 8);
        *bytes_l = (word & 0xff);
    }
}

uint16_t HXServo::bytes2word(uint8_t *bytes_l, uint8_t *bytes_h)
{
    uint16_t word;

    if (endianness) {
        word = *bytes_l;
        word <<= 8;
        word |= *bytes_h;
    } else {
        word = *bytes_h;
        word <<= 8;
        word |= *bytes_l;
    }
    return word;
}

/* Checksum: ~(sum of every byte after the two header bytes). */
uint8_t HXServo::data_check(const uint8_t buf[], uint8_t len)
{
    uint16_t temp = 0;
    for (int i = 2; i < len; ++i) {
        temp += buf[i];
    }
    return (uint8_t)(~temp);
}

/* -------------------------------------------------------------------------
 * Frame parsing (receive)
 * ---------------------------------------------------------------------- */
uint8_t HXServo::unpack()
{
    uint8_t error;
    uint8_t check_value;

    rx_status = PACKET_HEADER_1;

    while (uart->available()) {
        switch (rx_status) {
            case PACKET_HEADER_1:
                rx_packet.header_1 = uart->read();
                rx_status = rx_packet.header_1 == FRAME_HEADER_1 ? PACKET_HEADER_2 : PACKET_HEADER_1;
                break;

            case PACKET_HEADER_2:
                rx_packet.header_2 = uart->read();
                rx_status = rx_packet.header_2 == FRAME_HEADER_2 ? PACKET_ID : PACKET_HEADER_1;
                break;

            case PACKET_ID:
                rx_packet.elements.id = uart->read();
                rx_status = rx_packet.elements.id <= BROADCAST_ID ? PACKET_DATA_LENGTH : PACKET_HEADER_1;
                break;

            case PACKET_DATA_LENGTH:
                rx_packet.elements.length = uart->read();
                rx_status = rx_packet.elements.length <= HX_MAX_FRAME_SIZE + 1 ? PACKET_CMD : PACKET_HEADER_1;
                break;

            case PACKET_CMD:
                rx_packet.elements.cmd = uart->read();
                rx_status = rx_packet.elements.cmd <= CMD_SYNC_READ ? PACKET_PARAMETERS : PACKET_HEADER_1;
                break;

            case PACKET_PARAMETERS:
                for (uint8_t i = 0; i < rx_packet.elements.length - 2; i++) {
                    rx_packet.elements.args[i] = uart->read();
                }
                rx_status = PACKET_CHECKSUM;
                break;

            case PACKET_CHECKSUM:
                rx_packet.elements.args[rx_packet.elements.length - 2] = uart->read();
                check_value = data_check((const uint8_t *)&rx_packet, rx_packet.elements.length + 3);
                rx_status = rx_packet.elements.args[rx_packet.elements.length - 2] == check_value
                                ? PACKET_FINISH
                                : PACKET_HEADER_1;
                break;

            default:
                break;
        }

        if (rx_status == PACKET_FINISH) {
            break;
        }
    }

    error = rx_status == PACKET_FINISH ? 0 : 1;
    return error;
}

/* -------------------------------------------------------------------------
 * Frame building (transmit)
 * ---------------------------------------------------------------------- */
uint8_t HXServo::tx_frame_write(uint8_t id, uint8_t cmd, const uint8_t *data, uint8_t data_len)
{
    uint8_t frame_len = 6 + data_len;   /* header1 + header2 + id + length + cmd + checksum */
    uint8_t packet[frame_len];

    packet[0] = FRAME_HEADER_1;
    packet[1] = FRAME_HEADER_2;
    packet[2] = id;
    packet[3] = 2 + data_len;           /* length = cmd + params + checksum */
    packet[4] = cmd;

    for (uint8_t i = 0; i < data_len; i++) {
        packet[5 + i] = data[i];
    }

    packet[frame_len - 1] = data_check((const uint8_t *)packet, frame_len - 1);

#if HX_DEBUG
    HX_DEBUG_STREAM.print("TX: ");
    for (uint8_t i = 0; i < frame_len; i++) {
        HX_DEBUG_STREAM.print(packet[i], HEX);
        HX_DEBUG_STREAM.print(' ');
    }
    HX_DEBUG_STREAM.println();
#endif

    return uart->write(packet, frame_len);
}

/* -------------------------------------------------------------------------
 * Reply handling
 * ---------------------------------------------------------------------- */
ServoStatus_t HXServo::ack()
{
    ServoStatus_t status;
    uint8_t size;
    uint32_t tickstart;

    status.id = 0xFF;
    status.error_byte = 0;

    if (!rx_skip) {
        tickstart = millis();
        while (1) {
            size = uart->available();
            if (size >= rx_frame_length) {
                break;
            }
            if (millis() - tickstart > rx_timeout) {
                status.error_bits.bit_rx = 1;
                return status;
            }
        }

        if (unpack()) {
            status.error_bits.bit_rx = 1;
        } else {
            status.id = rx_packet.elements.id;
            status.error_byte = rx_packet.elements.cmd;
        }

#if HX_DEBUG
        HX_DEBUG_STREAM.print("RX: ");
        HX_DEBUG_STREAM.print(rx_packet.header_1, HEX);        HX_DEBUG_STREAM.print(' ');
        HX_DEBUG_STREAM.print(rx_packet.header_2, HEX);        HX_DEBUG_STREAM.print(' ');
        HX_DEBUG_STREAM.print(rx_packet.elements.id, HEX);     HX_DEBUG_STREAM.print(' ');
        HX_DEBUG_STREAM.print(rx_packet.elements.length, HEX); HX_DEBUG_STREAM.print(' ');
        HX_DEBUG_STREAM.print(rx_packet.elements.cmd, HEX);    HX_DEBUG_STREAM.print(' ');
        for (uint8_t i = 0; i < rx_packet.elements.length - 1; i++) {
            HX_DEBUG_STREAM.print(rx_packet.elements.args[i], HEX);
            HX_DEBUG_STREAM.print(' ');
        }
        HX_DEBUG_STREAM.println();
#endif
    }

    return status;
}

/* -------------------------------------------------------------------------
 * Low-level instructions
 * ---------------------------------------------------------------------- */
ServoStatus_t HXServo::ping(uint8_t id)
{
    ServoStatus_t status;

    status.id = id;
    status.error_byte = 0;
    rx_skip = 0;
    rx_frame_length = 6;

    if (!tx_frame_write(id, CMD_PING, NULL, 0)) {
        status.error_bits.bit_tx = 1;
        return status;
    }
    return ack();
}

ServoStatus_t HXServo::general_write(uint8_t id, uint8_t addr, uint8_t *data, uint8_t data_len)
{
    ServoStatus_t status;
    uint8_t buf[1 + data_len];

    /* Broadcast writes are not acknowledged; unicast writes are. */
    rx_skip = (id == BROADCAST_ID) ? 1 : 0;
    rx_frame_length = 6;

    status.id = id;
    status.error_byte = 0;
    buf[0] = addr;
    for (uint8_t i = 0; i < data_len; i++) {
        buf[1 + i] = data[i];
    }

    if (!tx_frame_write(id, CMD_WRITE, buf, sizeof(buf))) {
        status.error_bits.bit_tx = 1;
        return status;
    }
    return ack();
}

ServoStatus_t HXServo::general_read(uint8_t id, uint8_t addr, uint8_t *data, uint8_t data_len)
{
    ServoStatus_t status;
    uint8_t buf[2];

    buf[0] = addr;
    buf[1] = data_len;
    rx_skip = 0;
    rx_frame_length = 6 + data_len;   /* header(2)+id+length+status+params+checksum */

    if (!tx_frame_write(id, CMD_READ, buf, sizeof(buf))) {
        status.error_bits.bit_tx = 1;
        return status;
    }

    status = ack();
    if (status.error_bits.bit_rx) {
        return status;
    }
    for (uint8_t i = 0; i < data_len; i++) {
        data[i] = rx_packet.elements.args[i];
    }
    return status;
}

ServoStatus_t HXServo::reg_write(uint8_t id, uint8_t addr, uint8_t *data, uint8_t data_len)
{
    ServoStatus_t status;
    uint8_t buf[1 + data_len];

    rx_skip = (id == BROADCAST_ID) ? 1 : 0;
    rx_frame_length = 6;

    status.id = id;
    status.error_byte = 0;
    buf[0] = addr;
    for (uint8_t i = 0; i < data_len; i++) {
        buf[1 + i] = data[i];
    }

    if (!tx_frame_write(id, CMD_REG_WRITE, buf, sizeof(buf))) {
        status.error_bits.bit_tx = 1;
        return status;
    }
    return ack();
}

ServoStatus_t HXServo::reg_action(uint8_t id)
{
    ServoStatus_t status;

    rx_skip = (id == BROADCAST_ID) ? 1 : 0;
    rx_frame_length = 6;

    status.id = id;
    status.error_byte = 0;

    if (!tx_frame_write(id, CMD_ACTION, NULL, 0)) {
        status.error_bits.bit_tx = 1;
        return status;
    }
    return ack();
}

ServoStatus_t HXServo::sync_write(uint8_t addr, uint8_t *data, uint8_t data_len, uint8_t parameter_len)
{
    ServoStatus_t status;
    uint8_t total_size = data_len + 2;
    uint8_t buf[total_size];

    buf[0] = addr;
    buf[1] = parameter_len;

    status.id = BROADCAST_ID;
    status.error_byte = 0;

    for (uint8_t i = 0; i < data_len; i++) {
        buf[2 + i] = data[i];
    }

    if (!tx_frame_write(BROADCAST_ID, CMD_SYNC_WRITE, buf, sizeof(buf))) {
        status.error_bits.bit_tx = 1;
        return status;
    }
    return status;   /* sync write is broadcast: no reply expected */
}

ServoStatus_t HXServo::sync_read(uint8_t addr, uint8_t byte_num, uint8_t *id, uint8_t id_num,
                                 uint8_t *data, uint8_t *ok_mask)
{
    ServoStatus_t status;
    const uint8_t total_size = 2 + id_num;
    uint8_t buf[total_size];
    uint8_t answered = 0;

    buf[0] = addr;
    buf[1] = byte_num;
    status.id = BROADCAST_ID;
    status.error_byte = 0;

    for (uint8_t i = 0; i < id_num; i++) {
        buf[2 + i] = id[i];
    }

    /* Late replies from a previous transaction would be parsed as the head of the
     * first reply here, so start from a known-empty bus. */
    flush_rx();

    rx_skip = 0;
    rx_frame_length = 6 + byte_num;

    if (!tx_frame_write(BROADCAST_ID, CMD_SYNC_READ, buf, sizeof(buf))) {
        status.error_bits.bit_tx = 1;
        if (ok_mask) {
            *ok_mask = 0;
        }
        return status;
    }

    /* Each addressed servo replies in turn. File every frame by the ID it carries
     * rather than by arrival order: a servo that never answers must leave a gap,
     * not shift the servos behind it into its slot. */
    for (uint8_t i = 0; i < id_num; i++) {
        ServoStatus_t reply = ack();

        if (reply.error_bits.bit_rx) {
            status.error_bits.bit_rx = 1;
            break;   /* the timeout has already elapsed; the rest are not coming */
        }

        /* A reply frame carries the servo's error flags where a command carries the
         * instruction byte. Keep them, but not in the two driver-owned bits. */
        status.error_byte |= reply.error_byte & 0x3F;

        for (uint8_t slot = 0; slot < id_num; slot++) {
            if (id[slot] != reply.id || (answered & (1u << slot))) {
                continue;
            }
            for (uint8_t j = 0; j < byte_num; j++) {
                data[slot * byte_num + j] = rx_packet.elements.args[j];
            }
            answered |= (1u << slot);
            break;
        }
    }

    if (answered != (uint8_t)((1u << id_num) - 1)) {
        status.error_bits.bit_rx = 1;
        flush_rx();   /* a straggler must not corrupt the next transaction */
    }

    if (ok_mask) {
        *ok_mask = answered;
    }
    return status;
}

void HXServo::flush_rx()
{
    while (uart->available()) {
        uart->read();
    }
    rx_status = PACKET_HEADER_1;
}

/* -------------------------------------------------------------------------
 * High-level convenience commands
 * ---------------------------------------------------------------------- */
ServoStatus_t HXServo::enable_torque(uint8_t id)
{
    uint8_t data = 1;
    return general_write(id, REG_TORQUE_ENABLE, &data, 1);
}

ServoStatus_t HXServo::disable_torque(uint8_t id)
{
    uint8_t data = 0;
    return general_write(id, REG_TORQUE_ENABLE, &data, 1);
}

ServoStatus_t HXServo::cali_pos(uint8_t id)
{
    uint8_t data = 128;   /* special value: calibrate current shaft as center */
    return general_write(id, REG_TORQUE_ENABLE, &data, 1);
}

ServoStatus_t HXServo::select_mode(uint8_t id, uint8_t mode)
{
    uint8_t data = mode;
    return general_write(id, REG_MODE, &data, 1);
}

ServoStatus_t HXServo::write_pos_offset(uint8_t id, int16_t offset)
{
    uint16_t data;
    offset = LIMIT(offset, -2047, 2047);
    data = (uint16_t)MASK_HOST(offset, 11);
    return general_write(id, REG_POS_OFFSET_L, (uint8_t *)&data, sizeof(data));
}

ServoStatus_t HXServo::write_pos(uint8_t id, int16_t pos)
{
    uint16_t data;
    pos = LIMIT(pos, -30719, 30719);
    data = (uint16_t)MASK_HOST(pos, 15);
    return general_write(id, REG_GOAL_POSITION_L, (uint8_t *)&data, sizeof(data));
}

ServoStatus_t HXServo::write_acc(uint8_t id, uint8_t acc)
{
    uint8_t data = LIMIT(acc, 0, 254);
    return general_write(id, REG_ACC, &data, sizeof(data));
}

ServoStatus_t HXServo::write_speed(uint8_t id, int16_t speed)
{
    uint16_t data;
    speed = LIMIT(speed, -3400, 3400);
    data = (uint16_t)MASK_HOST(speed, 15);
    return general_write(id, REG_GOAL_SPEED_L, (uint8_t *)&data, sizeof(data));
}

ServoStatus_t HXServo::write_pos_ex(uint8_t id, uint8_t acc, int16_t speed, int16_t pos)
{
    uint8_t  data[7];
    uint16_t _pos;
    uint16_t _speed;

    acc   = LIMIT(acc, 0, 254);
    speed = LIMIT(speed, -3400, 3400);
    pos   = LIMIT(pos, -30719, 30719);

    _pos   = (uint16_t)MASK_HOST(pos, 15);
    _speed = (uint16_t)MASK_HOST(speed, 15);

    /* Register block starting at REG_ACC (41):
     * 41 acc | 42/43 goal position | 44/45 pwm speed (0) | 46/47 goal speed */
    data[0] = acc;
    word2bytes(_pos, &data[1], &data[2]);
    data[3] = 0;
    data[4] = 0;
    word2bytes(_speed, &data[5], &data[6]);

    return general_write(id, REG_ACC, (uint8_t *)&data, sizeof(data));
}

ServoStatus_t HXServo::write_pwm_speed(uint8_t id, int16_t speed)
{
    uint16_t data;
    speed = LIMIT(speed, -1000, 1000);
    data = (uint16_t)MASK_HOST(speed, 10);
    return general_write(id, REG_PWM_SPEED_L, (uint8_t *)&data, sizeof(data));
}

ServoStatus_t HXServo::write_max_torque(uint8_t id, uint16_t torque)
{
    uint16_t data = LIMIT(torque, 0, 1000);
    return general_write(id, REG_MAX_TORQUE_L, (uint8_t *)&data, sizeof(data));
}

ServoStatus_t HXServo::write_reg_pos_ex(uint8_t id, uint8_t acc, int16_t speed, int16_t pos)
{
    uint8_t  data[7];
    uint16_t _pos;
    uint16_t _speed;

    acc   = LIMIT(acc, 0, 254);
    speed = LIMIT(speed, -3400, 3400);
    pos   = LIMIT(pos, -30719, 30719);

    _pos   = (uint16_t)MASK_HOST(pos, 15);
    _speed = (uint16_t)MASK_HOST(speed, 15);

    data[0] = acc;
    word2bytes(_pos, &data[1], &data[2]);
    data[3] = 0;
    data[4] = 0;
    word2bytes(_speed, &data[5], &data[6]);

    return reg_write(id, REG_ACC, (uint8_t *)&data, sizeof(data));
}

ServoStatus_t HXServo::sync_write_pos_ex(int16_t (*data)[4], uint8_t id_num)
{
    ServoStatus_t status;
    uint8_t  id;
    uint8_t  acc;
    int16_t  speed;
    int16_t  pos;
    uint16_t u16_speed;
    uint16_t u16_pos;
    uint8_t  buf[id_num * 8];

    if (id_num > 30) {
        status.id = 0xFF;
        status.error_byte = 0;
        status.error_bits.bit_tx = 1;
        return status;
    }

    for (uint8_t i = 0; i < id_num; i++) {
        id    = (uint8_t)data[i][0];
        acc   = (uint8_t)data[i][1];
        speed = (int16_t)data[i][2];
        pos   = (int16_t)data[i][3];

        acc   = LIMIT(acc, 0, 254);
        speed = LIMIT(speed, -3400, 3400);
        pos   = LIMIT(pos, -30719, 30719);

        u16_pos   = (uint16_t)MASK_HOST(pos, 15);
        u16_speed = (uint16_t)MASK_HOST(speed, 15);

        buf[i * 8]       = id;
        buf[(i * 8) + 1] = acc;
        word2bytes(u16_pos, &buf[(i * 8) + 2], &buf[(i * 8) + 3]);
        buf[(i * 8) + 4] = 0;
        buf[(i * 8) + 5] = 0;
        word2bytes(u16_speed, &buf[(i * 8) + 6], &buf[(i * 8) + 7]);
    }

    return sync_write(REG_ACC, buf, sizeof(buf), 7);
}

ServoStatus_t HXServo::sync_read_cur_pos_ex(uint8_t *id, uint8_t id_num, int16_t (*data)[5],
                                            uint8_t *ok_mask)
{
    ServoStatus_t status;
    const uint8_t byte_len = 8;   /* 56..63: position, speed, load, voltage, temperature */
    uint8_t read_data[byte_len * id_num];
    uint8_t answered = 0;

    status = sync_read(REG_PRESENT_POSITION_L, byte_len, id, id_num, read_data, &answered);

    if (ok_mask) {
        *ok_mask = answered;
    }

    if (status.error_bits.bit_tx) {
        return status;
    }

    /* Decode only the servos that actually answered. The rest of read_data is
     * untouched, so writing it out would hand the caller stack garbage. */
    for (uint8_t i = 0; i < id_num; i++) {
        if (!(answered & (1u << i))) {
            continue;
        }

        const uint8_t o = i * byte_len;
        uint16_t u16_pos   = bytes2word(&read_data[o],     &read_data[o + 1]);
        uint16_t u16_speed = bytes2word(&read_data[o + 2], &read_data[o + 3]);
        uint16_t u16_load  = bytes2word(&read_data[o + 4], &read_data[o + 5]);

        data[i][0] = (int16_t)MASK_SERVO(u16_pos, 15);
        data[i][1] = (int16_t)MASK_SERVO(u16_speed, 15);
        data[i][2] = (int16_t)MASK_SERVO(u16_load, 10);
        data[i][3] = (int16_t)read_data[o + 6];   /* voltage     */
        data[i][4] = (int16_t)read_data[o + 7];   /* temperature */
    }
    return status;
}

/* -------------------------------------------------------------------------
 * Telemetry reads
 * ---------------------------------------------------------------------- */
ServoStatus_t HXServo::read_pos_offset(uint8_t id, int16_t *offset)
{
    ServoStatus_t status;
    uint8_t data[2];
    uint16_t _offset;

    status = general_read(id, REG_POS_OFFSET_L, data, 2);
    if (status.error_bits.bit_tx || status.error_bits.bit_rx) {
        *offset = 0;
        return status;
    }
    _offset = bytes2word(data, data + 1);
    *offset = (int16_t)MASK_SERVO(_offset, 11);
    return status;
}

ServoStatus_t HXServo::read_pos(uint8_t id, int16_t *pos)
{
    ServoStatus_t status;
    uint8_t data[2];
    uint16_t _pos;

    status = general_read(id, REG_PRESENT_POSITION_L, data, 2);
    if (status.error_bits.bit_tx || status.error_bits.bit_rx) {
        *pos = 0;
        return status;
    }
    _pos = bytes2word(data, data + 1);
    *pos = (int16_t)MASK_SERVO(_pos, 15);
    return status;
}

ServoStatus_t HXServo::read_speed(uint8_t id, int16_t *speed)
{
    ServoStatus_t status;
    uint8_t data[2];
    uint16_t _speed;

    status = general_read(id, REG_PRESENT_SPEED_L, data, 2);
    if (status.error_bits.bit_tx || status.error_bits.bit_rx) {
        *speed = 0;
        return status;
    }
    _speed = bytes2word(data, data + 1);
    *speed = (int16_t)MASK_SERVO(_speed, 15);
    return status;
}

ServoStatus_t HXServo::read_pos_speed(uint8_t id, int16_t *pos, int16_t *speed)
{
    ServoStatus_t status;
    uint8_t data[4];
    uint16_t _pos;
    uint16_t _speed;

    status = general_read(id, REG_PRESENT_POSITION_L, data, 4);
    if (status.error_bits.bit_tx || status.error_bits.bit_rx) {
        *pos = 0;
        *speed = 0;
        return status;
    }
    _pos = bytes2word(data, data + 1);
    *pos = (int16_t)MASK_SERVO(_pos, 15);

    _speed = bytes2word(data + 2, data + 3);
    *speed = (int16_t)MASK_SERVO(_speed, 15);
    return status;
}

ServoStatus_t HXServo::read_temperature(uint8_t id, uint8_t *temp)
{
    ServoStatus_t status;
    uint8_t data[1];

    status = general_read(id, REG_PRESENT_TEMPERATURE, data, 1);
    if (status.error_bits.bit_tx || status.error_bits.bit_rx) {
        *temp = 0;
        return status;
    }
    *temp = data[0];
    return status;
}

ServoStatus_t HXServo::read_voltage(uint8_t id, uint8_t *vol)
{
    ServoStatus_t status;
    uint8_t data[1];

    status = general_read(id, REG_PRESENT_VOLTAGE, data, 1);
    if (status.error_bits.bit_tx || status.error_bits.bit_rx) {
        *vol = 0;
        return status;
    }
    *vol = data[0];
    return status;
}

ServoStatus_t HXServo::read_current(uint8_t id, uint16_t *cur)
{
    ServoStatus_t status;
    uint8_t data[2];
    uint16_t _cur;

    status = general_read(id, REG_PRESENT_CURRENT_L, data, 2);
    if (status.error_bits.bit_tx || status.error_bits.bit_rx) {
        *cur = 0;
        return status;
    }
    _cur = bytes2word(data, data + 1);
    *cur = _cur;
    return status;
}

ServoStatus_t HXServo::read_load(uint8_t id, int16_t *load)
{
    ServoStatus_t status;
    uint8_t data[2];
    int16_t _load;

    status = general_read(id, REG_PRESENT_LOAD_L, data, 2);
    if (status.error_bits.bit_tx || status.error_bits.bit_rx) {
        *load = 0;
        return status;
    }
    _load = bytes2word(data, data + 1);
    *load = (int16_t)MASK_SERVO(_load, 10);
    return status;
}

ServoStatus_t HXServo::read_moving_status(uint8_t id, uint8_t *moving_status)
{
    ServoStatus_t status;
    uint8_t data;

    status = general_read(id, REG_MOVING_STATUS, &data, sizeof(data));
    if (status.error_bits.bit_tx || status.error_bits.bit_rx) {
        *moving_status = 0;
        return status;
    }
    *moving_status = data;
    return status;
}