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

/* Microseconds one 8N1 byte occupies on the wire (8 data + start + stop bits). */
#define WIRE_US_PER_BYTE(baud) (10000000UL / (baud))

/* Gap between polls while waiting for a late reply. Sleeping keeps available()
 * off the hot path: on some cores it takes a lock, and hammering it masks
 * interrupts often enough to jitter timing-sensitive work elsewhere. */
#define RX_POLL_US 50

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
    uart->setTimeout(rx_timeout);   /* Bound readBytes() in unpack(). */
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
/* Pulls exactly one expected-length frame in, resyncs past any leading garbage
 * and validates it in memory. The original drove a byte-at-a-time state machine
 * off available(), so it quit mid-frame whenever the parser outran the wire and
 * folded read()'s -1 into the payload -- both of which forced a 20ms timeout. */
uint8_t HXServo::unpack()
{
    uint8_t *buf = rx_packet.data_raw;
    uint8_t want = rx_frame_length;
    uint8_t len;
    uint8_t skip = 0;

    rx_status = PACKET_HEADER_1;

    if (want > sizeof(rx_packet.data_raw)) {
        want = sizeof(rx_packet.data_raw);
    }
    len = (uint8_t)uart->readBytes(buf, want);

    /* Drop anything ahead of the header, then top the frame back up. */
    while (skip + 1 < len && !(buf[skip] == FRAME_HEADER_1 && buf[skip + 1] == FRAME_HEADER_2)) {
        skip++;
    }
    if (skip) {
        len -= skip;
        memmove(buf, buf + skip, len);
        len += (uint8_t)uart->readBytes(&buf[len], skip);
    }

    if (len < 6 || buf[0] != FRAME_HEADER_1 || buf[1] != FRAME_HEADER_2) {
        return 1;
    }
    if (rx_packet.elements.id > BROADCAST_ID || rx_packet.elements.length < 2 ||
        rx_packet.elements.length + 4 > len) {
        return 1;
    }
    if (rx_packet.elements.args[rx_packet.elements.length - 2] !=
        data_check(buf, rx_packet.elements.length + 3)) {
        return 1;
    }

    rx_status = PACKET_FINISH;
    return 0;
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

    /* Drop stale or echoed bytes so ack() cannot match on them. */
    while (uart->available() > 0) {
        (void)uart->read();
    }

    return uart->write(packet, frame_len);
}

/* -------------------------------------------------------------------------
 * Reply handling
 * ---------------------------------------------------------------------- */
ServoStatus_t HXServo::ack()
{
    ServoStatus_t status;
    uint32_t wire_us;
    uint32_t tickstart;

    status.id = 0xFF;
    status.error_byte = 0;

    if (!rx_skip) {
        /* The reply cannot land before the wire has carried it, so sleep through
         * that window instead of spinning on available(). */
        wire_us = (uint32_t)rx_frame_length * WIRE_US_PER_BYTE(baudrate);
        delay(wire_us / 1000);
        delayMicroseconds(wire_us % 1000);

        tickstart = millis();
        while (uart->available() < (int)rx_frame_length) {
            if (millis() - tickstart > rx_timeout) {
                status.error_bits.bit_rx = 1;
                return status;
            }
            delayMicroseconds(RX_POLL_US);
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

ServoStatus_t HXServo::sync_read(uint8_t addr, uint8_t byte_num, uint8_t *id, uint8_t id_num, uint8_t *data)
{
    ServoStatus_t status;
    const uint8_t total_size = 2 + id_num;   /* addr + byte_num + one ID each */
    uint8_t buf[total_size];

    buf[0] = addr;
    buf[1] = byte_num;
    status.id = BROADCAST_ID;
    status.error_byte = 0;
    rx_skip = 0;
    rx_frame_length = 6 + byte_num;

    for (uint8_t i = 0; i < id_num; i++) {
        buf[2 + i] = id[i];
    }

    if (!tx_frame_write(BROADCAST_ID, CMD_SYNC_READ, buf, sizeof(buf))) {
        status.error_bits.bit_tx = 1;
        return status;
    }

    /* Each addressed servo replies in turn. */
    for (uint8_t i = 0; i < id_num; i++) {
        status = ack();
        if (status.error_bits.bit_rx) {
            return status;
        }
        for (uint8_t j = 0; j < byte_num; j++) {
            data[i * byte_num + j] = rx_packet.elements.args[j];
        }
    }
    return status;
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

ServoStatus_t HXServo::sync_read_cur_pos_ex(uint8_t *id, uint8_t id_num, int16_t (*data)[5])
{
    ServoStatus_t status;
    const uint8_t byte_len = 8;
    const uint8_t buf_size = id_num;
    const uint8_t read_buf_size = byte_len * id_num;
    uint8_t  read_data[read_buf_size];
    uint16_t u16_pos[buf_size];
    uint16_t u16_speed[buf_size];
    uint16_t u16_load[buf_size];

    status = sync_read(REG_PRESENT_POSITION_L, byte_len, id, id_num, read_data);
    if (status.error_bits.bit_tx || status.error_bits.bit_rx) {
        return status;
    }

    for (uint8_t i = 0; i < id_num; i++) {
        u16_pos[i]   = bytes2word(&read_data[i * byte_len],       &read_data[(i * byte_len) + 1]);
        u16_speed[i] = bytes2word(&read_data[(i * byte_len) + 2], &read_data[(i * byte_len) + 3]);
        u16_load[i]  = bytes2word(&read_data[(i * byte_len) + 4], &read_data[(i * byte_len) + 5]);

        data[i][0] = (int16_t)MASK_SERVO(u16_pos[i], 15);
        data[i][1] = (int16_t)MASK_SERVO(u16_speed[i], 15);
        data[i][2] = (int16_t)MASK_SERVO(u16_load[i], 10);
        data[i][3] = (int16_t)read_data[(i * byte_len) + 6];   /* voltage     */
        data[i][4] = (int16_t)read_data[(i * byte_len) + 7];   /* temperature */
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
