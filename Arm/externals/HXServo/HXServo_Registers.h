/**
 * @file    HXServo_Registers.h
 * @brief   Control-table (register) map and protocol constants for the
 *          Hiwonder Magnetic-Encoder intelligent bus servos.
 *
 * These definitions are shared by every servo in the HX "HM" family
 * (HX-10HM, HX-30HM, HX-65HM, ...). The wire protocol and memory layout are
 * identical across the family, so the same driver controls all of them --
 * only the mechanical/torque specs differ between models.
 *
 * Protocol family: 0xFF 0xFF header, checksum = ~(sum of all bytes after the
 * header). This is the "Hiwonder Magnetic-Encoder Bus Servo Communication
 * Protocol", NOT the 0x55-header LewanSoul/LX controller-board protocol.
 */
#ifndef HXSERVO_REGISTERS_H
#define HXSERVO_REGISTERS_H

/* -------------------------------------------------------------------------
 * Control table -- non-volatile (survives power cycle, stored in servo NVS)
 * ---------------------------------------------------------------------- */
#define REG_SERVO_MAIN_VERSION   3
#define REG_SERVO_SEC_VERSION    4
#define REG_ID                   5   /* Servo ID (0-253)                    */
#define REG_BAUD_RATE            6   /* Baud-rate index, see BAUD_RATE_*    */
#define REG_CW_DEAD              26  /* Clockwise dead-band                 */
#define REG_CCW_DEAD             27  /* Counter-clockwise dead-band         */
#define REG_POS_OFFSET_L         31  /* Position offset, low byte           */
#define REG_POS_OFFSET_H         32  /* Position offset, high byte          */
#define REG_MODE                 33  /* Operating mode, see *_MODE          */

/* -------------------------------------------------------------------------
 * Control table -- read/write (volatile, RAM)
 * ---------------------------------------------------------------------- */
#define REG_TORQUE_ENABLE        40  /* 0 = off, 1 = on, 128 = calibrate    */
#define REG_ACC                  41  /* Acceleration (0-254)                */
#define REG_GOAL_POSITION_L      42
#define REG_GOAL_POSITION_H      43
#define REG_PWM_SPEED_L          44
#define REG_PWM_SPEED_H          45
#define REG_GOAL_SPEED_L         46
#define REG_GOAL_SPEED_H         47
#define REG_MAX_TORQUE_L         48
#define REG_MAX_TORQUE_H         49

/* -------------------------------------------------------------------------
 * Control table -- read only (live telemetry)
 * ---------------------------------------------------------------------- */
#define REG_PRESENT_POSITION_L   56
#define REG_PRESENT_POSITION_H   57
#define REG_PRESENT_SPEED_L      58
#define REG_PRESENT_SPEED_H      59
#define REG_PRESENT_LOAD_L       60
#define REG_PRESENT_LOAD_H       61
#define REG_PRESENT_VOLTAGE      62  /* 0.1 V per count                     */
#define REG_PRESENT_TEMPERATURE  63  /* degrees Celsius                     */
#define REG_MOVING_STATUS        66  /* 0 = stopped, 1 = moving             */
#define REG_PRESENT_CURRENT_L    69
#define REG_PRESENT_CURRENT_H    70

/* -------------------------------------------------------------------------
 * Protocol framing
 * ---------------------------------------------------------------------- */
#define FRAME_HEADER_1           0xFF
#define FRAME_HEADER_2           0xFF

/* Instruction (command) bytes */
#define CMD_PING                 1
#define CMD_READ                 2
#define CMD_WRITE                3
#define CMD_REG_WRITE            4   /* Buffered write, applied on CMD_ACTION */
#define CMD_ACTION               5   /* Trigger a previously buffered REG_WRITE */
#define CMD_RESET                6
#define CMD_SYNC_READ            130
#define CMD_SYNC_WRITE           131

/* Broadcast ID: every servo on the bus receives the packet, none replies */
#define BROADCAST_ID             0xFE

/* -------------------------------------------------------------------------
 * Baud-rate indices written to REG_BAUD_RATE
 * ---------------------------------------------------------------------- */
#define BAUD_RATE_1M             0   /* 1,000,000 bps (factory default)     */
#define BAUD_RATE_0_5_M          1   /*   500,000 bps                       */
#define BAUD_RATE_250K           2   /*   250,000 bps                       */
#define BAUD_RATE_115200         4
#define BAUD_RATE_76800          5
#define BAUD_RATE_57600          6
#define BAUD_RATE_38400          7

/* -------------------------------------------------------------------------
 * Operating modes written to REG_MODE
 * ---------------------------------------------------------------------- */
#define POSITION_MODE            0   /* Absolute position servo mode        */
#define CLOSED_LOOP_MOTOR_MODE   1   /* Constant-speed wheel mode           */
#define OPEN_LOOP_MOTOR_MODE     2   /* PWM (duty) wheel mode, speed varies */

#endif /* HXSERVO_REGISTERS_H */
