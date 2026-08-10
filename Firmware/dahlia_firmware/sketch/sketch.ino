#include "src/low_level/servo_interface.h"
#include "src/spi/spi_service.h"

static_assert(JOINT_COUNT == SPI_JOINT_COUNT, "Packet joint count must match the arm joint count.");

constexpr int SERVO_PERIOD_MS = 5;   // 200 Hz servo loop
constexpr int SPI_STACK_SIZE = 2048;
constexpr int SPI_PRIORITY = 5;
constexpr float HOLD_VEL = 5.0f;   // Radians per second, matches JOINT_VEL on the host

// Timeout after which if new commands aren't received, arm holds its pose regardless of targets
constexpr unsigned long COMMAND_TIMEOUT_MS = 500;

DahliaArm arm;
SPIService<FeedbackPacket, CommandPacket> spi;

/**
 * @brief The only state shared between the SPI thread and the servo loop.
 */
struct SharedState {
    // Command, written by the SPI thread
    uint8_t mode;
    uint8_t torque;   // Raw byte from the host, TORQUE_APPLY still set
    float target_position[JOINT_COUNT];   // Radians
    float target_velocity[JOINT_COUNT];   // Radians per second
    int16_t raw_position[JOINT_COUNT];   // Ticks
    int16_t raw_velocity[JOINT_COUNT];   // Ticks per second
    uint8_t raw_acceleration[JOINT_COUNT];
    float gripper;   // Normalized clamp closure, [0.0, 1.0]
    uint8_t sequence;
    unsigned long command_at;   // millis() when the last command was accepted
    bool command_seen;

    // Feedback, written by the servo loop
    float current_position[JOINT_COUNT];
    float current_velocity[JOINT_COUNT];
    int16_t raw_current_position[JOINT_COUNT];
    int16_t raw_current_velocity[JOINT_COUNT];
    int16_t raw_load[JOINT_COUNT];
    uint8_t voltage[JOINT_COUNT];
    uint8_t temperature[JOINT_COUNT];
    uint8_t status;
    uint8_t torque_state;
    uint8_t comm_errors;
    uint8_t effective_mode;   // What the loop actually ran, which is MODE_HOLD while stale
    uint16_t loop_count;
};

SharedState shared;
K_MUTEX_DEFINE(shared_mutex);

K_THREAD_STACK_DEFINE(spi_stack, SPI_STACK_SIZE);
struct k_thread spi_thread;

/**
 * @brief SPI thread, sending/receiving newest feedback and commands, nonblocking.
 */
void spi_task(void*, void*, void*){
    while (true){
        k_mutex_lock(&shared_mutex, K_FOREVER);   // Lock when reading current state for telemetry
        spi.tx.magic = FEEDBACK_MAGIC;
        spi.tx.sequence = shared.sequence;
        spi.tx.mode = shared.effective_mode;
        spi.tx.status = shared.status;
        for (int i = 0; i < JOINT_COUNT; i++){
            spi.tx.current_position[i] = shared.current_position[i];
            spi.tx.current_velocity[i] = shared.current_velocity[i];
            spi.tx.raw_position[i] = shared.raw_current_position[i];
            spi.tx.raw_velocity[i] = shared.raw_current_velocity[i];
            spi.tx.raw_load[i] = shared.raw_load[i];
            spi.tx.voltage[i] = shared.voltage[i];
            spi.tx.temperature[i] = shared.temperature[i];
        }
        spi.tx.torque = shared.torque_state;
        spi.tx.comm_errors = shared.comm_errors;
        spi.tx.gripper = (uint8_t)lroundf(shared.gripper * 255.0f);
        spi.tx.loop_count = shared.loop_count;
        k_mutex_unlock(&shared_mutex);

        if (!spi.transfer()){
            k_msleep(1);   // Yield rather than spin the CPU on a bus that is not clocking
            continue;
        }

        if (spi.rx.magic != COMMAND_MAGIC){
            continue;
        }

        k_mutex_lock(&shared_mutex, K_FOREVER);   // Lock when receiving new command data
        shared.mode = spi.rx.mode;
        shared.torque = spi.rx.torque;
        for (int i = 0; i < JOINT_COUNT; i++){
            shared.target_position[i] = spi.rx.target_position[i];
            shared.target_velocity[i] = spi.rx.target_velocity[i];
            shared.raw_position[i] = spi.rx.raw_position[i];
            shared.raw_velocity[i] = spi.rx.raw_velocity[i];
            shared.raw_acceleration[i] = spi.rx.raw_acceleration[i];
        }
        shared.gripper = spi.rx.gripper / 255.0f;
        shared.sequence = spi.rx.sequence;
        shared.command_at = millis();
        shared.command_seen = true;
        k_mutex_unlock(&shared_mutex);
    }
}

void setup(){
    Serial.begin(115200);
    arm.initialize();

    // Hold the startup pose until the host sends its first command
    shared.mode = MODE_HOLD;
    shared.effective_mode = MODE_HOLD;
    shared.torque = 0;   // No TORQUE_APPLY, so the loop leaves torque as initialize() left it
    for (int i = 0; i < JOINT_COUNT; i++){
        shared.target_position[i] = arm.get_pos((JointID)i);
        shared.target_velocity[i] = HOLD_VEL;
        shared.raw_position[i] = arm.get_raw_pos((JointID)i);
        shared.raw_velocity[i] = 3400;
        shared.raw_acceleration[i] = 254;
    }
    shared.gripper = 0.0f;

    spi.initialize();
    k_thread_create(&spi_thread, spi_stack, K_THREAD_STACK_SIZEOF(spi_stack),
                    spi_task, NULL, NULL, NULL, SPI_PRIORITY, 0, K_NO_WAIT);
}

void loop() {
    unsigned long cycle_start = millis();

    arm.refresh_state();

    // Snapshot of the command, so the bus work below runs outside the lock
    uint8_t mode;
    uint8_t torque;
    float target_position[JOINT_COUNT];
    float target_velocity[JOINT_COUNT];
    int16_t raw_position[JOINT_COUNT];
    int16_t raw_velocity[JOINT_COUNT];
    uint8_t raw_acceleration[JOINT_COUNT];
    float gripper;
    uint8_t status = 0;

    k_mutex_lock(&shared_mutex, K_FOREVER);   // Lock to prevent SPI updates when reading from state
    for (int i = 0; i < JOINT_COUNT; i++){
        JointStateExternal state = arm.get_state((JointID)i);
        shared.current_position[i] = state.current_angle;
        shared.current_velocity[i] = state.current_velocity;
        shared.raw_current_position[i] = arm.get_raw_pos((JointID)i);
        shared.raw_current_velocity[i] = arm.get_raw_vel((JointID)i);
        shared.raw_load[i] = arm.get_raw_load((JointID)i);
        shared.voltage[i] = arm.get_voltage((JointID)i);
        shared.temperature[i] = arm.get_temperature((JointID)i);
        status |= (state.moving ? STATUS_MOVING : 0x00);

        target_position[i] = shared.target_position[i];
        target_velocity[i] = shared.target_velocity[i];
        raw_position[i] = shared.raw_position[i];
        raw_velocity[i] = shared.raw_velocity[i];
        raw_acceleration[i] = shared.raw_acceleration[i];
    }

    bool fresh = shared.command_seen && (cycle_start - shared.command_at) <= COMMAND_TIMEOUT_MS;
    mode = fresh ? shared.mode : MODE_HOLD;
    torque = shared.torque;
    gripper = shared.gripper;
    k_mutex_unlock(&shared_mutex);

    // Torque is only modified if validation byte is set
    if (fresh && (torque & TORQUE_APPLY)){
        for (int i = 0; i < JOINT_COUNT; i++){
            arm.set_torque((JointID)i, (torque & (1u << i)) != 0);
        }
    }

    if (mode == MODE_RAW){
        for (int i = 0; i < JOINT_COUNT; i++){
            arm.set_raw_pos((JointID)i, raw_position[i]);
            arm.set_raw_vel((JointID)i, raw_velocity[i]);
            arm.set_raw_acc((JointID)i, raw_acceleration[i]);
        }
        arm.end_effector.set_pos_norm(gripper);
    }
    else if (mode == MODE_ANGLE){
        for (int i = 0; i < JOINT_COUNT; i++){
            arm.set_pos((JointID)i, target_position[i]);
            arm.set_vel((JointID)i, target_velocity[i]);
        }
        arm.end_effector.set_pos_norm(gripper);
    }
    else {   // mode == MODE_HOLD
        for (int i = 0; i < JOINT_COUNT; i++){
            arm.set_raw_pos((JointID)i, arm.get_raw_pos((JointID)i));
        }
    }

    arm.motion_update();

    k_mutex_lock(&shared_mutex, K_FOREVER);   // Lock to publish what this cycle resolved to
    shared.torque_state = arm.torque_mask();
    shared.comm_errors = arm.comm_error_mask();
    shared.effective_mode = mode;
    shared.status = status
                  | (shared.torque_state ? STATUS_TORQUE : 0x00)
                  | (fresh ? 0x00 : STATUS_STALE);
    shared.loop_count++;
    k_mutex_unlock(&shared_mutex);

    // Account for bus communication and compute time, to ensure consistent loop rate
    unsigned long elapsed = millis() - cycle_start;
    if (elapsed < SERVO_PERIOD_MS){
        delay(SERVO_PERIOD_MS - elapsed);
    }
}
