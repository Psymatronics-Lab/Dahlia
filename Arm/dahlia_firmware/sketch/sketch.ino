#include "src/low_level/servo_interface.h"
#include "src/spi/spi_service.h"

static_assert(JOINT_COUNT == SPI_JOINT_COUNT, "Packet joint count must match the arm joint count.");

constexpr int SERVO_PERIOD_MS = 5;   // 200 Hz servo loop
constexpr int SPI_STACK_SIZE = 2048;
constexpr int SPI_PRIORITY = 5;

DahliaArm arm;
SPIService<FeedbackPacket, CommandPacket> spi;

/**
 * @brief The only state shared between the SPI thread and the servo loop.
 */
struct SharedState {
    float target_position[JOINT_COUNT];   // Radians
    float target_velocity[JOINT_COUNT];   // Radians per second
    float gripper;   // Normalized clamp closure, [0.0, 1.0]
    float current_position[JOINT_COUNT];
    float current_velocity[JOINT_COUNT];
    uint8_t status;
    uint8_t sequence;
};

SharedState shared;
K_MUTEX_DEFINE(shared_mutex);

K_THREAD_STACK_DEFINE(spi_stack, SPI_STACK_SIZE);
struct k_thread spi_thread;

/**
 * @brief SPI thread. Publishes the newest feedback and takes in the newest command,
 *        running independently of the servo loop so a slow transfer never delays motion.
 */
void spi_task(void*, void*, void*){
    while (true){
        k_mutex_lock(&shared_mutex, K_FOREVER);   // Lock when reading current state for telemetry
        spi.tx.magic = FEEDBACK_MAGIC;
        spi.tx.sequence = shared.sequence;
        for (int i = 0; i < JOINT_COUNT; i++){
            spi.tx.current_position[i] = shared.current_position[i];
            spi.tx.current_velocity[i] = shared.current_velocity[i];
        }
        spi.tx.status = shared.status;
        k_mutex_unlock(&shared_mutex);

        if (!spi.transfer() || spi.rx.magic != COMMAND_MAGIC){
            continue;
        }

        k_mutex_lock(&shared_mutex, K_FOREVER);   // Lock when receiving new command data
        for (int i = 0; i < JOINT_COUNT; i++){
            shared.target_position[i] = spi.rx.target_position[i];
            shared.target_velocity[i] = spi.rx.target_velocity[i];
        }
        shared.gripper = spi.rx.gripper / 255.0f;
        shared.sequence = spi.rx.sequence;
        k_mutex_unlock(&shared_mutex);
    }
}

void setup(){
    Serial.begin(115200);
    arm.initialize();

    // Hold the startup pose until the host sends its first command
    for (int i = 0; i < JOINT_COUNT; i++){
        shared.target_position[i] = arm.get_pos((JointID)i);
        shared.target_velocity[i] = 0.0f;
    }
    shared.gripper = 0.0f;

    spi.initialize();
    k_thread_create(&spi_thread, spi_stack, K_THREAD_STACK_SIZEOF(spi_stack),
                    spi_task, NULL, NULL, NULL, SPI_PRIORITY, 0, K_NO_WAIT);
}

void loop() {
    arm.refresh_state();

    float target_position[JOINT_COUNT];
    float target_velocity[JOINT_COUNT];
    float gripper;
    uint8_t status = 0;

    k_mutex_lock(&shared_mutex, K_FOREVER);   // Lock to prevent SPI updates when reading from state
    for (int i = 0; i < JOINT_COUNT; i++){
        JointStateExternal state = arm.get_state((JointID)i);
        shared.current_position[i] = state.current_angle;
        shared.current_velocity[i] = state.current_velocity;
        status |= (state.torque_on ? 0x01 : 0x00) | (state.moving ? 0x02 : 0x00);
        target_position[i] = shared.target_position[i];
        target_velocity[i] = shared.target_velocity[i];
    }
    shared.status = status;
    gripper = shared.gripper;
    k_mutex_unlock(&shared_mutex);

    for (int i = 0; i < JOINT_COUNT; i++){
        arm.set_pos((JointID)i, target_position[i]);
        arm.set_vel((JointID)i, target_velocity[i]);
    }
    arm.end_effector.set_pos_norm(gripper);
    arm.motion_update();

    delay(SERVO_PERIOD_MS);
}
