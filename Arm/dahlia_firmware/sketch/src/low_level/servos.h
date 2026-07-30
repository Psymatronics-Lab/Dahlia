#pragma once

#include <HXServo.h>
#include <zephyr/drivers/pwm.h>
#include <zephyrPinctrl.h>

// The gripper runs on hardware PWM rather the Servo library. 
static const struct pwm_dt_spec GRIPPER_PWM = PWM_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);   // Index 0 is digital pin 2
constexpr int GRIPPER_PIN_INDEX = 0;   // PB3 in digital 2 Arduino pin-control state
constexpr int GRIPPER_PERIOD_NS = 20000000;   // 50Hz frame
constexpr int GRIPPER_OPEN_NS = 544000;   // Pulse width at full open
constexpr int GRIPPER_CLOSED_NS = 2400000;   // Pulse width at full close
constexpr float CLAMP_OPEN_MM = 55.0f;
constexpr float CLAMP_CLOSED_MM = 5.0f;

constexpr float RAD_PER_TICK = 0.0015f;

class EndEffector{
    public:
        EndEffector(){
            this->pulse_ns = GRIPPER_OPEN_NS;
            this->clamp_width = CLAMP_OPEN_MM;
        }

        /**
         * @brief Route the pin to its timer channel and open the clamp.
         * @return false if the timer rejected the 20ms period.
         */
        bool initialize(){
            zephyr::arduino::init_dev_apply_channel_pinctrl(GRIPPER_PWM.dev, GRIPPER_PIN_INDEX);
            bool ok = full_open();
            delay(100);   // Blocking delay for servo to traverse
            return ok;
        }

        bool full_open(){
            return set_closure(0.0f);
        }

        bool full_close(){
            return set_closure(1.0f);
        }

        /**
         * @brief Set the opening size of the clamp in millimeters, [5.0mm, 55.0mm].
         */
        bool set_pos(float width){
            return set_closure((CLAMP_OPEN_MM - width) / (CLAMP_OPEN_MM - CLAMP_CLOSED_MM));
        }

        /**
         * @brief Set the closure of the clamp, 0.0 fully open to 1.0 fully closed.
         */
        bool set_pos_norm(float closure){
            return set_closure(closure);
        }

        float pos(){
            return clamp_width;
        }

        int pos_raw(){
            return pulse_ns / 1000;
        }

    private:
        /**
         * @brief Drive the servo to a closure fraction, clamped to [0.0, 1.0].
         */
        bool set_closure(float closure){
            closure = constrain(closure, 0.0f, 1.0f);
            this->clamp_width = CLAMP_OPEN_MM - closure * (CLAMP_OPEN_MM - CLAMP_CLOSED_MM);
            this->pulse_ns = GRIPPER_OPEN_NS + lroundf(closure * (GRIPPER_CLOSED_NS - GRIPPER_OPEN_NS));
            return pwm_set_dt(&GRIPPER_PWM, GRIPPER_PERIOD_NS, this->pulse_ns) == 0;
        }

        int pulse_ns;   // Servo pulse width, [544us, 2400us]
        float clamp_width;   // Opening size of the clamp in millimeters, [5.0mm, 55.0mm]
};

struct JointConfig {
    int servo_id;
    int16_t min_pos;
    int16_t max_pos;   // Aligns with the position the servo is at for max_angle, not numerics
    float min_angle;
    float max_angle;   // Always numerically larger
};

struct JointCommand {
    int16_t target_pos;
    int16_t target_vel;
    int16_t target_acc;
};

struct JointState{
    int16_t current_pos;
    int16_t current_vel;
    bool moving;
    bool torque_on;
};

struct JointStateExternal{
    float current_angle;
    float current_velocity;
    bool moving;
    bool torque_on;
};

enum JointID {
    BASE = 0,
    SHOULDER,
    ELBOW,
    WRIST_PITCH,
    WRIST_ROLL,
    JOINT_COUNT
};

class DahliaArm{
    public:
        DahliaArm() : bus_servos(Serial1, 1000000){
            this->last_motion_update = 0;
        }

        /**
         * @brief Bring up the bus, the gripper and the joint states.
         * @return false if the gripper PWM channel could not be configured.
         */
        bool initialize(){
            bus_servos.begin();
            bool ok = end_effector.initialize();
            enable();
            refresh_state();
            last_motion_update = millis();
            return ok;
        }

        /**
         * @brief Set the target rotation angle of a servo (in radians).
         */
        void set_pos(JointID joint, float rad){
            joint_commands[joint].target_pos = rad_to_pos(joint, rad);
        }

        /**
         * @brief Set the maximum rotation angular velocity of a servo (in radians per second).
         */
        void set_vel(JointID joint, float radpersec){
            joint_commands[joint].target_vel = rad_to_vel(radpersec);
        }

        /**
         * @brief Set the maximum rotation angular acceleration of a servo (in radians per second^2).
         */
        void set_acc(JointID joint, float radpersq){
            joint_commands[joint].target_acc = rad_to_acc(radpersq);
        }

        /**
         * @brief Get the current rotation angle of a servo (in radians).
         */
        float get_pos(JointID joint){
            return pos_to_rad(joint, joint_states[joint].current_pos);
        }

        /**
         * @brief Get the full state of a joint.
         */
        JointStateExternal get_state(JointID joint){
            const JointState& joint_state = joint_states[joint];
            JointStateExternal out_state = {
                pos_to_rad(joint, joint_state.current_pos),
                vel_to_rad(joint_state.current_vel),
                joint_state.moving,
                joint_state.torque_on
            };
            return out_state;
        }

        /**
         * @brief Turns torque on for all servos.
         */
        void enable(){
            for(int i = 0; i < JOINT_COUNT; i++){
                bus_servos.enable_torque(joint_configs[i].servo_id);
                joint_states[i].torque_on = true;
            }
        }

        /**
         * @brief Turns torque off for all servos.
         */
        void disable(){
            for(int i = 0; i < JOINT_COUNT; i++){
                bus_servos.disable_torque(joint_configs[i].servo_id);
                joint_states[i].torque_on = false;
            }
        }

        /**
         * @brief Refresh high-level telemetry states of all servos.
         */
        void refresh_state(){
            uint8_t ids[JOINT_COUNT];
            int16_t telemetry[JOINT_COUNT][5];

            for(int i = 0; i < JOINT_COUNT; i++){
                ids[i] = (uint8_t)joint_configs[i].servo_id;
            }

            ServoStatus_t status = bus_servos.sync_read_cur_pos_ex(ids, JOINT_COUNT, telemetry);
            if (status.error_bits.bit_tx || status.error_bits.bit_rx){ return; }

            for(int i = 0; i < JOINT_COUNT; i++){
                joint_states[i].current_pos = telemetry[i][0];
                joint_states[i].current_vel = telemetry[i][1];
                joint_states[i].moving = (telemetry[i][1] != 0);
            }
        }

        /**
         * @brief Command motion for all servos.
         */
        void motion_update(){
            int16_t full_pos[JOINT_COUNT][4];

            unsigned long now = millis();
            float dt = (now - last_motion_update) * 0.001f;
            dt = min(dt, 0.1f);
            last_motion_update = now;

            for(int i = 0; i < JOINT_COUNT; i++){
                const JointCommand& joint_command = joint_commands[i];
                full_pos[i][0] = joint_configs[i].servo_id;
                full_pos[i][1] = joint_command.target_acc;
                full_pos[i][2] = joint_command.target_vel;
                int16_t max_step = lround(joint_command.target_vel * dt);
                int16_t target_pos = joint_command.target_pos;
                int16_t current_pos = joint_states[i].current_pos;
                int16_t step = constrain(target_pos - current_pos, -max_step, max_step);
                full_pos[i][3] = current_pos + step;
            }

            bus_servos.sync_write_pos_ex(full_pos, JOINT_COUNT);
        }

        EndEffector end_effector;
    private:
        /**
         * @brief Convert from ticks to radians for servo-relative position.
         */
        float pos_to_rad(JointID joint, int16_t pos) const {
            const JointConfig& joint_config = joint_configs[joint];
            float norm_pos = float(pos - joint_config.min_pos) / float(joint_config.max_pos - joint_config.min_pos);
            float rad = norm_pos * (joint_config.max_angle - joint_config.min_angle) + joint_config.min_angle;
            return rad;
        }

        /**
         * @brief Convert from radians to ticks for servo-relative position.
         */
        int16_t rad_to_pos(JointID joint, float rad) const {
            const JointConfig& joint_config = joint_configs[joint];
            if (rad > joint_config.max_angle){
                rad = joint_config.max_angle;
            }
            if (rad < joint_config.min_angle){
                rad = joint_config.min_angle;
            }
            float norm_angle = float(rad - joint_config.min_angle) / float(joint_config.max_angle - joint_config.min_angle);
            int16_t pos = lround(norm_angle * (joint_config.max_pos - joint_config.min_pos)) + joint_config.min_pos;
            return pos;
        }

        /**
         * @brief Convert from ticks/s to rad/s for angular velocity.
         */
        float vel_to_rad(int16_t vel) const { return vel * RAD_PER_TICK; }

        /**
         * @brief Convert from rad/s to ticks/s for angular velocity.
         */
        int16_t rad_to_vel(float rad) const {
            int16_t vel = lround(rad / RAD_PER_TICK);
            if (vel > 3400){ vel = 3400; }
            else if (vel < 0){ vel = 0; }
            return vel;
        }

        /**
         * @brief Convert from ticks/s^2 to rad/s^2 for angular acceleration.
         */
        float acc_to_rad(int16_t acc) const { return acc * RAD_PER_TICK; }

        /**
         * @brief Convert from rad/s^2 to ticks/s^2 for angular acceleration.
         */
        int16_t rad_to_acc(float rad) const {
            int16_t acc = lround(rad / RAD_PER_TICK);
            if (acc > 254){ acc = 254; }
            else if (acc < 0){ acc = 0; }
            return acc;
        }

        unsigned long last_motion_update;
        HXServo bus_servos;
        JointConfig joint_configs[JOINT_COUNT] = {
            {1, 1250, -1250, -1.917f, 1.917f},   // BASE
            {2, -200, 2200, -0.307f, 3.375f},   // SHOULDER
            {3, 2260, 0, 0.0f, 3.467f},   // ELBOW
            {4, 1000, -1000, -1.534f, 1.534f},   // WRIST_PITCH
            {5, 2700, 800, -1.457f, 1.457f}    // WRIST_ROLL
        };
        JointCommand joint_commands[JOINT_COUNT] = {
            {0, 3400, 254},
            {0, 3400, 254},
            {2250, 3400, 254},
            {0, 3400, 254},
            {1750, 3400, 254}
        };
        JointState joint_states[JOINT_COUNT] = {
            {0, 0, false, false},
            {0, 0, false, false},
            {2250, 0, false, false},
            {0, 0, false, false},
            {1750, 0, false, false}
        };
};