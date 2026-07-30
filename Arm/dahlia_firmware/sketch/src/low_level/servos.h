#include <Servo.h>
#include <HXServo.h>

constexpr int SIGPIN = 3;
constexpr float RAD_PER_TICK = 0.0015f;

class EndEffector{
    public:
        EndEffector(int SIGPIN)
            : SIGPIN(SIGPIN),
              raw_pos(0),
              clamp_width(0.0f)
            {}

        void initialize(){
            drive_servo.attach(SIGPIN);
            raw_pos = 0;
            clamp_width = 55.0f;
            drive_servo.write(raw_pos);
            delay(100);   // Blocking delay for servo to traverse
        }

        void full_open(){
            raw_pos = 0;
            clamp_width = 55.0f;
            drive_servo.write(raw_pos);
        }

        void full_close(){
            raw_pos = 180;
            clamp_width = 5.0f;
            drive_servo.write(raw_pos);
        }

        void set_pos(float width){
            if (width >= 55.0f){
                full_open();
                return;
            }
            if (width <= 5.0f){
                full_close();
                return;
            }
            raw_pos = lroundf((width - 5.0f) / 50.0f * 180.0f);
            clamp_width = width;
            drive_servo.write(raw_pos);
        }

        void set_pos_norm(float width){
            if (width >= 1.0f){
                full_close();
                return;
            }
            if (width <= 0.0f){
                full_open();
                return;
            }
            raw_pos = lroundf(width * 180.0f);
            clamp_width = width * 50.0f + 5.0f;
            drive_servo.write(raw_pos);
        }

        float pos(){
            return clamp_width;
        }

        int pos_raw(){
            return raw_pos;
        }

    private:
        int SIGPIN;
        Servo drive_servo;
        int raw_pos;   // Raw servo position, [0, 180]
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
        DahliaArm()
        : bus_servos(Serial1, 1000000),
          end_effector(SIGPIN),
          last_motion_update(0)
        {}

        void initialize(){
            bus_servos.begin();
            end_effector.initialize();
            enable();
            refresh_state();
            last_motion_update = millis();
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
            for(int i = 0; i < JOINT_COUNT; i++){
                int16_t position = 0;
                int16_t speed = 0;
                bus_servos.read_pos_speed(joint_configs[i].servo_id, &position, &speed);
                joint_states[i].current_pos = position;
                joint_states[i].current_vel = speed;
                joint_states[i].moving = (speed != 0);
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