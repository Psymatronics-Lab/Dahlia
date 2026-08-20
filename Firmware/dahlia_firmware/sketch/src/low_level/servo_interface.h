#include <HXServo.h>
#include <zephyr/drivers/pwm.h>
#include <zephyrPinctrl.h>

constexpr int SIGPIN = 3;
static const struct pwm_dt_spec END_EFFECTOR_PWM = PWM_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1);
constexpr int END_EFFECTOR_PWM_PIN_INDEX = 0;
constexpr int END_EFFECTOR_PERIOD_NS = 20000000;
constexpr int END_EFFECTOR_OPEN_NS = 544000;
constexpr int END_EFFECTOR_CLOSED_NS = 2400000;
constexpr float RAD_PER_TICK = 0.0015340f;   // 2*pi/4096, the resolution of the HX family

// Initial servo command config to move to the zero pose.
constexpr int INIT_PERIOD_MS = 5;
constexpr unsigned long INIT_TIMEOUT_MS = 6000;
constexpr int16_t INIT_TOLERANCE = 12;
constexpr int16_t INIT_VEL = 900;
constexpr int16_t INIT_ACC = 40;

// Servo bus ceilings, from HXServo::write_pos_ex
constexpr int16_t MAX_TICK_VEL = 3400;
constexpr uint8_t MAX_TICK_ACC = 254;

class EndEffector{
    public:
        EndEffector(int)
            : raw_pos(0),
              clamp_width(0.0f)
            {}

        void initialize(){
            zephyr::arduino::init_dev_apply_channel_pinctrl(END_EFFECTOR_PWM.dev, END_EFFECTOR_PWM_PIN_INDEX);
            raw_pos = 0;
            clamp_width = 55.0f;
            write_raw();
            delay(100);   // Blocking delay for servo to traverse
        }

        void full_open(){
            raw_pos = 0;
            clamp_width = 55.0f;
            write_raw();
        }

        void full_close(){
            raw_pos = 180;
            clamp_width = 5.0f;
            write_raw();
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
            write_raw();
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
            write_raw();
        }

        float pos(){
            return clamp_width;
        }

        int pos_raw(){
            return raw_pos;
        }

    private:
        void write_raw(){
            int pulse_ns = END_EFFECTOR_OPEN_NS + lroundf(raw_pos / 180.0f * (END_EFFECTOR_CLOSED_NS - END_EFFECTOR_OPEN_NS));
            pwm_set_dt(&END_EFFECTOR_PWM, END_EFFECTOR_PERIOD_NS, pulse_ns);
        }

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
    int16_t current_load;
    uint8_t voltage;   // 0.1 V per count
    uint8_t temperature;   // Degrees Celsius
    bool moving;
    bool torque_on;
    bool comm_ok;   // Whether the last telemetry read for this joint succeeded
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
          end_effector(SIGPIN)
        {}

        bool initialize(){
            bus_servos.begin();

            // Built from joint_configs to maintain single source of truth
            for(int i = 0; i < JOINT_COUNT; i++){
                servo_ids[i] = (uint8_t)joint_configs[i].servo_id;
            }

            end_effector.initialize();

            refresh_state();
            for(int i = 0; i < JOINT_COUNT; i++){
                joint_commands[i].target_pos = joint_states[i].current_pos;
            }

            enable();
            return zero();
        }

        /**
         * @brief Drive every joint to its zero angle, blocking until the arm arrives.
         * @return True if every joint reached zero before the timeout.
         */
        bool zero(){
            for(int i = 0; i < JOINT_COUNT; i++){
                joint_commands[i].target_pos = zero_pos((JointID)i);
                joint_commands[i].target_vel = INIT_VEL;
                joint_commands[i].target_acc = INIT_ACC;
            }

            unsigned long deadline = millis() + INIT_TIMEOUT_MS;
            bool arrived = false;

            while (!arrived && millis() < deadline){
                refresh_state();
                motion_update();
                arrived = at_zero();
                delay(INIT_PERIOD_MS);
            }

            // Hand the running limits back, as MODE_HOLD never writes a velocity
            for(int i = 0; i < JOINT_COUNT; i++){
                joint_commands[i].target_vel = MAX_TICK_VEL;
                joint_commands[i].target_acc = MAX_TICK_ACC;
            }

            return arrived;
        }

        /**
         * @brief Whether every joint is sitting at its zero angle.
         */
        bool at_zero(){
            for(int i = 0; i < JOINT_COUNT; i++){
                // Held in a variable, as Arduino's abs() is a macro and re-evaluates
                int16_t error = joint_states[i].current_pos - zero_pos((JointID)i);

                if (abs(error) > INIT_TOLERANCE){
                    return false;
                }
            }
            return true;
        }

        /**
         * @brief Tick position of a joint's zero angle, straight off its calibration.
         */
        int16_t zero_pos(JointID joint) const { return rad_to_pos(joint, 0.0f); }

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
         * @brief Set the target position of a servo directly in ticks.
         */
        void set_raw_pos(JointID joint, int16_t pos){
            joint_commands[joint].target_pos = constrain(pos, tick_min(joint), tick_max(joint));
        }

        /**
         * @brief Set the maximum slew rate of a servo in ticks per second.
         */
        void set_raw_vel(JointID joint, int16_t vel){
            joint_commands[joint].target_vel = constrain(vel, (int16_t)0, MAX_TICK_VEL);
        }

        /**
         * @brief Set the acceleration ramp of a servo in ticks per second squared.
         */
        void set_raw_acc(JointID joint, uint8_t acc){
            joint_commands[joint].target_acc = min(acc, MAX_TICK_ACC);
        }

        /**
         * @brief Get a raw or diagnostic data value from a servo.
         */
        int16_t get_raw_pos(JointID joint){ return joint_states[joint].current_pos; }
        int16_t get_raw_vel(JointID joint){ return joint_states[joint].current_vel; }
        int16_t get_raw_load(JointID joint){ return joint_states[joint].current_load; }
        uint8_t get_voltage(JointID joint){ return joint_states[joint].voltage; }
        uint8_t get_temperature(JointID joint){ return joint_states[joint].temperature; }

        /**
         * @brief Get raw tick minimum of servo travel, may not align with min angle.
         * */
        int16_t tick_min(JointID joint){
            const JointConfig& c = joint_configs[joint];
            return min(c.min_pos, c.max_pos);
        }

        /**
         * @brief Get raw tick maximum of servo travel, may not align with max angle.
         */
        int16_t tick_max(JointID joint){
            const JointConfig& c = joint_configs[joint];
            return max(c.min_pos, c.max_pos);
        }

        /**
         * @brief Turn torque on or off for one servo.
         */
        void set_torque(JointID joint, bool on){
            // no-op to prevent excessive re-writing on the UART bus
            if (joint_states[joint].torque_on == on){ return; }

            ServoStatus_t status = on ? bus_servos.enable_torque(joint_configs[joint].servo_id)
                                      : bus_servos.disable_torque(joint_configs[joint].servo_id);

            if (status.error_bits.bit_tx || status.error_bits.bit_rx){ return; }

            joint_states[joint].torque_on = on;

            // prevents snapping back to original position immediately on torque restore
            if (on){ joint_commands[joint].target_pos = joint_states[joint].current_pos; }
        }

        /**
         * @brief Turns torque on for all servos.
         */
        void enable(){
            for(int i = 0; i < JOINT_COUNT; i++){
                set_torque((JointID)i, true);
            }
        }

        /**
         * @brief Turns torque off for all servos.
         */
        void disable(){
            for(int i = 0; i < JOINT_COUNT; i++){
                set_torque((JointID)i, false);
            }
        }

        /**
         * @brief Bit i set if joint i is holding.
         * */
        uint8_t torque_mask(){
            uint8_t mask = 0;
            for(int i = 0; i < JOINT_COUNT; i++){
                mask |= joint_states[i].torque_on ? (1u << i) : 0;
            }
            return mask;
        }

        /**
         * @brief Bit i set if joint i missed the last telemetry read.
         * */
        uint8_t comm_error_mask(){
            uint8_t mask = 0;
            for(int i = 0; i < JOINT_COUNT; i++){
                mask |= joint_states[i].comm_ok ? 0 : (1u << i);
            }
            return mask;
        }

        /**
         * @brief Refresh high-level telemetry states of all servos.
         */
        void refresh_state(){
            int16_t telemetry[JOINT_COUNT][5];   // {position, speed, load, voltage, temperature}
            uint8_t answered = 0;

            bus_servos.sync_read_cur_pos_ex(servo_ids, JOINT_COUNT, telemetry, &answered);

            for(int i = 0; i < JOINT_COUNT; i++){
                if (!(answered & (1u << i))){
                    joint_states[i].comm_ok = false;
                    continue;   // keep last state on a failed read
                }

                joint_states[i].comm_ok = true;
                joint_states[i].current_pos = telemetry[i][0];
                joint_states[i].current_vel = telemetry[i][1];
                joint_states[i].current_load = telemetry[i][2];
                joint_states[i].voltage = (uint8_t)telemetry[i][3];
                joint_states[i].temperature = (uint8_t)telemetry[i][4];
                joint_states[i].moving = (telemetry[i][1] != 0);
            }
        }

        /**
         * @brief Command motion for all servos.
         */
        void motion_update(){
            int16_t full_pos[JOINT_COUNT][4];

            for(int i = 0; i < JOINT_COUNT; i++){
                // prevents position snapping after no-torque manual movement of the arm
                if (!joint_states[i].torque_on){
                    joint_commands[i].target_pos = joint_states[i].current_pos;
                }

                const JointCommand& joint_command = joint_commands[i];
                full_pos[i][0] = joint_configs[i].servo_id;
                full_pos[i][1] = joint_command.target_acc;
                full_pos[i][2] = joint_command.target_vel;
                full_pos[i][3] = joint_command.target_pos;
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
            long vel = lround(rad / RAD_PER_TICK);
            if (vel > MAX_TICK_VEL){ vel = MAX_TICK_VEL; }
            else if (vel < 0){ vel = 0; }
            return (int16_t)vel;
        }

        /**
         * @brief Convert from ticks/s^2 to rad/s^2 for angular acceleration.
         */
        float acc_to_rad(int16_t acc) const { return acc * RAD_PER_TICK; }

        /**
         * @brief Convert from rad/s^2 to ticks/s^2 for angular acceleration.
         */
        int16_t rad_to_acc(float rad) const {
            long acc = lround(rad / RAD_PER_TICK);
            if (acc > MAX_TICK_ACC){ acc = MAX_TICK_ACC; }
            else if (acc < 0){ acc = 0; }
            return (int16_t)acc;
        }

        uint8_t servo_ids[JOINT_COUNT] = {0};   // Filled from joint_configs by initialize()
        HXServo bus_servos;
        JointConfig joint_configs[JOINT_COUNT] = {
            {1, -1250, 1250, -1.917f, 1.917f},   // BASE
            {2, 2200, -200, -3.375f, 0.307f},   // SHOULDER
            {3, 2250, 0, 0.0f, 3.451f},   // ELBOW
            {4, 1024, -1024, -1.534f, 1.534f},   // WRIST_PITCH
            {5, 2700, 800, -1.457f, 1.457f}    // WRIST_ROLL
        };
        JointCommand joint_commands[JOINT_COUNT] = {
            {0, 3400, 254},
            {0, 3400, 254},
            {2250, 3400, 254},
            {0, 3400, 254},
            {1750, 3400, 254}
        };
        // {position, velocity, load, voltage, temperature, moving, torque_on, comm_ok}
        JointState joint_states[JOINT_COUNT] = {
            {0, 0, 0, 0, 0, false, false, false},
            {0, 0, 0, 0, 0, false, false, false},
            {2250, 0, 0, 0, 0, false, false, false},
            {0, 0, 0, 0, 0, false, false, false},
            {1750, 0, 0, 0, 0, false, false, false}
        };
};
