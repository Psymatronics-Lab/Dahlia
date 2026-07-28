#include <Servo.h>
#include <HXServo.h>

constexpr int SIGPIN = 3

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
    int min_pos;
    int max_pos;
    float min_angle;
    float max_angle;
    bool inverted;
};

struct JointState {
    float target_angle;
    float target_speed;
    float current_angle;
    float current_speed;
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

        void initialize(){
            bus_servos.begin();
            for(int i = 1; i <= 5; i++){
                bus_servos.enable_torque(i);
            }

            // Base

            // Shoulder

            // Elbow

            // Wrist Pitch

            // Wrist Roll

        }

        /**
         * @brief Set the target rotation angle of a servo.
         */
        void set_pos(JointID joint, float radians){

        }

        /**
         * @brief Get the current rotation angle of a servo.
         */
        float get_pos(JointID joint){

        }

        /**
         * @brief Set the target rotation speed of a servo.
         */
        void set_speed(JointID joint, float speed){
            
        }

        /**
         * @brief Get the state of a joint.
         */
        JointState get_state(JointID joint){

        }

        /**
         * @brief Turns torque on for all servos.
         */
        void enable(){

        }

        /**
         * @brief Turns torque off for all servos.
         */
        void disable(){

        }

        EndEffector end_effector;
    private:
        HXServo bus_servos;
        JointConfig joint_configs[JOINT_COUNT];
        JointState joint_states[JOINT_COUNT];
};