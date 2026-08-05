/**
 * SyncMotion.ino
 * --------------
 * Drives several Hiwonder HX-series bus servos in synchronized motion with the
 * HXServo library, through a BusLinker V3.0 (or any transparent TTL adapter).
 *
 * It shows the two ways to move multiple joints together:
 *
 *   1) sync_write_pos_ex() -- one packet updates every servo at once. Lowest
 *      bus traffic; the servos begin their moves as the single frame lands.
 *
 *   2) write_reg_pos_ex() + reg_action() -- "stage, then trigger". Each servo
 *      is loaded with its target individually (buffered, not executed), then a
 *      single broadcast reg_action() makes them all start on the same tick.
 *      Use this when you must prepare targets over several frames but still
 *      want a simultaneous start.
 *
 * WIRING (Arduino UNO Q  <->  BusLinker V3.0)
 *   Serial1 TX -> BusLinker RX,  Serial1 RX -> BusLinker TX,  GND -> GND
 *   Power the servos from the BusLinker's DC input, not the Arduino.
 *
 * SETUP: give each servo a UNIQUE ID first (connect them one at a time and set
 * the ID) and list those IDs in SERVO_IDS below.
 */
#include <HXServo.h>

#define SERVO_SERIAL Serial1
#define NUM_SERVOS   3

HXServo servo(SERVO_SERIAL, 1000000);

const uint8_t SERVO_IDS[NUM_SERVOS] = { 1, 2, 3 };

// Two poses to alternate between: one column per servo. Position is signed,
// 0 = calibrated center. Tune these to your mechanism's safe range.
const int16_t POSE_A[NUM_SERVOS] = {  2048, -1500,  1000 };
const int16_t POSE_B[NUM_SERVOS] = { -2048,  1500, -1000 };

const uint8_t MOVE_ACC   = 50;    // acceleration ramp [0..254]
const int16_t MOVE_SPEED = 1200;  // speed limit [-3400..3400]

// --- Method 1: single-packet synchronized move -------------------------------
void moveAllSync(const int16_t pose[NUM_SERVOS]) {
  // Each row is {id, acc, speed, pos}, exactly what sync_write_pos_ex expects.
  int16_t frame[NUM_SERVOS][4];
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    frame[i][0] = SERVO_IDS[i];
    frame[i][1] = MOVE_ACC;
    frame[i][2] = MOVE_SPEED;
    frame[i][3] = pose[i];
  }
  servo.sync_write_pos_ex(frame, NUM_SERVOS);   // one frame, all servos move
}

// --- Method 2: stage individually, then trigger together ---------------------
void moveAllStaged(const int16_t pose[NUM_SERVOS]) {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    // Buffered write: loaded but NOT executed yet.
    servo.write_reg_pos_ex(SERVO_IDS[i], MOVE_ACC, MOVE_SPEED, pose[i]);
  }
  // Broadcast trigger: every staged servo starts on this tick.
  servo.reg_action(BROADCAST_ID);
}

void setup() {
  Serial.begin(115200);
  servo.begin();
  delay(100);

  // Bring every servo online and enable torque so they will hold/move.
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    ServoStatus_t st = servo.ping(SERVO_IDS[i]);
    if (st.error_bits.bit_rx) {
      Serial.print(F("WARNING: no reply from servo ID "));
      Serial.println(SERVO_IDS[i]);
    }
    servo.enable_torque(SERVO_IDS[i]);
  }
}

void loop() {
  Serial.println(F("Pose A (single-packet sync)"));
  moveAllSync(POSE_A);
  delay(1500);

  Serial.println(F("Pose B (staged + broadcast trigger)"));
  moveAllStaged(POSE_B);
  delay(1500);
}
