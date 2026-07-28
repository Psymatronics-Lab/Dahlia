/**
 * Telemetry.ino
 * -------------
 * Reads and prints live feedback from Hiwonder HX-series bus servos with the
 * HXServo library. Serial bus servos report far more than a PWM servo can:
 * position, speed, load, current, supply voltage, temperature, and a moving
 * flag. Watching current/temperature is how you catch an overload on a
 * high-torque servo (e.g. HX-65HM) before it faults.
 *
 * This sketch demonstrates both:
 *   - per-servo reads (read_pos, read_load, read_temperature, ...), and
 *   - sync_read_cur_pos_ex(), which pulls {pos, speed, load, voltage, temp}
 *     from several servos efficiently.
 *
 * The servos do NOT need torque enabled to be read. In fact, disabling torque
 * lets you backdrive a joint by hand and watch the position change live -- a
 * quick way to confirm the bus and sensor are working.
 *
 * WIRING: see BasicControl.ino (Serial1 TX->RX, RX->TX, common GND).
 */
#include <HXServo.h>

#define SERVO_SERIAL Serial1
#define NUM_SERVOS   3

HXServo servo(SERVO_SERIAL, 1000000);

const uint8_t SERVO_IDS[NUM_SERVOS] = { 1, 2, 3 };

// Print the full feedback set for one servo using the individual read calls.
void printOne(uint8_t id) {
  int16_t  pos = 0, speed = 0, load = 0;
  uint16_t current = 0;
  uint8_t  voltage = 0, temp = 0, moving = 0;

  ServoStatus_t st = servo.read_pos(id, &pos);
  if (st.error_bits.bit_rx) {                 // no reply -> nothing to trust
    Serial.print(F("ID "));
    Serial.print(id);
    Serial.println(F(": no reply (check wiring / baud / ID)"));
    return;
  }

  servo.read_speed(id, &speed);
  servo.read_load(id, &load);
  servo.read_current(id, &current);
  servo.read_voltage(id, &voltage);           // 0.1 V per count
  servo.read_temperature(id, &temp);          // degrees C
  servo.read_moving_status(id, &moving);

  Serial.print(F("ID "));        Serial.print(id);
  Serial.print(F(" | pos "));    Serial.print(pos);
  Serial.print(F(" | spd "));    Serial.print(speed);
  Serial.print(F(" | load "));   Serial.print(load);
  Serial.print(F(" | I "));      Serial.print(current);
  Serial.print(F(" | V "));      Serial.print(voltage / 10.0, 1);
  Serial.print(F(" | T "));      Serial.print(temp);
  Serial.print(F("C | "));       Serial.println(moving ? F("moving") : F("stopped"));

  // Example fault guard: shed torque if a servo runs hot.
  if (temp > 65) {
    servo.disable_torque(id);
    Serial.print(F("  -> ID "));
    Serial.print(id);
    Serial.println(F(" over 65C: torque disabled"));
  }
}

// Pull {pos, speed, load, voltage, temp} from every servo in fewer round-trips.
void printAllSync() {
  uint8_t ids[NUM_SERVOS];
  int16_t data[NUM_SERVOS][5];   // columns: pos, speed, load, voltage, temp
  for (uint8_t i = 0; i < NUM_SERVOS; i++) ids[i] = SERVO_IDS[i];

  ServoStatus_t st = servo.sync_read_cur_pos_ex(ids, NUM_SERVOS, data);
  if (st.error_bits.bit_tx || st.error_bits.bit_rx) {
    Serial.println(F("sync read failed"));
    return;
  }

  Serial.println(F("-- sync read --"));
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print(F("ID "));      Serial.print(ids[i]);
    Serial.print(F(" | pos "));  Serial.print(data[i][0]);
    Serial.print(F(" | spd "));  Serial.print(data[i][1]);
    Serial.print(F(" | load ")); Serial.print(data[i][2]);
    Serial.print(F(" | V "));    Serial.print(data[i][3] / 10.0, 1);
    Serial.print(F(" | T "));    Serial.print(data[i][4]);
    Serial.println(F("C"));
  }
}

void setup() {
  Serial.begin(115200);
  servo.begin();
  delay(100);

  // Read-only demo: leave torque OFF so joints can be moved by hand while the
  // position readout updates. Enable torque here if you want the servos held.
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servo.disable_torque(SERVO_IDS[i]);
  }
}

void loop() {
  // Per-servo detailed view.
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    printOne(SERVO_IDS[i]);
  }
  Serial.println();

  // Compact multi-servo view in fewer transactions.
  printAllSync();
  Serial.println();

  delay(500);
}
