/**
 * BasicControl.ino
 * -----------------
 * Minimal example for the HXServo library, driving a Hiwonder HX-series
 * magnetic-encoder bus servo (HX-10HM / HX-30HM / HX-65HM / ...) through a
 * BusLinker V3.0 (or any transparent TTL bus adapter).
 *
 * WIRING (Arduino UNO Q  <->  BusLinker V3.0)
 *   Arduino Serial1 TX  ->  BusLinker RX
 *   Arduino Serial1 RX  ->  BusLinker TX
 *   Arduino GND         ->  BusLinker GND   (common ground is REQUIRED)
 *   Servo power comes from the BusLinker's own DC input, NOT from the Arduino.
 *
 * NOTES
 *   - SERVO_SERIAL must be the hardware UART wired to the BusLinker. On the
 *     UNO Q the pin-0/1 header UART is Serial1; change it if you wire a
 *     different port.
 *   - The bus baud rate must match on all three: servo, BusLinker, and the
 *     value passed here. HX-HM servos ship at 1,000,000 bps.
 *   - Set HX_DEBUG to 1 (top of HXServo.h) to watch every TX/RX frame while
 *     bringing the bus up.
 */
#include <HXServo.h>

#define SERVO_SERIAL Serial1     // UART wired to the BusLinker
#define SERVO_ID     1           // change to your servo's ID

HXServo servo(SERVO_SERIAL, 1000000);

void setup() {
  Serial.begin(115200);          // USB serial monitor
  servo.begin();                 // opens SERVO_SERIAL at the configured baud

  delay(100);

  // 1) Confirm the servo is on the bus. Broadcast (0xFE) makes a lone servo
  //    reply with its own ID -- handy when you don't know the ID yet.
  ServoStatus_t st = servo.ping(SERVO_ID);
  if (st.error_bits.bit_rx) {
    Serial.println(F("No reply from servo -- check wiring, baud, and ID."));
  } else {
    Serial.print(F("Servo online, ID = "));
    Serial.println(st.id);
  }

  // 2) Torque must be enabled before the servo will hold or move to a target.
  servo.enable_torque(SERVO_ID);
}

void loop() {
  int16_t pos;

  // Move toward one end with an acceleration and speed limit, then read back
  // the live position. Position is signed; 0 is the calibrated center.
  servo.write_pos_ex(SERVO_ID, /*acc=*/50, /*speed=*/1000, /*pos=*/ 2048);
  delay(1000);
  servo.read_pos(SERVO_ID, &pos);
  Serial.print(F("pos = ")); Serial.println(pos);

  servo.write_pos_ex(SERVO_ID, /*acc=*/50, /*speed=*/1000, /*pos=*/-2048);
  delay(1000);
  servo.read_pos(SERVO_ID, &pos);
  Serial.print(F("pos = ")); Serial.println(pos);
}
