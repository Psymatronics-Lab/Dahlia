/*
 * Change any bus servo at one ID to another ID, continously.
 *
 * The loop polls for a servo at the old ID and converts it as soon as one appears.
 * Useful for fixing Servo Studio GUI bugs on ID = 0 servos.
 */
#include <HXServo.h>

HXServo busServos(Serial1, 1000000);   // bind to the UART wired to the BusLinker

const uint8_t OLD_ID = 0;
const uint8_t NEW_ID = 1;

void setup() {
  Serial.begin(115200);
  busServos.begin();
  delay(500);
  Serial.println("Polling for a servo at ID 0...");
}

void loop() {
  ServoStatus_t st = busServos.ping(OLD_ID);

  if (st.error_bits.bit_rx) {
    // Either no servo is plugged or was already converted
    Serial.print("Waiting for a servo at ID: ");
    Serial.println(OLD_ID);
    delay(500);
  }
  else{
    Serial.println("Servo found at ID 0, changing ID...");

    uint8_t data = NEW_ID;
    busServos.general_write(OLD_ID, REG_ID, &data, 1);
    delay(100);

    st = busServos.ping(NEW_ID);
    if (st.error_bits.bit_rx) {
      Serial.println("ID change failed -- servo does not answer at the new ID");
    } else {
      Serial.print("Done, servo now answers as ID ");
      Serial.println(st.id);
    }
    delay(500);
  }
}
