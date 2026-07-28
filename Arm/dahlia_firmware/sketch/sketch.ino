#include <Servo.h>
#include <HXServo.h>
#include "Arduino_RouterBridge.h"

Servo endEffector;
HXServo busServos(Serial1, 1000000);   // bind to the UART wired to the BusLinker

const int SW_pin = 7;
const int X_pin = A0;
const int Y_pin = A1;

bool EF_open = true;
int shoulder_pos = 0;
int elbow_pos = 2250;

void setup() {
  Serial.begin(115200);            // USB monitor (debug only)
  busServos.begin();                   // open the servo bus

  ServoStatus_t st = busServos.ping(1);
  if (!st.error_bits.bit_rx) {
    Serial.print("Servo online, ID = ");
    Serial.println(st.id);
  }
  busServos.enable_torque(1);
  busServos.enable_torque(2);
  busServos.enable_torque(3);
  busServos.enable_torque(4);
  busServos.enable_torque(5);

  endEffector.attach(3);
  pinMode(SW_pin, INPUT_PULLUP);

  busServos.write_pos(1, 0);
  busServos.write_pos(2, shoulder_pos);
  busServos.write_pos(3, elbow_pos);
  busServos.write_pos(4, 300);
  busServos.write_pos(5,1750);
}

void loop() {
  // if (digitalRead(SW_pin) == LOW){
  //   EF_open = !EF_open;
  //   if (EF_open){
  //     endEffector.write(10);
  //   }
  //   else{
  //     endEffector.write(40);
  //   }
  // }
  // int xValue = analogRead(X_pin);
  // int yValue = analogRead(Y_pin);

  // int mappedX = map(xValue, 0, 1023, -10, 5) * 2;
  // int mappedY = map(yValue, 0, 1023, -10, 5) * 2;
  // shoulder_pos += mappedX;
  // elbow_pos += mappedY;
  // Serial.println(shoulder_pos);
  // Serial.println(elbow_pos);
  // busServos.write_pos(2, shoulder_pos);
  // busServos.write_pos(3, elbow_pos);
  delay(100);
}