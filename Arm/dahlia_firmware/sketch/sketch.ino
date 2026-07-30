#include "src/low_level/servos.h"
#include "Arduino_RouterBridge.h"

DahliaArm arm;

void setup(){
  Serial.begin(115200);
  if (!arm.initialize()){
    Serial.println("Gripper PWM setup failed");
  }
}

void loop() {
  arm.refresh_state();
  Serial.println(arm.get_pos(BASE));
  Serial.println(arm.get_pos(SHOULDER));
  Serial.println(arm.get_pos(ELBOW));
  Serial.println(arm.get_pos(WRIST_PITCH));
  Serial.println(arm.get_pos(WRIST_ROLL));
  arm.set_pos(BASE, 90.0 * DEG_TO_RAD);
  arm.motion_update();
  delay(50);
}