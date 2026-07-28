#include "config.h"
#include "src/sensors/joystick.h"
#include "src/sensors/rotencoder.h"
#include "src/sensors/imu.h"
#include "src/peripherals/led_matrix.h"
#include "src/ble/ble.h"

Joystick joystick(JOY_X_PIN, JOY_Y_PIN, JOY_SW_PIN);
RotEncoder rotencoder(ENC_A_PIN, ENC_B_PIN, ENC_SW_PIN, 4);
IMU imu(IMU_SCL_PIN, IMU_SDA_PIN);
LedMatrix ledmatrix(DSPLY_DIN_PIN, DSPLY_CLK_PIN, DSPLY_CS_PIN);
BLEPublisher ble;

void setup() {
  joystick.initialize();
  rotencoder.initialize();
  imu.initialize();
  ledmatrix.initialize();
  ble.initialize();
}

void loop() {
  joystick.update();
  rotencoder.update();
  imu.update();

  int joy_x = joystick.x();
  int joy_y = joystick.y();
  bool joy_sw = joystick.pressed();

  int32_t enc_pos = rotencoder.pos();
  bool enc_sw = rotencoder.pressed();
  
  float roll = imu.roll();
  float pitch = imu.pitch();
  float yaw = imu.yaw();

  if (joy_sw){
    ledmatrix.setLed(0, 7, true);
  }
  else{
    ledmatrix.setLed(0, 7, false);
  }
  if (enc_sw){
    ledmatrix.setLed(0, 6, true);
  }
  else{
    ledmatrix.setLed(0, 6, false);
  }
  ledmatrix.barRow(7, joy_x, 950, -950, true);
  ledmatrix.barRow(6, joy_y, -950, 950, true);
  ledmatrix.barRow(4, roll, -90, 90, true);
  ledmatrix.barRow(3, pitch, -90, 90, true);
  ledmatrix.barRow(2, yaw, -90, 90, true);
  ledmatrix.show();

  ble.publish_data(enc_pos, joy_x, joy_y, roll, pitch, yaw, joy_sw, enc_sw);
  delay(10);
}
