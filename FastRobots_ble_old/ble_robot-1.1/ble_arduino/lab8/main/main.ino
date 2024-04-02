#include "ble_arduino.h"
#include "IMU.h"
#include "tof.h"
#include "motor.h"

void setup() {
  Serial.begin(115200);
  ping_led();

  ble_setup();
  imu_setup();
  tof_setup();
  motor_setup();
}

void loop() {
  ble_loop();
  // imu_test_loop();
  // tof_test_loop();
  // motor_test();
}