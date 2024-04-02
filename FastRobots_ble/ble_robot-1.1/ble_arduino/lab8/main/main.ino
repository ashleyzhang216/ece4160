#include "ble_arduino.h"
#include "motor.h"
// #include "IMU.h"
// #include "tof.h"
#include "utility.h"

void setup() {
  Serial.begin(115200);

  ble_setup();
  motor_setup();
  // imu_setup();
  // tof_setup();
}

void loop() {
  ble_loop();
}