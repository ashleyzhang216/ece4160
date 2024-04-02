#include "ble_arduino.h"
#include "IMU.h"
#include "tof.h"

void setup() {
  Serial.begin(115200);

  ble_setup();
  imu_setup();
  tof_setup();
}

void loop() {
  ble_loop();
  // imu_test_loop();
  // tof_test_loop();
}