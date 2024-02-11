#include "ble_arduino.h"
#include "IMU.h"

void setup() {
  Serial.begin(115200);

  ble_setup();
  imu_setup();
}

void loop() {
  ble_loop();
  // imu_test_loop();
}