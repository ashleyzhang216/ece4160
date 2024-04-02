#ifndef TOF_H
#define TOF_H

#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

#define TOF_XSHUT_PIN 4

#define TOF_FRONT 0
#define TOF_BACK 1

SFEVL53L1X tof_sensor0; 
SFEVL53L1X tof_sensor1; // has xshut pin
const uint8_t tof_addr0 = 0x2A;
const uint8_t tof_addr1 = 0x29;

int prev_dist0_mm = 0;
int prev_dist1_mm = 0;
bool tof0_ranging = false;
bool tof1_ranging = false;

int prev_tof_datapoints[2]; // index 1 is previous point, index 2 is two points ago
long unsigned int prev_tof_datatimes [2];
int target_dist;

void tof_setup() {
  Wire.begin();

  pinMode(TOF_XSHUT_PIN, OUTPUT);
  digitalWrite(TOF_XSHUT_PIN, LOW);

  tof_sensor0.setI2CAddress(tof_addr0); 

  if (tof_sensor0.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor 0 failed to begin. Please check wiring. Freezing...");
    while (1);
  }

  tof_sensor0.setDistanceModeLong();
  
  digitalWrite(TOF_XSHUT_PIN, HIGH);

  if (tof_sensor1.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1);
  }
  tof_sensor1.setDistanceModeLong();

  Serial.println("TOF sensors successfully initialized");
}

void tof_test_loop() {
  tof_sensor0.startRanging(); 
  tof_sensor1.startRanging(); 

  while (!tof_sensor0.checkForDataReady() || !tof_sensor1.checkForDataReady())
  {
    delay(1);
  }

  int dist0 = tof_sensor0.getDistance(); //Get the result of the measurement from the sensor
  int dist1 = tof_sensor1.getDistance(); //Get the result of the measurement from the sensor
  
  tof_sensor0.clearInterrupt();
  tof_sensor0.stopRanging();
  tof_sensor1.clearInterrupt();
  tof_sensor1.stopRanging();

  // Serial.print("Distance(mm): ");
  Serial.print("dist0 (mm): ");
  Serial.print(dist0);
  Serial.print(", dist1 (mm): ");
  Serial.print(dist1);

  Serial.println();
}

int get_dist(int sensor) {
  if(sensor == 0) {

    if(!tof0_ranging) {
      tof_sensor0.startRanging(); 
      tof0_ranging = true;
    }

    if(tof_sensor0.checkForDataReady()) {
      prev_dist0_mm = tof_sensor0.getDistance();
      tof_sensor0.clearInterrupt();
      tof_sensor0.stopRanging();
      tof_sensor0.startRanging(); 
      tof0_ranging = true;
    }

    return prev_dist0_mm;

  } else if(sensor == 1) {

    if(!tof1_ranging) {
      tof_sensor1.startRanging(); 
      tof1_ranging = true;
    }

    if(tof_sensor1.checkForDataReady()) {
      prev_dist1_mm = tof_sensor1.getDistance();
      tof_sensor1.clearInterrupt();
      tof_sensor1.stopRanging();
      tof_sensor1.startRanging(); 
      tof1_ranging = true;
    }

    return prev_dist1_mm;

  } else {
    Serial.println("Error: get_dist() got invalid sensor");
    while(1);
  }
}

#endif // TOF_H