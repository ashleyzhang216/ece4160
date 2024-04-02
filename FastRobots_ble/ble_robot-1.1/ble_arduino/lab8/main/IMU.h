/****************************************************************
 * Example1_Basics.ino
 * ICM 20948 Arduino Library Demo
 * Use the default configuration to stream 9-axis IMU data
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/
#ifndef IMU_H
#define IMU_H

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "utility.h"

#define SERIAL_PORT Serial
#define WIRE_PORT Wire 
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 0

struct imu_snapshot {
  float acc_x;
  float acc_y;
  float acc_z;

  float gyr_x;
  float gyr_y;
  float gyr_z;

  float mag_x;
  float mag_y;
  float mag_z;

  float temp;
};

// for yaw pid in ble, TODO: move to new header file
imu_snapshot prev_imu_datapoints[2]; // index 1 is previous point, index 2 is two points ago
long unsigned int prev_imu_datatimes [2];
float cur_yaw;

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

void imu_setup() {
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

    myICM.begin(WIRE_PORT, AD0_VAL);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  pinMode(LED_BUILTIN, OUTPUT);
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}

imu_snapshot get_imu_data() {
  imu_snapshot data;

  while(!myICM.dataReady());
  myICM.getAGMT();

  data.acc_x = myICM.accX();
  data.acc_y = myICM.accY();
  data.acc_z = myICM.accZ();

  data.gyr_x = myICM.gyrX();
  data.gyr_y = myICM.gyrY();
  data.gyr_z = myICM.gyrZ();

  data.mag_x = myICM.magX();
  data.mag_y = myICM.magY();
  data.mag_z = myICM.magZ();

  data.temp = myICM.temp();

  return data;
}

float get_roll(imu_snapshot data) {
  return rad_to_deg(atan2(data.acc_x, data.acc_z));
}

float get_pitch(imu_snapshot data) {
  return rad_to_deg(atan2(data.acc_y, data.acc_z));
}

void imu_test_loop() {
  imu_snapshot data = get_imu_data();

  // SERIAL_PORT.print("roll: ");
  // printFormattedFloat(get_roll(data), 5, 2);
  Serial.println(get_roll(data));
  // SERIAL_PORT.print(", pitch: ");
  // printFormattedFloat(get_pitch(data), 5, 2);
  // SERIAL_PORT.println();

  delay(30);
}

#endif // IMU_H