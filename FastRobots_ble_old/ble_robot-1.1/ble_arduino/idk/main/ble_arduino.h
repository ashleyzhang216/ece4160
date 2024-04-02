#ifndef BLE_ARDUINO_H
#define BLE_ARDUINO_H

#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "IMU.h"
#include "utility.h"
#include "tof.h"
#include "motor.h"

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "c330a0e0-98d2-4fd3-9f2c-0ab3bf3cac8a"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;
//////////// Global Variables ////////////

enum CommandTypes
{
    PING = 0,
    SEND_TWO_INTS = 1, 
    SEND_THREE_FLOATS = 2,
    ECHO = 3,
    DANCE = 4,
    SET_VEL = 5,
    GET_TIME_MILLIS = 6,
    GET_MANY_TIME_MILLIS = 7,
    SEND_TIME_DATA = 8,
    GET_TEMP_READINGS = 9,
    GET_ROLL_PITCH = 10,
    GET_GYRO_DATA = 11,
    GET_ACC_GYRO_DATA = 12,
    GET_DIST_DATA = 13,
    ADJUST_PWM_VALS = 14,
    BRAKE_MOTORS = 15,
    GET_PID_DATA = 16,
    MOVE_FORWARD_TIMED = 17,
    MOVE_WHEELS_TIMED = 18,
    PID_TO_WALL = 19,
    CHANGE_SETPOINT = 20,
    PID_YAW = 21
};

#define NUM_TIMESTAMPS 1000
unsigned long timestamps[NUM_TIMESTAMPS];
int temperatures[NUM_TIMESTAMPS];
float roll_data[NUM_TIMESTAMPS];
float pitch_data[NUM_TIMESTAMPS];
float gyro_raw[3][NUM_TIMESTAMPS];
int dists[2][NUM_TIMESTAMPS];
float vels[NUM_TIMESTAMPS];

void
handle_command()
{   
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;
    unsigned long start;

    // Get robot command type (an integer)
    /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and 
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
    success = robot_cmd.get_command_type(cmd_type);

    // Check if the last tokenization was successful and return if failed
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        
        case ADJUST_PWM_VALS:

            // ping_led();

            // // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(L_MOTOR_MIN);
            if (!success)
                return;

            // // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(L_MOTOR_MAX);
            if (!success)
                return;

            // // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(R_MOTOR_MIN);
            if (!success)
                return;

            // // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(R_MOTOR_MAX);
            if (!success)
                return;

            // // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(p_val);
            if (!success)
                return;
              
            // // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(i_val);
            if (!success)
                return;
            
            // // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(d_val);
            if (!success)
                return;

            tx_estring_value.clear();
            tx_estring_value.append("L_MOTOR_MAX: ");
            tx_estring_value.append(L_MOTOR_MAX);
            tx_estring_value.append(", R_MOTOR_MAX: ");
            tx_estring_value.append(R_MOTOR_MAX);
            tx_estring_value.append(", p_val: ");
            tx_estring_value.append(p_val);
            tx_estring_value.append(", i_val: ");
            tx_estring_value.append(i_val);
            tx_estring_value.append(", d_val: ");
            tx_estring_value.append(d_val);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            ping_led();

            break;
        
        case BRAKE_MOTORS:

            brake_motors();

            ping_led();

            break;
        
        case MOVE_WHEELS_TIMED:

            ping_led();

            success = robot_cmd.get_next_value(cur_l_speed);
            if (!success)
                return;
            
            success = robot_cmd.get_next_value(cur_r_speed);
            if (!success)
                return;

            success = robot_cmd.get_next_value(move_duration);
            if (!success)
                return;

            cur_l_pwm = move_motor(L_MOTOR, cur_l_speed);
            cur_r_pwm = move_motor(R_MOTOR, cur_r_speed);

            delay(move_duration);
            brake_motors();

            tx_estring_value.clear();
            tx_estring_value.append("cur_l_pwm: ");
            tx_estring_value.append(cur_l_pwm);
            tx_estring_value.append(", cur_r_pwm: ");
            tx_estring_value.append(cur_r_pwm);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            ping_led();

            break;
      
        /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}

void
ble_setup()
{
    BLE.begin();

    // Set advertised local name and service
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);

    // Add BLE characteristics
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);

    // Add BLE service
    BLE.addService(testService);

    // Initial values for characteristics
    // Set initial values to prevent errors when reading for the first time on central devices
    tx_characteristic_float.writeValue(0.0);

    /*
     * An example using the EString
     */
    // Clear the contents of the EString before using it
    tx_estring_value.clear();

    // Append the string literal "[->"
    tx_estring_value.append("[->");

    // Append the float value
    tx_estring_value.append(9.0);

    // Append the string literal "<-]"
    tx_estring_value.append("<-]");

    // Write the value to the characteristic
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();
}

void
write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
            
        }

        previousMillis = currentMillis;
    }
}

void
read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

void ble_loop()
{
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        // While central is connected
        while (central.connected()) {
            // Send data
            write_data();

            // Read data
            read_data();
        }

        Serial.println("Disconnected");
    }
}

#endif // BLE_ARDUINO_H
