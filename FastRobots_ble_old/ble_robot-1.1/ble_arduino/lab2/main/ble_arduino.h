#ifndef BLE_ARDUINO_H
#define BLE_ARDUINO_H

#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "IMU.h"
#include "utility.h"

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
    GET_ACC_GYRO_DATA = 12
};

#define NUM_TIMESTAMPS 1000
unsigned long timestamps[NUM_TIMESTAMPS];
int temperatures[NUM_TIMESTAMPS];
float roll_data[NUM_TIMESTAMPS];
float pitch_data[NUM_TIMESTAMPS];
float gyro_raw[3][NUM_TIMESTAMPS];

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
        /*
         * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
         */
        case PING:
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        /*
         * Extract two integers from the command string
         */
        case SEND_TWO_INTS:
            int int_a, int_b;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_b);
            if (!success)
                return;

            Serial.print("Two Integers: ");
            Serial.print(int_a);
            Serial.print(", ");
            Serial.println(int_b);
            
            break;
        /*
         * Extract three floats from the command string
         */
        case SEND_THREE_FLOATS:
            /*
             * Your code goes here.
             */

            break;
        /*
         * Add a prefix and postfix to the string value extracted from the command string
         */
        case ECHO:

            char char_arr[MAX_MSG_SIZE];

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
                return;

            tx_estring_value.clear();
            tx_estring_value.append("Robot says -> ");
            tx_estring_value.append(char_arr);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());
                        
            break;
        /*
         * DANCE
         */
        case DANCE:
            Serial.println("Look Ma, I'm Dancin'!");

            break;
        
        /*
         * SET_VEL
         */
        case SET_VEL:

            break;
        
        case GET_TIME_MILLIS:
            
            tx_estring_value.clear();
            tx_estring_value.append("T:");
            tx_estring_value.append((int)(millis()));
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        
        case GET_MANY_TIME_MILLIS:
            start = millis();

            while(millis() - start < 3000) {
              tx_estring_value.clear();
              tx_estring_value.append("T:");
              tx_estring_value.append((int)(millis()));
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            break;
        
        case SEND_TIME_DATA:

            for(unsigned long i = 0; i < NUM_TIMESTAMPS; i++) {
              timestamps[i] = millis();
            }

            for(unsigned long i = 0; i < NUM_TIMESTAMPS; i++) {
              tx_estring_value.clear();
              tx_estring_value.append("T:");
              tx_estring_value.append((int)(timestamps[i]));
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            break;

        case GET_TEMP_READINGS:

            for(unsigned long i = 0; i < NUM_TIMESTAMPS; i++) {
              timestamps[i] = millis();
              temperatures[i] = analogReadTemp();
            }

            for(unsigned long i = 0; i < NUM_TIMESTAMPS; i++) {
              tx_estring_value.clear();
              tx_estring_value.append("T:");
              tx_estring_value.append((int)(timestamps[i]));
              tx_estring_value.append(",");
              tx_estring_value.append((int)(temperatures[i]));
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            break;
        
        case GET_ROLL_PITCH:

            for(unsigned long i = 0; i < NUM_TIMESTAMPS; i++) {
              imu_snapshot data = get_imu_data();
              timestamps[i] = millis();
              roll_data[i] = get_roll(data);
              pitch_data[i] = get_pitch(data);

              delay(10);
            }

            for(unsigned long i = 0; i < NUM_TIMESTAMPS; i++) {
              tx_estring_value.clear();
              tx_estring_value.append("T:");
              tx_estring_value.append((int)(timestamps[i]));
              tx_estring_value.append(",");
              tx_estring_value.append(roll_data[i]);
              tx_estring_value.append(",");
              tx_estring_value.append(pitch_data[i]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            break;
        
        case GET_GYRO_DATA:

            start = millis();
            for(unsigned long i = 0; i < NUM_TIMESTAMPS; i++) {
              imu_snapshot data = get_imu_data();
              timestamps[i] = millis() - start;
              gyro_raw[0][i] = data.gyr_x;
              gyro_raw[1][i] = data.gyr_y;
              gyro_raw[2][i] = data.gyr_z;

              delay(10);
            }

            for(unsigned long i = 0; i < NUM_TIMESTAMPS; i++) {
              tx_estring_value.clear();
              tx_estring_value.append("T:");
              tx_estring_value.append((int)(timestamps[i]));
              tx_estring_value.append(",");
              tx_estring_value.append(gyro_raw[0][i]);
              tx_estring_value.append(",");
              tx_estring_value.append(gyro_raw[1][i]);
              tx_estring_value.append(",");
              tx_estring_value.append(gyro_raw[2][i]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            break;
        
        case GET_ACC_GYRO_DATA:

            ping_led();
            
            start = millis();
            for(unsigned long i = 0; i < NUM_TIMESTAMPS; i++) {
              imu_snapshot data = get_imu_data();
              timestamps[i] = millis() - start; // max time is 65k ish millis before overflow
              roll_data[i] = get_roll(data);
              pitch_data[i] = get_pitch(data);
              gyro_raw[0][i] = data.gyr_x;
              gyro_raw[1][i] = data.gyr_y;
              gyro_raw[2][i] = data.gyr_z;
            }

            for(unsigned long i = 0; i < NUM_TIMESTAMPS; i++) {
              tx_estring_value.clear();
              tx_estring_value.append("T:");
              tx_estring_value.append((int)(timestamps[i]));
              tx_estring_value.append(",");
              tx_estring_value.append(roll_data[i]);
              tx_estring_value.append(",");
              tx_estring_value.append(pitch_data[i]);
              tx_estring_value.append(",");
              tx_estring_value.append(gyro_raw[0][i]);
              tx_estring_value.append(",");
              tx_estring_value.append(gyro_raw[1][i]);
              tx_estring_value.append(",");
              tx_estring_value.append(gyro_raw[2][i]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

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

void
ble_loop()
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
