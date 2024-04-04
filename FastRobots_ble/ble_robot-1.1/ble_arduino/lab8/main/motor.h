#ifndef MOTOR_H
#define MOTOR_H

#define R_MOTOR_IN1 7
#define R_MOTOR_IN2 6
#define L_MOTOR_IN1 A5
#define L_MOTOR_IN2 A3

// #define R_MOTOR_IN1 D12
// #define R_MOTOR_IN2 D11
// #define L_MOTOR_IN1 A3
// #define L_MOTOR_IN2 A2

int L_MOTOR_MIN = 48;
int R_MOTOR_MIN = 48;
int L_MOTOR_MAX = 255;
int R_MOTOR_MAX = 255;

int l_pwm_val = 0;
int r_pwm_val = 0;

float p_val = 0.0;
float i_val = 0.0;
float d_val = 0.0;

// for test commands in ble
int cur_speed = 0;
int move_duration;
int cur_l_speed, cur_r_speed;
int cur_l_pwm, cur_r_pwm;

// for test pid in ble
float cur_vel; // in mm/msec
int cur_pos; // in mm
int cur_error; // in mm
float cur_pd;

// for yaw pid in ble, TODO: move to new header file


enum Motor {
  L_MOTOR = 0,
  R_MOTOR = 1
};

void motor_setup() {
  pinMode(L_MOTOR_IN1, OUTPUT);
  pinMode(L_MOTOR_IN2, OUTPUT);
  pinMode(R_MOTOR_IN1, OUTPUT);
  pinMode(R_MOTOR_IN2, OUTPUT);

  analogWrite(L_MOTOR_IN1, 0);
  analogWrite(L_MOTOR_IN2, 0);
  analogWrite(R_MOTOR_IN1, 0);
  analogWrite(R_MOTOR_IN2, 0);
}

int move_motor(Motor m, int speed) {
  if(speed < -100) speed = -100;
  if(speed > 100) speed = 100;

  Serial.print("speed: ");
  Serial.print(speed);
  Serial.print(", ");

  if(m == L_MOTOR) {

    // pwm_val = abs((speed / 100.0) * L_MOTOR_MAX);
    l_pwm_val = map(abs(speed), 0, 100, L_MOTOR_MIN, L_MOTOR_MAX);

    if(speed > 0) {
      analogWrite(L_MOTOR_IN1, 0);
      analogWrite (L_MOTOR_IN2, l_pwm_val);
      Serial.print("Left forwards: ");
    } else if(speed < 0) {
      analogWrite(L_MOTOR_IN2, 0);
      analogWrite(L_MOTOR_IN1, l_pwm_val);
      Serial.print("Left backwards: ");
    } else {
      analogWrite(L_MOTOR_IN1, 0);
      analogWrite(L_MOTOR_IN2, 0);
      Serial.print("Left coast: ");
    }

    Serial.println(l_pwm_val);
    return l_pwm_val;
  } else {

    // pwm_val = abs((speed / 100.0) * R_MOTOR_MAX);
    r_pwm_val = map(abs(speed), 0, 100, R_MOTOR_MIN, R_MOTOR_MAX);

    if(speed > 0) {
      analogWrite(R_MOTOR_IN1, 0);
      analogWrite (R_MOTOR_IN2, r_pwm_val);
      Serial.print("Right forwards: ");
    } else if(speed < 0) {
      analogWrite(R_MOTOR_IN2, 0);
      analogWrite(R_MOTOR_IN1, r_pwm_val);
      Serial.print("Right backwards: ");
    } else {
      analogWrite(R_MOTOR_IN1, 0);
      analogWrite(R_MOTOR_IN2, 0);
      Serial.print("Right coast: ");
    }

    Serial.println(r_pwm_val);
    return r_pwm_val;
  }
}

void brake_motors() {
  analogWrite(L_MOTOR_IN1, 0);
  analogWrite(L_MOTOR_IN2, 0);
  analogWrite(R_MOTOR_IN1, 0);
  analogWrite(R_MOTOR_IN2, 0);
}

void motor_test() {

  Serial.println("Starting motor test");

  // move_motor(L_MOTOR, 50);
  // delay(2000);
  // move_motor(L_MOTOR, -50);
  // delay(2000);

  // brake_motors();

  // move_motor(R_MOTOR, 50);
  // delay(2000);
  // move_motor(R_MOTOR, -50);
  // delay(2000);

  // brake_motors();

  int power = 30;

  // move_motor(R_MOTOR, power);
  move_motor(L_MOTOR, -1 * power);
  delay(2000);
  
  brake_motors();
  delay(1000);

  move_motor(R_MOTOR, -1 * power);
  move_motor(L_MOTOR, power);
  delay(2000);
  
  brake_motors();
  delay(1000);
  
  move_motor(R_MOTOR, power);
  move_motor(L_MOTOR, -1 * power);
  delay(2000);
  
  brake_motors();
  delay(1000);

  move_motor(R_MOTOR, -1 * power);
  move_motor(L_MOTOR, -1 * power);
  delay(2000);
  
  brake_motors();
  delay(1000);

  while(1);

  // R motor first: R motor off when running F/F : L_MOTOR_IN2 & R_MOTOR_IN2
  // L motor first: L motor off when running B/B : L_MOTOR_IN1 & R_MOTOR_IN1
}

#endif // MOTOR_H