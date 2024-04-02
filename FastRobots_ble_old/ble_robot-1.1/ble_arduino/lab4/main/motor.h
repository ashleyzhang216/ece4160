#ifndef MOTOR_H
#define MOTOR_H

#define L_MOTOR_IN1 7
#define L_MOTOR_IN2 6
#define R_MOTOR_IN1 A5
#define R_MOTOR_IN2 A3

void motor_setup() {
  pinMode(L_MOTOR_IN1, OUTPUT);
  pinMode(L_MOTOR_IN2, OUTPUT);
  pinMode(R_MOTOR_IN1, OUTPUT);
  pinMode(R_MOTOR_IN2, OUTPUT);

  digitalWrite(L_MOTOR_IN1, LOW);
  digitalWrite(L_MOTOR_IN2, LOW);
  digitalWrite(R_MOTOR_IN1, LOW);
  digitalWrite(R_MOTOR_IN2, LOW);
}

void motor_test() {

  Serial.println("Starting motor test");

  // digitalWrite(L_MOTOR_IN1, HIGH);
  analogWrite(L_MOTOR_IN1, 128);
  analogWrite(R_MOTOR_IN1, 128);
  // delay(2000);
  // digitalWrite(L_MOTOR_IN1, LOW);
  // analogWrite(R_MOTOR_IN1, 128);
  while(1);
}

#endif // MOTOR_H