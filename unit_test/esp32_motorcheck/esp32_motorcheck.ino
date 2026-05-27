#include <Wire.h>

const int PWMA = 13;
const int AIN2 = 14;
const int AIN1 = 27;
const int BIN1 = 26;
const int BIN2 = 25;
const int PWMB = 33;

void MotorWriting(double vL, double vR) {
  if (vR >= 0) {
    digitalWrite(BIN1, LOW); 
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, HIGH); 
    digitalWrite(BIN2, LOW);
    vR = -vR;
  }
  if (vL >= 0) {
    digitalWrite(AIN1, LOW); 
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, HIGH);  
    digitalWrite(AIN2, LOW);
    vL = -vL;
  }
  analogWrite(PWMA, (int)vL);
  analogWrite(PWMB, (int)vR);
}

void MotorCheck() {
  MotorWriting(100, 100);
  delay(1000);
  MotorWriting(-100, -100);
  delay(1000);
  MotorWriting(100, -100);
  delay(1000);
  MotorWriting(-100, 100);
  delay(1000);
}

void setup() {
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  MotorWriting(0, 0);
}

void loop() {
  MotorTest();
}
