#include "E12.h"
#include "Mecanum.h"
motorParameter_t defaultMotorParameter;

dcMotor M1(defaultMotorParameter);
dcMotor M2(defaultMotorParameter);
dcMotor M3(defaultMotorParameter);
dcMotor M4(defaultMotorParameter);

QuadMotorDriver Q1(0x0000);


void setup() {
  E12.init(16000000);
  E12.attach(Q1,0);
  Q1.attach(M1, 0);
  Q1.attach(M2, 1);
  Q1.attach(M3, 2);
  Q1.attach(M4, 3);
}

void loop() {
  M1.setTargetedSpeed(200);
  delay(500);
  M2.setTargetedSpeed(300);
  M1.setTargetedSpeed(-200);
  delay(500);
  M2.setTargetedSpeed(-300);
  E12.broadcast();
}
