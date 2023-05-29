#include "CRU.h"

_CRU CRU;

void _CRU::update() {
  GPIO.out_w1tc = (1<<LATCH);
  for (uint8_t i = 0; i < 8; i++) {
    GPIO.out_w1tc = (1 << CLOCK);
    GPIO.out_w1ts = ((this->indConfig & 0b10000000 >> i) && 1) << DATA;
    GPIO.out_w1tc = (!((this->indConfig & 0b10000000 >> i) && 1)) << DATA;
    GPIO.out_w1ts = (1 << CLOCK);
  }
  for (uint8_t i = 0; i < 8; i++) {
    GPIO.out_w1tc = (1 << CLOCK);
    GPIO.out_w1ts = ((this->inConfig & 0b10000000 >> i) && 1) << DATA;
    GPIO.out_w1tc = (!((this->inConfig & 0b10000000 >> i) && 1)) << DATA;
    GPIO.out_w1ts = (1 << CLOCK);
  }
  GPIO.out_w1ts = (1<<LATCH);

  for (uint8_t i = 0; i < 4; i++) {
    ledcWrite(i, this->M[i]);
  }
}

void _CRU::writeMotor(uint8_t number, uint16_t duty, uint8_t dir) {
  this->M[number] = duty;
  this->inConfig &= ~(0b11 << number * 2);
  this->inConfig |= dir << number * 2;
}

void _CRU::init() {
  pinMode(CLOCK, OUTPUT);
  pinMode(LATCH, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(SERVO1, OUTPUT);
  pinMode(SERVO2, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(EN3, OUTPUT);
  pinMode(EN4, OUTPUT);
  digitalWrite(LATCH, HIGH);

  for (uint8_t i = 0; i < 4; i++) {
    ledcSetup(i, 4000, 12);
  }

  ledcSetup(4, 250, 12);
  ledcSetup(5, 250, 12);
  ledcAttachPin(EN1, 0);
  ledcAttachPin(EN2, 1);
  ledcAttachPin(EN3, 2);
  ledcAttachPin(EN4, 3);
  ledcAttachPin(SERVO1, 4);
  ledcAttachPin(SERVO2, 5);
}

void _CRU::writeMicroseconds(uint8_t number, uint16_t micro) {
  if (number == 0) {
    ledcWrite(4, (uint32_t)micro / 4000.00 * 4096.00);
  } else if (number == 1) {
    ledcWrite(5, (uint32_t)micro / 4000.00 * 4096.00);
  }
}

void _CRU::writeInd(uint8_t config){
  this->indConfig = config;
}
