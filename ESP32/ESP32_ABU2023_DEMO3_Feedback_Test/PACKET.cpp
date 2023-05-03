
#include "PACKET.h"
uint8_t CRC8_Finder(uint8_t *packet, uint16_t length) {
  uint8_t CRC = initial;
  for (uint16_t i = 0; i < length; i++) {
    CRC = CRC ^ (*(packet + i));
    for (uint8_t j = 0; j < 8; j++) {
      if (CRC & 0x80) {
        CRC = (CRC << 1) ^ polynomial;
      } else {
        CRC = CRC << 1;
      }
    }
  }
  return CRC;
}

QuadMotorDriver::QuadMotorDriver(motor *m) {
  for (uint8_t i = 0; i < 4; i++) {
    this->M[i] = m[i];
  }
}

void QuadMotorDriver::setHeader(uint8_t *packet) {
  *(uint32_t *)packet = transmitHeader;
  *(uint16_t *)(packet + typeIDOffset) = this->typeID;
  *(uint16_t *)(packet + deviceIDOffset) = this->typeID;
  *(uint32_t *)(packet + statusOffset) = this->status;
}

void QuadMotorDriver::getPacket(uint8_t *packet) {
  this->setHeader(packet);
  for (uint8_t i = 0; i < 4; i++) {
    switch (this->CMD[i]) {
      case motorParameterCMD:
        *(motorParameter_t *)(motorTotalOffset) = M[i].parameter;
        *(uint16_t *)(cmdTotalOffset) = motorParameterCMD;
        break;

      case motorParameterReadCMD:
        *(uint16_t *)(cmdTotalOffset) = motorParameterReadCMD;
        break;
      case speedControlCMD:
        *(speedControl_t *)(motorTotalOffset) = M[i].speedControl;
        *(uint16_t *)(cmdTotalOffset) = speedControlCMD;
        break;
      case positionControlCMD:
        *(positionControl_t *)(motorTotalOffset) = M[i].positionControl;
        *(uint16_t *)(cmdTotalOffset) = positionControlCMD;
        break;
    }
  }
  *(uint8_t *)(packet + packetSize) = CRC8_Finder(packet, packetSize);
}

void QuadMotorDriver::setMode(uint8_t number, uint8_t mode) {
  M[number - 1].mode = mode;
}

void QuadMotorDriver::setSpeed(uint8_t number, float speed) {
  M[number - 1].speedControl.targetedSpd = speed;
}
void QuadMotorDriver::setDeviceID(uint16_t deviceID) {
  this->deviceID = deviceID;
}

void QuadMotorDriver::setSpeedPID(uint8_t number, float kp, float ki, float kd) {
  M[number - 1].parameter.speedKp = kp;
  M[number - 1].parameter.speedKi = ki;
  M[number - 1].parameter.speedKd = kd;
}

void QuadMotorDriver::setPosPID(uint8_t number, float kp, float ki, float kd) {
  M[number - 1].parameter.posKp = kp;
  M[number - 1].parameter.posKi = ki;
  M[number - 1].parameter.posKd = kd;
}
void QuadMotorDriver::setCMD(uint8_t number, uint16_t CMD) {
  this->CMD[number - 1] = CMD;
}

void QuadMotorDriver::setSpeedP(uint8_t number, float kp){
  M[number-1].parameter.speedKp = kp;
}
void QuadMotorDriver::setSpeedI(uint8_t number, float ki){
  M[number-1].parameter.speedKi = ki;
}
void QuadMotorDriver::setSpeedD(uint8_t number, float kd){
  M[number-1].parameter.speedKd = kd;
}


void QuadMotorDriver::setPosP(uint8_t number, float kp){
  M[number-1].parameter.posKp = kp;
}
void QuadMotorDriver::setPosI(uint8_t number, float ki){
  M[number-1].parameter.posKi = ki;
}
void QuadMotorDriver::setPosD(uint8_t number, float kd){
  M[number-1].parameter.posKd = kd;
}

float QuadMotorDriver::getSpeedP(uint8_t number){
  return M[number-1].parameter.speedKp;
}
float QuadMotorDriver::getSpeedI(uint8_t number){
  return M[number-1].parameter.speedKi;
}
float QuadMotorDriver::getSpeedD(uint8_t number){
  return M[number-1].parameter.speedKd;
}


float QuadMotorDriver::getPosP(uint8_t number){
  return M[number-1].parameter.posKp;
}
float QuadMotorDriver::getPosI(uint8_t number){
  return M[number-1].parameter.posKi;
}
float QuadMotorDriver::getPosD(uint8_t number){
  return M[number-1].parameter.posKd;
}

float QuadMotorDriver::getSpeed(uint8_t number){
  return M[number-1].speedControl.targetedSpd;
}



