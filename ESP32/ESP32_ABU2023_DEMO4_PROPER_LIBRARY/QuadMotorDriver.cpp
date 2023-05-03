
#include "QuadMotorDriver.h"

QuadMotorDriver::QuadMotorDriver(uint16_t deviceID){
  this->deviceID = deviceID;
}

void QuadMotorDriver::messageAssemble(uint8_t* message) {
  *(uint32_t*)(message) = transmitHeader;
  *(uint16_t*)(message + typeIDOffset) = this->typeID;
  *(uint16_t*)(message + deviceIDOffset) = this->deviceID;
  *(uint32_t*)(message + statusOffset) = this->status;
  for (uint8_t i = 0; i < 4; i++) {
    if(this->attached[i]){
      (*(dcMotor *)(motorAddress[i])).messageAssemble(message + headerOffset + i*perMotorLength);
    }
  }
}

void QuadMotorDriver::getTypeID(uint16_t& typeID) {
  typeID = this->typeID;
}

void QuadMotorDriver::setDeviceID(uint16_t deviceID) {
  this->deviceID = deviceID;
}

void QuadMotorDriver::attach(dcMotor& motor,uint8_t channel){
  this->attached[channel] = 1;
  this->motorAddress[channel] = (uint8_t *)(&motor);
}