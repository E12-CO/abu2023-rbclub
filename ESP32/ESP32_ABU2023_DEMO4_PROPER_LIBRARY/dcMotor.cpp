#include "dcMotor.h"
dcMotor::dcMotor(motorParameter_t motorParameter){
  this->parameter = motorParameter;
}

void dcMotor::setPosPID(float kp, float ki, float kd) {
  this->parameter.posKp = kp;
  this->parameter.posKi = ki;
  this->parameter.posKd = kd;
}

void dcMotor::setSpeedPID(float kp, float ki, float kd) {
  this->parameter.speedKp = kp;
  this->parameter.speedKi = ki;
  this->parameter.speedKd = kd;
}

void dcMotor::readPosPID(float& kp, float& ki, float& kd) {
  kp = this->parameter.posKp;
  ki = this->parameter.posKi;
  kd = this->parameter.posKd;
}

void dcMotor::readSpeedPID(float& kp, float& ki, float& kd) {
  kp = this->parameter.speedKp;
  ki = this->parameter.speedKi;
  kd = this->parameter.speedKd;
}

void dcMotor::setCMD(uint32_t cmd) {
  this->cmd = cmd;
}

void dcMotor::readMode(uint32_t& mode) {
  mode = this->mode;
}

void dcMotor::readCMD(uint32_t& cmd) {
  cmd = this->cmd;
}

void dcMotor::setTargetedSpeed(float speed) {
  this->targetedSpeed = speed * this->reductionRatio;
}

void dcMotor::readCurrentSpeed(float& speed) {
  speed = this->currentSpeed;
}

void dcMotor::setTargetedPosition(int64_t position) {
  this->targetedPos = position;
}

void dcMotor::readCurrentPosition(int64_t& position) {
  position = this->currentPos;
}

void dcMotor::setReductionRatio(float reductionRatio) {
  this->reductionRatio = reductionRatio;
}

void dcMotor::readReductionRatio(float& reductionRatio) {
  reductionRatio = this->reductionRatio;
}

void dcMotor::setPWM(uint16_t pwm, uint16_t dir) {
  this->pwm = pwm;
  this->dir = dir;
}

void dcMotor::readPWM(uint16_t& pwm, uint16_t& dir) {
  pwm = this->pwm;
  dir = this->dir;
}

void dcMotor::readResultedSpeedPID(float& resultedP, float& resultedI, float& resultedD, float& resultedPID) {
  resultedP = this->speedPID.resultedP;
  resultedI = this->speedPID.resultedI;
  resultedD = this->speedPID.resultedD;
  resultedPID = this->speedPID.result;
}

void dcMotor::readResultedPosPID(float& resultedP, float& resultedI, float& resultedD, float& resultedPID) {
  resultedP = this->posPID.resultedP;
  resultedI = this->posPID.resultedI;
  resultedD = this->posPID.resultedD;
  resultedPID = this->posPID.result;
}

void dcMotor::readTargetedSpeed(float& speed) {
  speed = this->targetedSpeed;
}

void dcMotor::readTargetedPosition(int64_t& position) {
  position = this->targetedPos;
}

void dcMotor::updateSpeedControl() {
  this->speedControl.targetedSpd = this->targetedSpeed;
}

void dcMotor::updatePositionControl() {
  this->positionControl.targetedPos = this->targetedPos;
}

void dcMotor::updateDirectControl() {
  this->directControl.pwm = this->pwm;
  this->directControl.dir = this->dir;
}

void dcMotor::messageAssemble(uint8_t* message) {
  switch (this->cmd) {
    case motorParameterCMD:
      *(motorParameter_t*)(message) = this->parameter;
      *(uint16_t*)(message + cmdOffset) = motorParameterCMD;
      break;
    case motorParameterReadCMD:
      *(uint16_t*)(message + cmdOffset) = motorParameterReadCMD;
      break;
    case speedControlCMD:
      this->updateSpeedControl();
      *(speedControl_t*)(message) = this->speedControl;
      *(uint16_t*)(message + cmdOffset) = speedControlCMD;
      break;
    case positionControlCMD:
      this->updatePositionControl();
      *(positionControl_t*)(message) = this->positionControl;
      *(uint16_t*)(message + cmdOffset) = positionControlCMD;
      break;
    case directControlCMD:
      this->updateDirectControl();
      *(directControl_t*)(message) = this->directControl;
      *(uint16_t*)(message + cmdOffset) = directControlCMD;
      break;
  }
}

void dcMotor::setParameterCycle(uint8_t cycle){
  this->parameterCycle = cycle;
}
