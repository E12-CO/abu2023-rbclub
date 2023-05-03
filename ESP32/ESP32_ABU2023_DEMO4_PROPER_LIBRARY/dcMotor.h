#include "stdlib.h"
#include "stdint.h"

#define cmdOffset 118

#define motorParameterCMD 0x5501
#define motorParameterReadCMD 0x5502
#define speedControlCMD 0x5503
#define speedControlRunUntilContact 0x5603
#define speedControlStop 0x5703
#define positionControlCMD 0x5504
#define directControlCMD 0x5505
#define CMDerrorCMD 0xAA01
typedef struct {
  float kp;
  float ki;
  float kd;
} stdPIDParameter_t;

typedef struct {
  float posKp = 0;
  float posKi = 0;
  float posKd = 0;
  float speedKp = 0;
  float speedKi = 0;
  float speedKd = 0;
  float maxSpeed = 25000;
  float posRampCoe = 0;
  float spdRampCoe = 0;
  float encCPR = 1024;
  float speedResultedP = 0;
  float speedResultedI = 0;
  float speedResultedD = 0;
  float posResultedP = 0;
  float posResultedI = 0;
  float posResultedD = 0;
  float posResultedPID = 0;
  float speedResultedPID = 0;
  uint16_t encoderDigitalFilter = 0;
  uint16_t PIDPeriodARR = 0;
  int16_t pwm = 0;
  uint16_t dir = 0;
} motorParameter_t;

typedef struct
{
  float targetedSpd = 0;
  int64_t currentPos = 0;
  float currentSpd = 0;
  uint32_t status = 0;
} speedControl_t;

typedef struct
{
  int64_t targetedPos = 0;
  int64_t currentPos = 0;
  float targetedSpd = 0;
  float currentSpd = 0;
  uint32_t status = 0;
} positionControl_t;

typedef struct
{
  uint16_t pwm = 0;
  uint8_t dir = 0;
} directControl_t;

typedef struct
{
  float resultedP = 0;
  float resultedI = 0;
  float resultedD = 0;
  float errorIntegral = 0;
  float errorDerivative = 0;
  float error = 0;
  float prevError = 0;
  float result = 0;

} motorPID_t;

typedef struct
{
  motorParameter_t parameter;
  speedControl_t speedControl;
  positionControl_t positionControl;
  motorPID_t speedPID;
  motorPID_t posPID;
  int64_t currentPos;
  int64_t prevPos;
  int64_t targetedPos;
  uint64_t prevMicros;
  float timeConstant;
  float currentSpeed;
  float targetedSpeed;

  uint32_t returnMode;
  uint32_t mode;
  uint32_t status;

  uint16_t pwm;
  uint16_t dir;


} motor;

class dcMotor{
public:
  dcMotor(motorParameter_t motorParameter);
  void setPosPID(float kp, float ki, float kd);
  void readPosPID(float& kp, float& ki, float& kd);
  void setSpeedPID(float kp, float ki, float kd);
  void readSpeedPID(float& kp, float& ki, float& kd);
  void setCMD(uint32_t cmd);
  void readMode(uint32_t& mode);
  void readCMD(uint32_t& cmd);
  void setTargetedSpeed(float speed);
  void readCurrentSpeed(float& speed);
  void setTargetedPosition(int64_t position);
  void readCurrentPosition(int64_t& position);
  void setReductionRatio(float reductionRatio);
  void readReductionRatio(float& reductionRatio);
  void setPWM(uint16_t pwm, uint16_t dir);
  void readPWM(uint16_t& pwm, uint16_t& dir);
  void readResultedSpeedPID(float& resultedP, float& resultedI, float& resultedD, float& resultedPID);
  void readResultedPosPID(float& resultedP, float& resultedI, float& resultedD, float& resultedPID);
  void readTargetedSpeed(float& speed);
  void readTargetedPosition(int64_t& position);
  void setParameterCycle(uint8_t cycle);
  void messageAssemble(uint8_t* message);

private:
  motorParameter_t parameter;
  speedControl_t speedControl;
  positionControl_t positionControl;
  directControl_t directControl;
  motorPID_t speedPID;
  motorPID_t posPID;
  int64_t currentPos = 0;
  int64_t prevPos = 0;
  int64_t targetedPos = 0;
  uint64_t prevMicros = 0;
  float timeConstant = 0;
  float currentSpeed = 0;
  float targetedSpeed = 0;
  uint16_t cmd = 0;
  float reductionRatio = 1;
  uint16_t mode = 0;
  uint16_t pwm = 0;
  uint16_t dir = 0;
  uint8_t parameterCycle = 10;

  void updateSpeedControl();
  void updatePositionControl();
  void updateDirectControl();
};