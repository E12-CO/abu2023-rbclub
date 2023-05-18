
#include "stdint.h"
#include "stdio.h"
#define transmitHeader 0x63010298
#define returnHeader 0x64010377
#define packetSize 512
#define perMotorLength 120
#define headerOffset 12
#define cmdOffset 118
#define typeIDOffset 4
#define deviceIDOffset 6
#define statusOffset 8
#define cmdTotalOffset packet + headerOffset + perMotorLength* i + cmdOffset
#define motorTotalOffset packet + headerOffset + perMotorLength* i
#define generalErrorMsk 0b111111

#define packetNorminal 0x00
#define packetCRCError 0x01
#define packetHeaderError 0x02
#define packetTypeIDError 0x03
#define packetDeviceIDError 0x04
#define packetStatusError 0x05
#define packetCMDError 0x06
#define packetCMD1Error 0x07
#define packetCMD2Error 0x08
#define packetCMD3Error 0x09
#define packetCMD4Error 0x0A

#define motorParameterCMD 0x5501
#define motorParameterReadCMD 0x5502
#define speedControlCMD 0x5503
#define positionControlCMD 0x5504
#define directControlCMD 0x5505
#define CMDerrorCMD 0xAA01

#define polynomial 0xB7
#define initial 0x00
#define CONFIG 0x01
#define NORMAL 0x02


typedef struct
{
  float posKp;
  float posKi;
  float posKd;
  float speedKp;
  float speedKi;
  float speedKd;
  float maxSpeed;
  float posRampCoe;
  float spdRampCoe;
  float encCPR;
  float speedResultedP;
  float speedResultedI;
  float speedResultedD;
  float posResultedP;
  float posResultedI;
  float posResultedD;
  float posResultedPID;
  float speedResultedPID;
  uint16_t encoderDigitalFilter;
  uint16_t PIDPeriodARR;
  uint16_t pwm;
  uint16_t dir;
} motorParameter_t;

typedef struct
{
  float targetedSpd;
  int64_t currentPos;
  float currentSpd;
  uint32_t status;
} speedControl_t;

typedef struct
{
  int64_t targetedPos;
  int64_t currentPos;
  float targetedSpd;
  float currentSpd;
  uint32_t status;
} positionControl_t;

typedef struct
{
  float resultedP;
  float resultedI;
  float resultedD;
  float errorIntegral;
  float errorDerivative;
  float error;
  float prevError;
  float result;

} motorPID_t;

typedef struct
{
	uint16_t pwm;
	uint16_t dir;
}directControl_t;

typedef struct
{
  motorParameter_t parameter;
  speedControl_t speedControl;
  positionControl_t positionControl;
  directControl_t directControl;
  uint32_t mode;

} motor;

uint8_t CRC8_Finder(uint8_t *packet, uint16_t length) ;

class QuadMotorDriver {
private:

  
public:

  QuadMotorDriver(motor* m);
  void setHeader(uint8_t* packet);
  void getPacket(uint8_t* packet);
  void setMode(uint8_t number,uint8_t mode);
  void setSpeed(uint8_t number,float speed);
  void setDeviceID(uint16_t deviceID);
  void setSpeedPID(uint8_t number,float kp,float ki,float kd);
  void setPosPID(uint8_t number,float kp,float ki,float kd);
  void setCMD(uint8_t number,uint16_t CMD);
  void setSpeedP(uint8_t number,float kp);
  void setSpeedI(uint8_t number,float ki);
  void setSpeedD(uint8_t number,float kd);
  void setPosP(uint8_t number,float kp);
  void setPosI(uint8_t number,float ki);
  void setPosD(uint8_t number,float kd);
  float getSpeedP(uint8_t number);
  float getSpeedI(uint8_t number);
  float getSpeedD(uint8_t number);
  float getPosP(uint8_t number);
  float getPosI(uint8_t number);
  float getPosD(uint8_t number);
  float getSpeed(uint8_t number);
  void setPWM(uint8_t number,uint16_t pwm,uint16_t dir);
  void setMaxSpeed(uint8_t number,float maxSpeed);

  motor M[4];
  uint32_t status = 0;
  uint16_t deviceID = 0;
  uint16_t typeID = 0x55AA;
  uint16_t CMD[4] = {motorParameterCMD,motorParameterCMD,motorParameterCMD,motorParameterCMD};
  void setHeader();
};
