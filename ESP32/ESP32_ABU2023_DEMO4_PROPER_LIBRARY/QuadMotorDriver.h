#include "dcMotor.h"

#define transmitHeader 0x63010298
#define receiveHeader 0x64010377
#define perMotorLength 120
#define headerOffset 12
#define cmdOffset 118
#define typeIDOffset 4
#define deviceIDOffset 6
#define statusOffset 8


class QuadMotorDriver {
public:
  QuadMotorDriver(uint16_t deviceID);
  void attach(dcMotor& motor,uint8_t channel);
  void messageAssemble(uint8_t* message);
  void setDeviceID(uint16_t deviceID);
  void getTypeID(uint16_t& typeID);
private:
  uint16_t typeID = 0x55AA;
  uint16_t deviceID = 0;
  uint32_t status = 0;
  uint8_t* motorAddress[4];
  uint8_t attached[4];
};
