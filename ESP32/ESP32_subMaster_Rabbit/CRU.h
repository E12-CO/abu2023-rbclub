#ifndef CRU_INCLUDED
#define CRU_INCLUDED
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <Esp.h>

#define DATA 4
#define LATCH 2
#define CLOCK 15
#define SERVO1 22
#define SERVO2 23
#define EN1 13
#define EN2 12
#define EN3 14
#define EN4 27


class _CRU{
public:
  void writeMotor(uint8_t number,uint16_t duty,uint8_t dir);
  void writeInd(uint8_t config);
  void writeMicroseconds(uint8_t number,uint16_t micro);
  void update();
  void init();
private:
  uint16_t M[4];
  uint8_t inConfig;
  uint8_t indConfig;

};
extern _CRU CRU;
#endif
