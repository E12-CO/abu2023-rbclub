#include "stdlib.h"
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <SPI.h>
#include <esp_now.h>
#include "QuadMotorDriver.h"

#define CS_LATCH 2
#define CS_CLOCK 4
#define CS_DATA 15
#define E12_SCLK 14
#define E12_MISO 12
#define E12_MOSI 13
#define periData 27
#define periClock 25
#define periLatch 26

#define initial 0x00
#define polynomial 0xB7

#define QuadMotorDriverTypeID 0x55AA

static hw_timer_t *timer1 = NULL;

class _E12 {
public:
  void calculateCRC(uint8_t* message, uint16_t length, uint16_t& crcOut);
  uint8_t TXPacket[8][1024];
  uint8_t RXPacket[8][1024];
  uint8_t init(uint32_t speed);
  void attach(QuadMotorDriver& Q1, uint8_t channel);
  uint8_t attached[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  uint16_t deviceType[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  uint8_t* deviceAddress[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  void writeChipSelect(uint8_t config);
  void peripheralWrite(uint8_t genOut, uint8_t LEDR, uint8_t LEDL, uint8_t ind);
  void setCsConfig(uint8_t number);
  void resetCsConfig(uint8_t number);
  void shiftCsConfig(uint8_t number);
  void broadcast();
  uint64_t broadcastCounter = 0;
private:
  void peripheralSetup();
  void brownOutSetup();
  void SPISetup(uint32_t speed);
  void communicateTimerSetup();
  uint8_t numberOfSlave();
  uint8_t csConfig = 0;
  uint8_t scsConfig = 0;
  uint8_t ledR = 0;
  uint8_t ledL = 0;
  uint8_t ind = 0;
  uint8_t genOut = 0;
  
};

extern _E12 E12;

void IRAM_ATTR onTimer();

