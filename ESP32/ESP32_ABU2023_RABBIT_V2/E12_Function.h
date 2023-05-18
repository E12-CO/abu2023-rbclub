#include "PACKET.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include<WiFi.h>
#include<esp_now.h>
#include<SPI.h>

#define CS_LATCH 2
#define CS_CLOCK 4
#define CS_DATA 15
#define E12_SCLK 14
#define E12_MISO 12
#define E12_MOSI 13
#define periData 27
#define periClock 25
#define periLatch 26
static const uint32_t BUFFER_SIZE = 512;
extern uint8_t RXPacket[1024];
extern uint8_t TXPacket[1024];
extern uint8_t RXPacket2[1024];
extern uint8_t TXPacket2[1024];
extern uint8_t RXPacket3[1024];
extern uint8_t TXPacket3[1024];
extern uint8_t packet[1024];
extern uint64_t timeOutCounter;
extern uint8_t speedFlag[4];
extern float calculatedSpeed[4];
extern float mSpeed[4];
extern QuadMotorDriver Q1;
extern uint8_t displayEnable;

void E12BusSetup(uint32_t speed);
void writeChipSelect(uint8_t config);
void startChipSelect();
void shiftChipSelect();
void sendData();
void peripheralWrite(uint8_t genOut, uint8_t LEDR, uint8_t LEDL, uint8_t ind);
void serialReceive();
void displayParameter();
void updateSpeed();