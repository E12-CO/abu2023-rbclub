#include "E12.h"

_E12 E12;

uint8_t _E12::init(uint32_t speed) {
  this->brownOutSetup();
  this->SPISetup(speed);
  this->peripheralSetup();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    return 1;
  }
  //writeChipSelect(0xFF);
  return 0;
}

void _E12::attach(QuadMotorDriver& Q1, uint8_t channel) {
  this->attached[channel] = 1;
  this->deviceType[channel] = QuadMotorDriverTypeID;
  this->deviceAddress[channel] = (uint8_t*)(&Q1);
}

void _E12::broadcast() {
  for (uint8_t i = 0; i < 8; i++) {
    if (attached[i]) {
      switch (deviceType[i]) {
        case QuadMotorDriverTypeID:
          (*(QuadMotorDriver*)(deviceAddress[i])).messageAssemble(this->TXPacket[i]);
          break;
      }
    }
  }
}

void _E12::calculateCRC(uint8_t* message, uint16_t length, uint16_t& crcOut) {
  uint8_t CRC = initial;
  for (uint16_t i = 0; i < length; i++) {
    CRC = CRC ^ (*(message + i));
    for (uint8_t j = 0; j < 8; j++) {
      if (CRC & 0x80) {
        CRC = (CRC << 1) ^ polynomial;
      } else {
        CRC = CRC << 1;
      }
    }
  }
  crcOut = CRC;
}

void _E12::peripheralSetup(){
  pinMode(periData, OUTPUT);
  pinMode(periClock, OUTPUT);
  pinMode(periLatch, OUTPUT);
  digitalWrite(periLatch, HIGH);
}

void _E12::brownOutSetup(){
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
}

void _E12::SPISetup(uint32_t speed){
  pinMode(E12_SCLK, OUTPUT);
  pinMode(E12_MISO, INPUT_PULLUP);
  pinMode(E12_MOSI, OUTPUT);
  pinMode(CS_LATCH, OUTPUT);
  pinMode(CS_CLOCK, OUTPUT);
  pinMode(CS_DATA, OUTPUT);

  SPI.beginTransaction(SPISettings(speed, MSBFIRST, SPI_MODE0));
  SPI.begin(E12_SCLK, E12_MISO, E12_MOSI, 36);
}

void _E12::communicateTimerSetup(){
  timer1 = timerBegin(1, 80, true);

  timerAttachInterrupt(timer1, &onTimer, true);
  timerAlarmWrite(timer1, 2000, true);
  timerAlarmEnable(timer1);
}

void IRAM_ATTR onTimer() {
  E12.broadcast();
  E12.broadcastCounter++;
}