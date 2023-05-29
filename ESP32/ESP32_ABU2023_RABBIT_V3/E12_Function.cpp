#include "E12_Function.h"

void E12BusSetup(uint32_t speed)
{
  pinMode(E12_SCLK, OUTPUT);
  pinMode(E12_MISO, INPUT_PULLUP);
  pinMode(E12_MOSI, OUTPUT);
  pinMode(CS_LATCH, OUTPUT);
  pinMode(CS_CLOCK, OUTPUT);
  pinMode(CS_DATA, OUTPUT);
  pinMode(periData, OUTPUT);
  pinMode(periClock, OUTPUT);
  pinMode(periLatch, OUTPUT);
  digitalWrite(periLatch, HIGH);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  SPI.beginTransaction(SPISettings(speed, MSBFIRST, SPI_MODE0));
  SPI.begin(E12_SCLK, E12_MISO, E12_MOSI, 36);
  
  writeChipSelect(0xFF);
}


void writeChipSelect(uint8_t config){
  GPIO.out_w1tc = (1<<CS_LATCH);
  for(uint8_t i = 0;i<8;i++){
    GPIO.out_w1tc = (1<<CS_CLOCK);
    GPIO.out_w1ts = ((config & 0b1 << i) && 1) << CS_DATA;
    GPIO.out_w1tc = (!((config & 0b1 << i) && 1)) << CS_DATA;
    GPIO.out_w1ts = (1<<CS_CLOCK);
  }
  GPIO.out_w1ts = (1<<CS_LATCH);
}

void startChipSelect(){
  GPIO.out_w1tc = (1<<CS_LATCH);
  GPIO.out_w1tc = (1<<CS_CLOCK);
  GPIO.out_w1tc = (1<<CS_DATA);
  GPIO.out_w1ts = (1<<CS_CLOCK);
  GPIO.out_w1ts = (1<<CS_LATCH);
}

void shiftChipSelect(){
  GPIO.out_w1tc = (1<<CS_LATCH);
  GPIO.out_w1tc = (1<<CS_CLOCK);
  GPIO.out_w1ts = (1<<CS_DATA);
  GPIO.out_w1ts = (1<<CS_CLOCK);
  GPIO.out_w1ts = (1<<CS_LATCH);
}

void sendData() {
  writeChipSelect(0b11111101);
  delayMicroseconds(1);
  SPI.transferBytes(TXPacket,RXPacket, packetSize+1);
  delayMicroseconds(1);
  writeChipSelect(0xFF);
  delay(1);
  writeChipSelect(0b11111011);
  delayMicroseconds(1);
  SPI.transferBytes(TXPacket2,RXPacket2, packetSize+1);
  delayMicroseconds(1);
  writeChipSelect(0xFF);
}

void peripheralWrite(uint8_t genOut, uint8_t LEDR, uint8_t LEDL, uint8_t ind) {
  GPIO.out_w1tc = (1 << periLatch);
  for (uint8_t i = 0; i < 8; i++) {
    GPIO.out_w1tc = (1 << periClock);
    GPIO.out_w1ts = ((ind & 0b10000000 >> i) && 1) << periData;
    GPIO.out_w1tc = (!((ind & 0b10000000 >> i) && 1)) << periData;
    GPIO.out_w1ts = (1 << periClock);
  }
  for (uint8_t i = 0; i < 8; i++) {
    GPIO.out_w1tc = (1 << periClock);
    GPIO.out_w1ts = ((LEDL & 0b1 << i) && 1) << periData;
    GPIO.out_w1tc = (!((LEDL & 0b1 << i) && 1)) << periData;
    GPIO.out_w1ts = (1 << periClock);
  }
  for (uint8_t i = 0; i < 8; i++) {
    GPIO.out_w1tc = (1 << periClock);
    GPIO.out_w1ts = ((LEDR & 0b1 << i) && 1) << periData;
    GPIO.out_w1tc = (!((LEDR & 0b1 << i) && 1)) << periData;
    GPIO.out_w1ts = (1 << periClock);
  }
  for (uint8_t i = 0; i < 8; i++) {
    GPIO.out_w1tc = (1 << periClock);
    GPIO.out_w1ts = ((genOut & 0b1 << i) && 1) << periData;
    GPIO.out_w1tc = (!((genOut & 0b1 << i) && 1)) << periData;
    GPIO.out_w1ts = (1 << periClock);
  }
  GPIO.out_w1ts = (1 << periLatch);

}
