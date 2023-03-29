#define CS_LATCH 2
#define CS_CLOCK 4
#define CS_DATA 15
#define E12_SCLK 14
#define E12_MISO 12
#define E12_MOSI 13
#define periData 27
#define periClock 25
#define periLatch 26
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include<WiFi.h>
#include<esp_now.h>
#include<SPI.h>

uint8_t packet[1024];

#define polynomial 0xB7
#define initial 0xFF
#define CONFIG 0x01
#define NORMAL 0x02

uint32_t CRC8_Finder(uint8_t *packet, uint8_t length) {
  uint16_t iteration = (uint16_t)(length);
  uint8_t CRC = initial;
  for (uint16_t i = 0; i < iteration; i++) {
    CRC = CRC ^ (*(packet + i));
    for (uint8_t j = 0; j < 32; j++) {
      if (CRC & 0x80) {
        CRC = CRC << 1;
        CRC ^= polynomial;
      }
      else {
        CRC = CRC << 1;
      }
    }
  }
  return CRC;
}


void peripheralWrite(uint8_t genOut,uint8_t LEDR,uint8_t LEDL,uint8_t ind){
  GPIO.out_w1tc = (1<<periLatch);
  for(uint8_t i = 0;i<8;i++){
    GPIO.out_w1tc = (1<<periClock);
    GPIO.out_w1ts = ((ind & 0b10000000 >> i) && 1) << periData;
    GPIO.out_w1tc = (!((ind & 0b10000000 >> i) && 1)) << periData;
    GPIO.out_w1ts = (1<<periClock);
  }
  for(uint8_t i = 0;i<8;i++){
    GPIO.out_w1tc = (1<<periClock);
    GPIO.out_w1ts = ((LEDL & 0b1 << i) && 1) << periData;
    GPIO.out_w1tc = (!((LEDL & 0b1 << i) && 1)) << periData;
    GPIO.out_w1ts = (1<<periClock);
  }
  for(uint8_t i = 0;i<8;i++){
    GPIO.out_w1tc = (1<<periClock);
    GPIO.out_w1ts = ((LEDR & 0b1 << i) && 1) << periData;
    GPIO.out_w1tc = (!((LEDR & 0b1 << i) && 1)) << periData;
    GPIO.out_w1ts = (1<<periClock);
  }
  for(uint8_t i = 0;i<8;i++){
    GPIO.out_w1tc = (1<<periClock);
    GPIO.out_w1ts = ((genOut & 0b1 << i) && 1) << periData;
    GPIO.out_w1tc = (!((genOut & 0b1 << i) && 1)) << periData;
    GPIO.out_w1ts = (1<<periClock);
  }
  GPIO.out_w1ts = (1<<periLatch);
  
}

void setup() {
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
  SPI.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE0));
  SPI.begin(E12_SCLK, E12_MISO, E12_MOSI, 36);
  writeChipSelect(0xFF);

  for (uint8_t i = 0; i < 200; i++) {
    packet[i] = i;
  }
  packet[198] = 200;
  packet[199] = 185;
  Serial.begin(115200);


}

void loop() {
  writeChipSelect(0xFE);
  SPI.writeBytes(packet, 65);
  writeChipSelect(0xFF);
  peripheralWrite(0, 0xFF, 0xFC, 0b11111110);
  delay(100);
  peripheralWrite(0, 0xFF, 0xFF, 0b11111111);
  delay(200);
  Serial.println((uint8_t)CRC8_Finder(packet,199));


}
