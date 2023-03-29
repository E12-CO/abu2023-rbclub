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
#include "driver/spi_master.h"
#include<WiFi.h>
#include<esp_now.h>
#include<SPI.h>

uint8_t packet[1024];

#define polynomial 0xB7
#define initial 0x00
#define CONFIG 0x01
#define NORMAL 0x02

uint8_t RXPacket[1024];
uint8_t message[1024];

typedef struct {
  float kp;
  float ki;
  float kd;
  float speed;
} motor;
motor m1 = {1, 0.002, 0.5, 2500};
motor m2 = {0, 0, 0, 0};
motor m3 = {0, 0, 0, 0};
motor m4 = {0, 0, 0, 0};

typedef struct {
  uint16_t a;
  uint16_t b;
} motor2;

motor2 m5 = {0xABCD, 0x0123};
motor2 m6 = {0x4567, 0x89AB};

uint32_t CRC8_Finder(uint8_t *packet, uint16_t length) {
  uint8_t CRC = initial;
  for (uint16_t i = 0; i < length; i++) {
    CRC = CRC ^ (*(packet + i));
    for (uint8_t j = 0; j < 8; j++) {
      if (CRC & 0x80) {
        CRC = (CRC << 1) ^ polynomial;
      }
      else {
        CRC = CRC << 1;
      }
    }
  }
  return CRC;
}

void messageAssembler(uint8_t *message) {
  *(uint32_t*)(message) = (uint32_t)0x63010298;
  *(motor *)(message + 4) = m1;
  *(motor *)(message + 20) = m2;
  *(motor *)(message + 36) = m3;
  *(motor *)(message + 52) = m4;
  *(uint8_t*)(message + 68) = CRC8_Finder(message, 68);
}

void messageAssembler2(uint8_t *message) {
  *((motor2 *)(message)) = m5;
  *((motor2 *)(message + 4)) = m6;
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
  SPI.beginTransaction(SPISettings(2400000, MSBFIRST, SPI_MODE0));
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
  while (Serial.available() > 0) {
    String temp = Serial.readString();
    if (temp[1] == 'p') {
      m1.kp = (temp.substring(3)).toFloat();
    }
    if (temp[1] == 'i') {
      m1.ki = (temp.substring(3)).toFloat();
    }
    if (temp[1] == 'd') {
      m1.kd = (temp.substring(3)).toFloat();
    }
    if (temp[1] == 's') {
      m1.speed = (temp.substring(3)).toFloat();
    }
    Serial.print("kp : ");
    Serial.println(m1.kp, 10);
    Serial.print("ki : ");
    Serial.println(m1.ki, 10);
    Serial.print("kd : ");
    Serial.println(m1.kd, 10);
    Serial.print("Speed : ");
    Serial.println(m1.speed, 10);
    Serial.println("");

    if (temp[1] == 'a') {

      for (uint8_t i = 0; i < 69; i++) {

        Serial.print(i);
        Serial.print(" ");
        Serial.println(RXPacket[i]);
      }
    }
  }

  messageAssembler(message);
  writeChipSelect(0xFE);
  for (uint8_t i = 0; i < 69; i++) {
    RXPacket[i] = SPI.transfer(message[i]);
  }
  writeChipSelect(0xFF);

  peripheralWrite(0, 0xFF, 0xFC, 0b11111110);
  delay(1);
  peripheralWrite(0, 0xFF, 0xFF, 0b11111111);
  delay(2);


}
