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

#define delayTime 500

#define m1x       1.0
#define m1y       1.0
#define m1z       1.0
#define m2x       1.0
#define m2y       1.0
#define m2z       1.0
#define m3x       1.0
#define m3y       1.0
#define m3z       1.0
#define m4x       1.0
#define m4y       1.0
#define m4z       1.0

#define maxSpeed   12000
#define threshold  350
#define dataSize 121
uint8_t packet[1024];

uint8_t indConfig = 0b11111111;

uint64_t timeOutCounter = 0;

#define polynomial 0xB7
#define initial 0x00
#define CONFIG 0x01
#define NORMAL 0x02

uint8_t RXPacket[1024];
uint8_t message[1024];

uint32_t prevMillis = 0;

typedef struct {
  float kp;
  float ki;
  float kd;
  float speed;
} motor;
motor m1 = {0, 0, 0, 0};
motor m2 = {0, 0, 0, 0};
motor m3 = {0, 0, 0, 0};
motor m4 = {0, 0, 0, 0};

float speedCoefficient = 0.5;
uint8_t speedFlag[4] = {0, 0, 0, 0};
float calculatedSpeed[4] = {0, 0, 0, 0};
float mSpeed[4] = {0, 0, 0, 0};
uint8_t stateRT = 1, stateRD = 1, stateRL = 1, stateRR = 1;
uint8_t stateLT = 1, stateLD = 1, stateLL = 1, stateLR = 1;

uint8_t laststateRT = 1, laststateRD = 1, laststateRL = 1, laststateRR = 1;
uint8_t laststateLT = 1, laststateLD = 1, laststateLL = 1, laststateLR = 1;

byte incomingByte;
double sensorData[6] = {0, 0, 0, 0, 0, 0};
int ind1 = LOW;

typedef struct struct_message {
  uint8_t rt, rd, rl, rr, lt, ld, ll, lr;
  int32_t encl, encr;
  int32_t x, y, z, w;
} struct_message;

struct_message myData;

void calculateSpeed() {
  float RPM[4] = {0, 0, 0, 0};
  int16_t xComponent = 0, yComponent = 0, zComponent = 0;

  if (myData.w > 2047 + threshold || myData.w < 2047 - threshold) {
    yComponent = map(myData.w, 0, 4095, -2048, 2047);
  }
  else {
    yComponent = 0;
  }


  if (myData.z > 2047 + threshold || myData.z < 2047 - threshold) {
    xComponent = map(myData.z, 0, 4095, 2047, -2048);
  }
  else {
    xComponent = 0;
  }

  if (myData.x > 2047 + threshold || myData.x < 2047 - threshold) {
    zComponent = map(myData.x, 0, 4095, -2048, 2047);
  }
  else {
    zComponent = 0;
  }

  int16_t M1, M2, M3, M4;

  M1 = (int16_t)m1x * xComponent + m1y * yComponent - m1z * zComponent;
  M2 = (int16_t) - m2x * xComponent + m2y * yComponent - m2z * zComponent;
  M3 = (int16_t) - m3x * xComponent - m3y * yComponent - m3z * zComponent;
  M4 = (int16_t)m4x * xComponent - m4y * yComponent - m4z * zComponent;

  int16_t maxComponent = abs(M1);
  if (maxComponent < abs(M2)) {
    maxComponent = abs(M2);
  }
  if (maxComponent < abs(M3)) {
    maxComponent = abs(M3);
  }
  if (maxComponent < abs(M4)) {
    maxComponent = abs(M4);
  }
  if (maxComponent < 2048) {
    maxComponent = 2047;
  }


  calculatedSpeed[0] = (float)speedCoefficient * maxSpeed * M1 / maxComponent;
  calculatedSpeed[1] = (float)speedCoefficient * maxSpeed * M2 / maxComponent;
  calculatedSpeed[2] = (float)speedCoefficient * maxSpeed * M3 / maxComponent;
  calculatedSpeed[3] = (float)speedCoefficient * maxSpeed * M4 / maxComponent;

}

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
  *(uint8_t*)(message + dataSize - 1) = CRC8_Finder(message, dataSize - 1);
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


void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  timeOutCounter = 0;
  stateRT = myData.rt;
  stateRD = myData.rd;
  stateRL = myData.rl;
  stateRR = myData.rr;
  stateLT = myData.lt;
  stateLD = myData.ld;
  stateLL = myData.ll;
  stateLR = myData.lr;

  calculateSpeed();

  /*
    Serial.print(myData.rt);
    Serial.print(" ");
    Serial.print (myData.rd);
    Serial.print (" ");
    Serial.print (myData.rl);
    Serial.print(" ");
    Serial.print (myData.rr);
    Serial.print(" ");
    Serial.print (myData.lt);
    Serial.print(" ");
    Serial.print (myData.ld);
    Serial.print(" ");
    Serial.print (myData.ll);
    Serial.print(" ");
    Serial.print (myData.lr);
    Serial.print(" ");
    Serial.print (myData.encl);
    Serial.print(" ");
    Serial.print (myData.encr);
    Serial.print(" ");
    Serial.print (myData.x);
    Serial.print(" ");
    Serial.print (myData.y);
    Serial.print(" ");
    Serial.print (myData.z);
    Serial.print(" ");
    Serial.print (myData.w);
    Serial.println(" ");*/

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
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
  SPI.begin(E12_SCLK, E12_MISO, E12_MOSI, 36);
  writeChipSelect(0xFF);

  for (uint8_t i = 0; i < 200; i++) {
    packet[i] = i;
  }
  packet[198] = 200;
  packet[199] = 185;
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);



}

void updateSpeed() {
  if (speedFlag[0]) {
    m1.speed = mSpeed[0];
  }
  else {
    m1.speed = calculatedSpeed[0];
  }
  if (speedFlag[1]) {
    m2.speed = mSpeed[1];
  }
  else {
    m2.speed = calculatedSpeed[1];
  }
  if (speedFlag[2]) {
    m3.speed = mSpeed[2];
  }
  else {
    m3.speed = calculatedSpeed[2];
  }
  if (speedFlag[3]) {
    m4.speed = mSpeed[3];
  }
  else {
    m4.speed = calculatedSpeed[3];
  }
}

void sendData() {
  messageAssembler(message);
  writeChipSelect(0xFE);
  delayMicroseconds(1);
  for (uint8_t i = 0; i < dataSize; i++) {
    RXPacket[i] = SPI.transfer(message[i]);
  }
  delayMicroseconds(1);
  writeChipSelect(0xFF);
}

void displayParameter() {
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.print("kp : "); Serial.print(m1.kp, 10); Serial.print(" "); Serial.print(m2.kp, 10); Serial.print(" "); Serial.print(m3.kp, 10); Serial.print(" "); Serial.println(m4.kp, 10);
  Serial.print("ki : "); Serial.print(m1.ki, 10); Serial.print(" "); Serial.print(m2.ki, 10); Serial.print(" "); Serial.print(m3.ki, 10); Serial.print(" "); Serial.println(m4.ki, 10);
  Serial.print("kd : "); Serial.print(m1.kd, 10); Serial.print(" "); Serial.print(m2.kd, 10); Serial.print(" "); Serial.print(m3.kd, 10); Serial.print(" "); Serial.println(m4.kd, 10);
  Serial.print("ks : "); Serial.print(m1.speed, 10); Serial.print(" "); Serial.print(m2.speed, 10); Serial.print(" "); Serial.print(m3.speed, 10); Serial.print(" "); Serial.println(m4.speed, 10);
  Serial.print("en : "); Serial.print(speedFlag[0]); Serial.print("            "); Serial.print(speedFlag[1]); Serial.print("            "); Serial.print(speedFlag[2]); Serial.print("            "); Serial.print(speedFlag[3]); Serial.println("           ");

}
void loop() {
 /* while (Serial.available() > 0) {
    String temp = Serial.readString();
    if (temp[1] == 'p') {
      if (temp[3] == '1') {
        m1.kp = (temp.substring(5)).toFloat();
      }
      else if (temp[3] == '2') {
        m2.kp = (temp.substring(5)).toFloat();
      }
      else if (temp[3] == '3') {
        m3.kp = (temp.substring(5)).toFloat();
      }
      else if (temp[3] == '4') {
        m4.kp = (temp.substring(5)).toFloat();
      }
      else if (temp[3] == 'a') {
        m1.kp = (temp.substring(5)).toFloat();
        m2.kp = (temp.substring(5)).toFloat();
        m3.kp = (temp.substring(5)).toFloat();
        m4.kp = (temp.substring(5)).toFloat();
      }

    }
    if (temp[1] == 'i') {
      if (temp[3] == '1') {
        m1.ki = (temp.substring(5)).toFloat();
      }
      else if (temp[3] == '2') {
        m2.ki = (temp.substring(5)).toFloat();
      }
      else if (temp[3] == '3') {
        m3.ki = (temp.substring(5)).toFloat();
      }
      else if (temp[3] == '4') {
        m4.ki = (temp.substring(5)).toFloat();
      }
      else if (temp[3] == 'a') {
        m1.ki = (temp.substring(5)).toFloat();
        m2.ki = (temp.substring(5)).toFloat();
        m3.ki = (temp.substring(5)).toFloat();
        m4.ki = (temp.substring(5)).toFloat();
      }
    }
    if (temp[1] == 'd') {
      if (temp[3] == '1') {
        m1.kd = (temp.substring(5)).toFloat();
      }
      else if (temp[3] == '2') {
        m2.kd = (temp.substring(5)).toFloat();
      }
      else if (temp[3] == '3') {
        m3.kd = (temp.substring(5)).toFloat();
      }
      else if (temp[3] == '4') {
        m4.kd = (temp.substring(5)).toFloat();
      }
      else if (temp[3] == 'a') {
        m1.kd = (temp.substring(5)).toFloat();
        m2.kd = (temp.substring(5)).toFloat();
        m3.kd = (temp.substring(5)).toFloat();
        m4.kd = (temp.substring(5)).toFloat();
      }
    }
    if (temp[1] == 's') {
      if (temp[3] == '1') {
        mSpeed[0] = (temp.substring(5)).toFloat();
      }
      else if (temp[3] == '2') {
        mSpeed[1] = (temp.substring(5)).toFloat();
      }
      else if (temp[3] == '3') {
        mSpeed[2] = (temp.substring(5)).toFloat();
      }
      else if (temp[3] == '4') {
        mSpeed[3] = (temp.substring(5)).toFloat();
      }
      else if (temp[3] == 'a') {
        mSpeed[0] = (temp.substring(5)).toFloat();
        mSpeed[1] = (temp.substring(5)).toFloat();
        mSpeed[2] = (temp.substring(5)).toFloat();
        mSpeed[3] = (temp.substring(5)).toFloat();
      }
    }
    if (temp[1] == 'n') {
      if (temp[3] == '1') {
        speedFlag[0] = (temp.substring(5)).toInt();
      }
      else if (temp[3] == '2') {
        speedFlag[1] = (temp.substring(5)).toInt();
      }
      else if (temp[3] == '3') {
        speedFlag[2] = (temp.substring(5)).toInt();
      }
      else if (temp[3] == '4') {
        speedFlag[3] = (temp.substring(5)).toInt();
      }
      else if (temp[3] == 'a') {
        speedFlag[0] = (temp.substring(5)).toInt();
        speedFlag[1] = (temp.substring(5)).toInt();
        speedFlag[2] = (temp.substring(5)).toInt();
        speedFlag[3] = (temp.substring(5)).toInt();
      }
    }
    updateSpeed();

    if (temp[1] == 'm') {
      Serial.println(WiFi.macAddress());
    }
    if (temp[1] == 'a') {

      for (uint8_t i = 0; i < 69; i++) {

        Serial.print(i);
        Serial.print(" ");
        Serial.println(RXPacket[i]);
      }
    }
  }
*/
  updateSpeed();

  if(millis() - prevMillis > delayTime){
    displayParameter();
    prevMillis = millis();  
  }


  if (stateRT != laststateRT) {
    laststateRT = stateRT;

    if (stateRT != LOW) {
      //        Serial.write(sensorData,6);

      Serial.print(sensorData[0]);
      Serial.print(" ");
      Serial.print(sensorData[1]);
      Serial.print(" ");
      Serial.print(sensorData[2]);
      Serial.print(" ");
      Serial.print(sensorData[3]);
      Serial.print(" ");
      Serial.print(sensorData[4]);
      Serial.print(" ");
      Serial.println(sensorData[5]);
      //        Serial.println(sensorData);
      /**/
      laststateRT = stateRT;
    }
  }
  timeOutCounter++;
  if (timeOutCounter > 300) {
    indConfig &= ~(1 << 5);
    peripheralWrite(0, 0xFF, 0xFF, indConfig);
  }
  else {
    indConfig |= (1 << 5);
    peripheralWrite(0, 0xFF, 0xFF, indConfig);
  }

  sendData();
  indConfig &= ~(1 << 0);
  peripheralWrite(0, 0xFF, 0xFC, indConfig);
  delay(1);
  indConfig |= (1 << 0);
  peripheralWrite(0, 0xFF, 0xFF, indConfig);

}
