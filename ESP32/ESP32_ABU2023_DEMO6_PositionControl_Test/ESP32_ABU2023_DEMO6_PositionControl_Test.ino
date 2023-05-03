#include "stdint.h"
#include "E12_Function.h"


#define delayTime 500

#define m1x 1.0
#define m1y 1.0
#define m1z 1.0
#define m2x 1.0
#define m2y 1.0
#define m2z 1.0
#define m3x 1.0
#define m3y 1.0
#define m3z 1.0
#define m4x 1.0
#define m4y 1.0
#define m4z 1.0

#define maxSpeed 19999
#define threshold 350
uint8_t packet[1024];
uint64_t timeOutCounter = 0;
volatile uint64_t generalCounter = 0;

uint8_t indConfig = 0b11111111;

uint8_t RXPacket[1024];
uint8_t TXPacket[1024];

uint32_t prevMillis = 0;
uint8_t commuFault = 0;

motorParameter_t motorParameterDefault = { 0, 0, 0, 0, 0, 0, 20000, 0, 0, 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4000, 0, 0 };
speedControl_t speedControlDefault = { 0, 0, 0, 0 };
positionControl_t positionControlDefault = { 0, 0, 0, 0, 0 };
motor motorDefault = { motorParameterDefault,
                       speedControlDefault,
                       positionControlDefault,
                       speedControlCMD };
uint8_t displayEnable = 0;
float speedCoefficient = 0.4;
uint8_t speedFlag[4] = { 0, 0, 0, 0 };
float calculatedSpeed[4] = { 0, 0, 0, 0 };
uint8_t mode[4] = {0,0,0,0};
int64_t mPos[4] = {0,0,0,0};
float mSpeed[4] = { 0, 0, 0, 0 };

motor MOTOR[4] = { motorDefault, motorDefault, motorDefault, motorDefault };

QuadMotorDriver Q1(MOTOR);

uint8_t stateRT = 1, stateRD = 1, stateRL = 1, stateRR = 1;
uint8_t stateLT = 1, stateLD = 1, stateLL = 1, stateLR = 1;

uint8_t laststateRT = 1, laststateRD = 1, laststateRL = 1, laststateRR = 1;
uint8_t laststateLT = 1, laststateLD = 1, laststateLL = 1, laststateLR = 1;

byte incomingByte;
double sensorData[6] = { 0, 0, 0, 0, 0, 0 };
int ind1 = LOW;

typedef struct struct_message {
  uint8_t rt, rd, rl, rr, lt, ld, ll, lr;
  int32_t encl, encr;
  int32_t x, y, z, w;
} struct_message;

struct_message myData;

hw_timer_t *timer1 = NULL;

void calculateSpeed() {
  float RPM[4] = { 0, 0, 0, 0 };
  int16_t xComponent = 0, yComponent = 0, zComponent = 0;

  if (myData.w > 2047 + threshold || myData.w < 2047 - threshold) {
    yComponent = map(myData.w, 0, 4095, 2047, -2048);
  } else {
    yComponent = 0;
  }


  if (myData.z > 2047 + threshold || myData.z < 2047 - threshold) {
    xComponent = map(myData.z, 0, 4095, -2048, 2047);
  } else {
    xComponent = 0;
  }

  if (myData.x > 2047 + threshold || myData.x < 2047 - threshold) {
    zComponent = map(myData.x, 0, 4095, 2047, -2048);
  } else {
    zComponent = 0;
  }

  int16_t M1, M2, M3, M4;

  M1 = (int16_t)m1x * xComponent + m1y * yComponent - m1z * zComponent;
  M2 = (int16_t)-m2x * xComponent + m2y * yComponent - m2z * zComponent;
  M3 = (int16_t)-m3x * xComponent - m3y * yComponent - m3z * zComponent;
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

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
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
  updateSpeed();
  

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
/*
void IRAM_ATTR onTimer() {
  updateSpeed();
  for (uint8_t j = 0; j < 4; j++) {
    Q1.setCMD(j + 1, speedControlCMD);
  }
  Q1.getPacket(TXPacket);
  sendData();
  generalCounter++;
}*/

void setPIDParameters() {
  for (uint8_t i = 0; i < 4; i++) {
    Q1.setSpeedPID(i + 1, 0.95, 15, 0);
  }
}

void setup() {
  setPIDParameters();
  E12BusSetup(16000000);
  for (uint16_t i = 0; i < 256; i++) {
    packet[i] = i + 1;
  }
  packet[400] = 89;

  Serial.begin(1000000);

  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    return;
  }

  //timer1 = timerBegin(1, 80, true);

  //timerAttachInterrupt(timer1, &onTimer, true);
  //timerAlarmWrite(timer1, 2000, true);
  //timerAlarmEnable(timer1);

  esp_now_register_recv_cb(OnDataRecv);
}


void loop() {

  if (millis() - prevMillis > delayTime) {
    if (displayEnable) {
      displayParameter();
      //Serial.println(generalCounter);
    }

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
      laststateRT = stateRT;
    }
  }

  timeOutCounter++;
  if (timeOutCounter > 300) {
    for (uint8_t i = 0; i < 4; i++) {
      calculatedSpeed[i] = 0;
    }
    updateSpeed();
    indConfig &= ~(1 << 5);
    peripheralWrite(0, 0xFF, 0xFF, indConfig);
  } else {
    indConfig |= (1 << 5);
    peripheralWrite(0, 0xFF, 0xFF, indConfig);
  }
  serialReceive();

  for (uint8_t j = 0; j < 4; j++) {
    Q1.setCMD(j + 1, motorParameterCMD);
  }
  Q1.getPacket(TXPacket);
  sendData();
  for (uint8_t i = 0; i < 9; i++) {
    updateSpeed();
    updatePos();
    for (uint8_t j = 0; j < 4; j++) {
      if(mode[j] == 0){
        Q1.setCMD(j + 1, speedControlCMD);
      }
      else if (mode[j] == 1){
        Q1.setCMD(j + 1, positionControlCMD);
      }
      
    }
    Q1.getPacket(TXPacket);
    sendData();
  }


  indConfig &= ~(1 << 0);
  peripheralWrite(0, 0xFF, 0xFC, indConfig);
  delay(1);
  indConfig |= (1 << 0);
  peripheralWrite(0, 0xFF, 0xFF, indConfig);
}
