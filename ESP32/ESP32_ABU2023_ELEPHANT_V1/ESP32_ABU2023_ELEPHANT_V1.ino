#include "stdint.h"
#include "E12_Function.h"

uint8_t broadcastAddress[] = { 0xA0, 0xB7, 0x65, 0x60, 0xB7, 0x70 }; // ไป SubMaster
uint8_t returnAddressPalm[] = { 0x94, 0xE6, 0x86, 0x3C, 0x94, 0x5C }; // ไป รีโมท ปาล์ม
uint8_t returnAddressJame[] = { 0xE0, 0x5A, 0x1B, 0xA1, 0x04, 0x40 }; // ไป รีโมท เจมส์

volatile esp_err_t result;

#define delayTime 500
int16_t maxComponent = 0;
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

//Settable Parameter
#define threshold 350
#define maxSpeed 24000
#define maxRollerSpeed 5200
#define maxSpeedCoefficient 0.5
#define minSpeedCoefficient 0.1
#define maxPWMCoefficient 1
#define minPWMCoefficient 0.7
#define communicationTimeOut 200



uint8_t packet[1024];
uint64_t timeOutCounterPalm = 0;
uint64_t timeOutCounterJame = 0;
volatile uint64_t generalCounter = 0;



uint8_t indConfig = 0b11111111;
uint8_t robotJameStarted = 1;
uint8_t robotPalmStarted = 1;

uint8_t RXPacket[1024];
uint8_t TXPacket[1024];
uint8_t RXPacket2[1024];
uint8_t TXPacket2[1024];
uint8_t RXPacket3[1024];
uint8_t TXPacket3[1024];
float RPM[4] = { 0, 0, 0, 0 };
int16_t xComponent = 0, yComponent = 0, zComponent = 0;
uint32_t prevMillis = 0;
uint8_t commuFault = 0;
int16_t M1, M2, M3, M4;
uint8_t geegeeMode = 0;

motorParameter_t motorParameterDefault = { 0, 0, 0, 0, 0, 0, 24000, 0, 0, 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4000, 0, 0 };
speedControl_t speedControlDefault = { 0, 0, 0, 0 };
positionControl_t positionControlDefault = { 0, 0, 0, 0, 0 };
motor motorDefault = { motorParameterDefault,
                       speedControlDefault,
                       positionControlDefault,
                       speedControlCMD };
uint8_t displayEnable = 0;

float speedCoefficientJame = 0.2;
float speedCoefficientPalm = 0.2;
float manualSpeedCoefficientJame = 0.7;
float manualSpeedCoefficientPalm = 0.7;

uint8_t speedFlag[4] = { 0, 0, 0, 0 };

float mSpeed[4] = { 0, 0, 0, 0 };
uint32_t counter5 = 0;

float calculatedSpeed[4] = { 0, 0, 0, 0 }; //ClosedLoop Mode (not geegeemode)
int16_t calculatedPWM[4] = { 0, 0, 0, 0 }; //OpenLoop Mode (geegeemode)

motor MOTOR[4] = { motorDefault, motorDefault, motorDefault, motorDefault };

QuadMotorDriver Q1(MOTOR);
QuadMotorDriver Q2(MOTOR);
QuadMotorDriver Q3(MOTOR);
uint32_t prevMillisElevation = 0;
uint32_t prevMillisLoad = 0;

uint8_t stateJameRT = 1, stateJameRD = 1, stateJameRL = 1, stateJameRR = 1;
uint8_t stateJameLT = 1, stateJameLD = 1, stateJameLL = 1, stateJameLR = 1;
uint8_t stateJameCom1 = 1, stateJameCom2 = 1, stateJameCom3 = 1, stateJameCom4 = 1, stateJameCom5 = 1;

uint8_t statePalmRT = 1, statePalmRD = 1, statePalmRL = 1, statePalmRR = 1;
uint8_t statePalmLT = 1, statePalmLD = 1, statePalmLL = 1, statePalmLR = 1;
uint8_t statePalmCom1 = 1, statePalmCom2 = 1, statePalmCom3 = 1, statePalmCom4 = 1, statePalmCom5 = 1;

byte incomingByte;
double sensorData[6] = { 0, 0, 0, 0, 0, 0 };
int ind1 = LOW;
uint8_t master = 0;

float upperRollerSpeed = 0;
float lowerRollerSpeed = 0;

uint8_t ackCounterPalm = 0;
uint8_t ackCounterJame = 0;

typedef struct struct_message {
  uint32_t started;
  uint32_t cycleMillis;
  int32_t id;
  int32_t encl, encr;
  int32_t x, y, z, w;
  uint8_t rt, rd, rl, rr, lt, ld, ll, lr;
  uint8_t com1, com2, com3, com4, com5;
} struct_message;

int32_t prevJameEncL = 0;
int32_t prevJameEncR = 0;
int32_t prevPalmEncL = 0;
int32_t prevPalmEncR = 0;

struct_message bufferData, kineticData, aimmingData;

struct_message defaultMessage = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

hw_timer_t *timer1 = NULL;

typedef struct {
  uint32_t acknowledge;
  float speedCoeffcient;
  uint32_t prevCycleMillis;
} returnStruct_t;
returnStruct_t returnPacketPalm;
returnStruct_t returnPacketJame;


typedef struct {
  uint8_t gripper = 2;
} sub_message_t;

sub_message_t sub_message;

uint16_t cmdCounter = 0;
uint32_t firstMillis = 0;
int32_t elevationCounter = 0;
int32_t prevElevationCounter = 0;
#define elevationInterval 100 //not more than 300 or less than 30
uint32_t prevElevationMillis = 0;
uint8_t elevationDir = 0;
uint8_t elevationRunning = 0;
#define maxPWM 4200

void calculateElevationSpeed(struct_message myData) {

  if (elevationCounter > prevElevationCounter && !elevationRunning) {
    prevElevationMillis = millis();

    elevationDir = 2;
    if (!myData.com2) {
      Q2.setPWM(3, 3600, elevationDir);

    } else {
      Q2.setPWM(3, 4200, elevationDir);
    }
    elevationRunning = 1;
  } else if (elevationCounter < prevElevationCounter && !elevationRunning) {
    prevElevationMillis = millis();
    elevationDir = 1;
    if (!myData.com2) {
      Q2.setPWM(3, 3600, elevationDir);
    } else {
      Q2.setPWM(3, 4200, elevationDir);
    }
    elevationRunning = 1;
  }

  if (millis() - prevElevationMillis > elevationInterval && elevationRunning) {

    if (elevationDir == 2) {
      prevElevationCounter++;
    } else if (elevationDir == 1) {
      prevElevationCounter--;
    } else {
      prevElevationCounter = elevationCounter;
    }

    if (prevElevationCounter == elevationCounter) {
      Q2.setPWM(3, 4200, 0);
    }
    elevationRunning = 0;
  }
}

void calculateGripperSpeed(struct_message myData) {
  if (!myData.ll) {
    sub_message.gripper = 2;
  } else if (!myData.lr) {
    sub_message.gripper = 4;
  } else {
  }
  result = esp_now_send(broadcastAddress, (uint8_t *)&sub_message, sizeof(sub_message));
}

void calculateChassisFeederSpeed(struct_message myData) {
  if (!myData.rr) {
    Q3.setPWM(3, 3600, 0x01);
  } else if (!myData.rl) {
    Q3.setPWM(3, 3600, 0x02);
  } else {
    Q3.setPWM(3, 4200, 0);
  }
}

void calculateLiftSpeed(struct_message myData) {
  if (!myData.lt) {
    Q3.setPWM(1, 4200, 0x01);
  } else if (!myData.ld) {
    Q3.setPWM(1, 3600, 0x02);
  } else {
    Q3.setPWM(1, 4200, 0);
  }
  if (!myData.rt) {
    Q3.setPWM(2, 4200, 0x01);
  } else if (!myData.rd) {
    Q3.setPWM(2, 3600, 0x02);
  } else {
    Q3.setPWM(2, 4200, 0);
  }
}

void calculateRollerSpeed(struct_message myData) {
  if (!myData.rd) {
    upperRollerSpeed = 0;
    lowerRollerSpeed = 0;
  } else if (!myData.rt) {
    upperRollerSpeed = 2000;
    lowerRollerSpeed = -2000;
  } else if (!myData.lt) {
    upperRollerSpeed = 0.8 * maxRollerSpeed;
    lowerRollerSpeed = -0.7 * maxRollerSpeed;
  } else if (!myData.ld) {
    upperRollerSpeed = 0.7 * maxRollerSpeed;
    lowerRollerSpeed = -0.6 * maxRollerSpeed;
  } else if (!myData.ll) {
    upperRollerSpeed = 0.6 * maxRollerSpeed;
    lowerRollerSpeed = -0.5 * maxRollerSpeed;
  } else if (!myData.lr) {
    upperRollerSpeed = 0.5 * maxRollerSpeed;
    lowerRollerSpeed = -0.4 * maxRollerSpeed;
  }

  if (!myData.rl) {
    Q2.setPWM(4, 2600, 0x02);
  } else if (!myData.rr) {
    Q2.setPWM(4, 3500, 0x01);
  } else {
    Q2.setPWM(4, 0, 0);
  }
}

void calculateSpeed(struct_message myData) {


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



  M1 = (int16_t)m1x * xComponent + m1y * yComponent - m1z * zComponent;
  M2 = (int16_t)-m2x * xComponent + m2y * yComponent - m2z * zComponent;
  M3 = (int16_t)-m3x * xComponent - m3y * yComponent - m3z * zComponent;
  M4 = (int16_t)m4x * xComponent - m4y * yComponent - m4z * zComponent;

  maxComponent = abs(M1);
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
  if (myData.id == 0) {
    calculatedSpeed[0] = (float)speedCoefficientJame * maxSpeed * M1 / maxComponent;
    calculatedSpeed[1] = (float)speedCoefficientJame * maxSpeed * M2 / maxComponent;
    calculatedSpeed[2] = (float)speedCoefficientJame * maxSpeed * M3 / maxComponent;
    calculatedSpeed[3] = (float)speedCoefficientJame * maxSpeed * M4 / maxComponent;
    calculatedPWM[0] = manualSpeedCoefficientJame * maxPWM * M1 / maxComponent;
    calculatedPWM[1] = manualSpeedCoefficientJame * maxPWM * M2 / maxComponent;
    calculatedPWM[2] = manualSpeedCoefficientJame * maxPWM * M3 / maxComponent;
    calculatedPWM[3] = manualSpeedCoefficientJame * maxPWM * M4 / maxComponent;
  } else if (myData.id == 1) {
    speedCoefficientPalm = 0.25;
    manualSpeedCoefficientPalm = 0.8;
    calculatedSpeed[0] = (float)speedCoefficientPalm * maxSpeed * M1 / maxComponent;
    calculatedSpeed[1] = (float)speedCoefficientPalm * maxSpeed * M2 / maxComponent;
    calculatedSpeed[2] = (float)speedCoefficientPalm * maxSpeed * M3 / maxComponent;
    calculatedSpeed[3] = (float)speedCoefficientPalm * maxSpeed * M4 / maxComponent;
    calculatedPWM[0] = manualSpeedCoefficientPalm * maxPWM * M1 / maxComponent;
    calculatedPWM[1] = manualSpeedCoefficientPalm * maxPWM * M2 / maxComponent;
    calculatedPWM[2] = manualSpeedCoefficientPalm * maxPWM * M3 / maxComponent;
    calculatedPWM[3] = manualSpeedCoefficientPalm * maxPWM * M4 / maxComponent;
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&bufferData, incomingData, sizeof(bufferData));
  if (bufferData.id == 1) {
    timeOutCounterPalm = 0;
    memcpy(&aimmingData, &bufferData, sizeof(bufferData));
    if (robotPalmStarted) {
      prevPalmEncL = aimmingData.encl;
      prevPalmEncR = aimmingData.encr;
      robotPalmStarted = 0;
    }
    //Serial.println(aimmingData.w);

    if (aimmingData.started == 1) {
      prevPalmEncL = aimmingData.encl;
      prevPalmEncR = aimmingData.encr;
      ackCounterPalm = 10;
      returnPacketPalm.acknowledge = 1;
      result = esp_now_send(returnAddressPalm, (uint8_t *)&returnPacketPalm, sizeof(returnPacketPalm));
      delayMicroseconds(500);
    }
  } else if (bufferData.id == 0) {
    timeOutCounterJame = 0;
    memcpy(&kineticData, &bufferData, sizeof(bufferData));
    if (robotJameStarted) {
      prevJameEncL = kineticData.encl;
      prevJameEncR = kineticData.encr;
      robotJameStarted = 0;
    }

    if (kineticData.started == 1) {
      prevJameEncL = kineticData.encl;
      prevJameEncR = kineticData.encr;
      ackCounterJame = 10;
      returnPacketJame.acknowledge = 1;
      result = esp_now_send(returnAddressJame, (uint8_t *)&returnPacketJame, sizeof(returnPacketJame));
      delayMicroseconds(500);
    }
  }

  if (kineticData.com4 == 1) {
    geegeeMode = 0;
  } else {
    geegeeMode = 1;
  }

  if (kineticData.com5 == 1) {
    master = 0;

  } else {

    master = 1;
  }
  if (kineticData.encr - prevJameEncR != 0) {
    speedCoefficientJame += (kineticData.encr - prevJameEncR) * 0.05;
    manualSpeedCoefficientJame += (kineticData.encr - prevJameEncR) * 0.05;
    prevJameEncR = kineticData.encr;
    if (speedCoefficientJame > maxSpeedCoefficient) {
      speedCoefficientJame = maxSpeedCoefficient;
    } else if (speedCoefficientJame < minSpeedCoefficient) {
      speedCoefficientJame = minSpeedCoefficient;
    }
    if (manualSpeedCoefficientJame > maxPWMCoefficient) {
      manualSpeedCoefficientJame = maxPWMCoefficient;
    } else if (manualSpeedCoefficientJame < minPWMCoefficient) {
      manualSpeedCoefficientJame = minPWMCoefficient;
    }
  }

  if (aimmingData.encl - prevPalmEncL != 0) {
    speedCoefficientPalm += (aimmingData.encl - prevPalmEncL) * 0.05;
    manualSpeedCoefficientPalm += (aimmingData.encl - prevPalmEncL) * 0.05;
    prevPalmEncL = aimmingData.encl;
    if (speedCoefficientPalm > maxSpeedCoefficient) {
      speedCoefficientPalm = maxSpeedCoefficient;
    } else if (speedCoefficientPalm < minSpeedCoefficient) {
      speedCoefficientPalm = minSpeedCoefficient;
    }
    if (manualSpeedCoefficientPalm > maxPWMCoefficient){
      manualSpeedCoefficientPalm = maxPWMCoefficient;
    }
    else if (manualSpeedCoefficientPalm < minPWMCoefficient){
      manualSpeedCoefficientPalm = minPWMCoefficient;
    }
  }

  if (aimmingData.encr != prevPalmEncR) {
    elevationCounter += (aimmingData.encr - prevPalmEncR);
    prevPalmEncR = aimmingData.encr;
  }

  if (master == 0) {
    calculateSpeed(kineticData);

  } else if (master == 1) {

    calculateSpeed(aimmingData);

  } else {
    calculateSpeed(defaultMessage);
  }
  updateSpeed();
  calculateRollerSpeed(aimmingData);
  calculateChassisFeederSpeed(kineticData);
  
  calculateElevationSpeed(aimmingData);
  calculateLiftSpeed(kineticData);
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
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Successfully Transmitted" : "Failed To Transmitted");
}
void setPIDParameters() {
  for (uint8_t i = 0; i < 4; i++) {
    Q1.setSpeedPID(i + 1, 0.9, 10, 0);
  }

  Q2.setSpeedPID(1, 1.8, 2, 0);
  Q2.setSpeedPID(2, 1.8, 2, 0);

  Q2.setPWM(3,0,0);
  Q2.setPWM(4,0,0);
  Q3.setPWM(1,0,0);
  Q3.setPWM(2,0,0);
  Q3.setPWM(3,0,0);
  Q3.setPWM(4,0,0);
}

void setup() {
  setPIDParameters();
  E12BusSetup(16000000);
  for (uint16_t i = 0; i < 256; i++) {
    packet[i] = i + 1;
  }

  Serial.begin(1000000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    return;
  }
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  memcpy(peerInfo.peer_addr, returnAddressPalm, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  memcpy(peerInfo.peer_addr, returnAddressJame, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);

  //timer1 = timerBegin(1, 80, true);

  //timerAttachInterrupt(timer1, &onTimer, true);
  //timerAlarmWrite(timer1, 2000, true);
  //timerAlarmEnable(timer1);

  esp_now_register_recv_cb(OnDataRecv);

  WiFi.setTxPower(WIFI_POWER_19_5dBm);
}

void setSafeStateJame() {
  for (uint8_t i = 0; i < 4; i++) {
    calculatedSpeed[i] = 0;
    calculatedPWM[i] = 0;
  }
  updateSpeed();

  for (uint8_t i = 0; i < 4; i++) {
    Q3.setPWM(i + 1, 4200, 0);
  }
}

void setSafeStatePalm() {
  upperRollerSpeed = 0;
  lowerRollerSpeed = 0;
  Q2.setSpeed(1, upperRollerSpeed);
  Q2.setSpeed(2, lowerRollerSpeed);
  Q2.setPWM(3, 0, 0);
  Q2.setPWM(4, 0, 0);
  if (master == 1) {
    for (uint8_t i = 0; i < 4; i++) {
      calculatedSpeed[i] = 0;
      calculatedPWM[i] = 0;
    }
    updateSpeed();
  }
}

void returnPacketProcessPalm() {

  returnPacketPalm.prevCycleMillis = aimmingData.cycleMillis;
  if (ackCounterPalm > 0) {
    returnPacketPalm.acknowledge = 1;
    result = esp_now_send(returnAddressPalm, (uint8_t *)&returnPacketPalm, sizeof(returnPacketPalm));
    delayMicroseconds(1000);
    ackCounterPalm--;
  } else {
    returnPacketPalm.acknowledge = 0;
    result = esp_now_send(returnAddressPalm, (uint8_t *)&returnPacketPalm, sizeof(returnPacketPalm));
    delayMicroseconds(1000);
  }
}


void returnPacketProcessJame() {

  returnPacketJame.prevCycleMillis = kineticData.cycleMillis;
  if (ackCounterJame > 0) {
    returnPacketJame.acknowledge = 1;
    result = esp_now_send(returnAddressJame, (uint8_t *)&returnPacketJame, sizeof(returnPacketJame));
    delayMicroseconds(1000);
    ackCounterJame--;
  } else {
    returnPacketJame.acknowledge = 0;
    result = esp_now_send(returnAddressJame, (uint8_t *)&returnPacketJame, sizeof(returnPacketJame));
    delayMicroseconds(1000);
  }
}


void loop() {
  calculateGripperSpeed(kineticData);
  timeOutCounterPalm++;
  timeOutCounterJame++;
  if (timeOutCounterPalm > communicationTimeOut) {
    setSafeStatePalm();
    indConfig &= ~(1 << 5);
    peripheralWrite(0, 0xFF, 0xFF, indConfig);
  }
  else{
    returnPacketProcessPalm();
  }

  if (timeOutCounterJame > communicationTimeOut) {
    setSafeStateJame();
    indConfig &= ~(1 << 5);
    peripheralWrite(0, 0xFF, 0xFF, indConfig);
  }
  else{
    returnPacketProcessJame();
  }

  if (timeOutCounterPalm <= communicationTimeOut && timeOutCounterJame <= communicationTimeOut) {
    indConfig |= (1 << 5);
    peripheralWrite(0, 0xFF, 0xFF, indConfig);
  }




  if (cmdCounter >= 10) {
    for (uint8_t j = 0; j < 4; j++) {
      Q1.setCMD(j + 1, motorParameterCMD);
    }
    Q1.getPacket(TXPacket);

    for (uint8_t j = 0; j < 4; j++) {
      Q2.setCMD(j + 1, motorParameterCMD);
    }
    Q2.getPacket(TXPacket2);

    for (uint8_t j = 0; j < 4; j++) {
      Q3.setCMD(j + 1, motorParameterCMD);
    }
    Q3.getPacket(TXPacket3);

    sendData();
    cmdCounter = 0;
  } else {

    if (geegeeMode) {
      for (uint8_t j = 0; j < 4; j++) {
        Q1.setCMD(j + 1, directControlCMD);
      }
    } else {
      for (uint8_t j = 0; j < 4; j++) {
        Q1.setCMD(j + 1, speedControlCMD);
      }
    }
    updateSpeed();

    Q2.setSpeed(1, upperRollerSpeed);
    Q2.setSpeed(2, lowerRollerSpeed);
    Q2.setCMD(1, speedControlCMD);
    Q2.setCMD(2, speedControlCMD);
    Q2.setCMD(3, directControlCMD);
    Q2.setCMD(4, directControlCMD);

    Q3.setCMD(1, directControlCMD);
    Q3.setCMD(2, directControlCMD);
    Q3.setCMD(3, directControlCMD);
    Q3.setCMD(4, directControlCMD);

    Q1.getPacket(TXPacket);
    Q2.getPacket(TXPacket2);
    Q3.getPacket(TXPacket3);
    sendData();
    cmdCounter++;
  }
  indConfig &= ~(1 << 0);
  peripheralWrite(0, 0xFF, 0xFC, indConfig);
  delay(1);
  indConfig |= (1 << 0);
  peripheralWrite(0, 0xFF, 0xFF, indConfig);
}
