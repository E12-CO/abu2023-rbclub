#include "stdint.h"// Include for using "uint8_t" "uint16_t" and so on.
#include "E12_Function.h"// Include Library for E12 drive.
uint8_t broadcastAddress[] = { 0x30, 0xC6, 0xF7, 0x00, 0x4D, 0x84 };
uint8_t returnAddress[] = { 0xC0, 0x49, 0xEF, 0x69, 0x8D, 0x28 };

#define delayTime 500// used to be the delay constant of for loop.

// Speed constant of each axis
// Used in chassis drive motors.
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

#define maxSpeed 4200// Maximum speed (Maximum PWM number).
#define threshold 350
uint64_t timeOutCounter = 0;
volatile uint64_t generalCounter = 0;
int16_t M1 = 0, M2 = 0, M3 = 0;// PWM of Omni-drive motors
uint8_t indConfig = 0b11111111;// Bit control of on board LED indicators.

// RX and TX packet for E12
uint8_t RXPacket[1024];
uint8_t TXPacket[1024];
// RX and TX packet for E12
uint8_t RXPacket2[1024];
uint8_t TXPacket2[1024];

uint32_t prevMillis = 0;
uint8_t commuFault = 0;

uint8_t cmdCounter = 0;

motorParameter_t motorParameterDefault = { 0, 0, 0, 0, 0, 0, 24000, 0, 0, 1024, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4000, 0, 0 };
speedControl_t speedControlDefault = { 0, 0, 0, 0 };
positionControl_t positionControlDefault = { 0, 0, 0, 0, 0 };
motor motorDefault = { motorParameterDefault,
                       speedControlDefault,
                       positionControlDefault,
                       speedControlCMD
                     };
uint8_t displayEnable = 0;
float speedCoefficient = 0.75;
uint8_t speedFlag[4] = { 0, 0, 0, 0 };
float calculatedSpeed[4] = { 0, 0, 0, 0 };
float mSpeed[4] = { 0, 0, 0, 0 };
uint32_t counter5 = 0;

motor MOTOR[4] = { motorDefault, motorDefault, motorDefault, motorDefault };
uint32_t firstMillis = 0;
QuadMotorDriver BoxB(MOTOR);
QuadMotorDriver BoxA(MOTOR);
uint32_t prevMillisElevation = 0;
uint32_t prevMillisLoad = 0;

uint8_t stateRT = 1, stateRD = 1, stateRL = 1, stateRR = 1;
uint8_t stateLT = 1, stateLD = 1, stateLL = 1, stateLR = 1;

uint8_t stateCom1 = 1, stateCom2 = 1, stateCom3 = 1, stateCom4 = 1, stateCom5 = 1;
uint8_t laststateRT = 1, laststateRD = 1, laststateRL = 1, laststateRR = 1;
uint8_t laststateLT = 1, laststateLD = 1, laststateLL = 1, laststateLR = 1;

byte incomingByte;
double sensorData[6] = { 0, 0, 0, 0, 0, 0 };
int ind1 = LOW;

float upperRollerSpeed = 0;
float lowerRollerSpeed = 0;
volatile uint8_t robotStarted = 1;
uint16_t maxMotorSpeed = 0;

int32_t prevEncL = 0;
int32_t prevEncR = 0;

int32_t elevationCounter = 0;
int32_t prevElevationCounter = 0;
#define elevationInterval 100
uint32_t prevElevationMillis = 0;
uint8_t elevationDir = 0;
uint8_t elevationRunning = 0;


// Electronics Topologies
// ///////////////////////////
// Box A (BoxA) - Right hand
// Channel 1 -> Left Roller
// Channel 2 -> Right Roller
// Channel 3 -> Linear Motor
// Channel 4 -> Lead screw
#define BOX_A_RightRoller 1
#define BOX_A_LeftRoller  2
#define BOX_A_LinearMotor 3
#define BOX_A_LeadScrew   4
// ///////////////////////////
// Box B (BoxB) - Left hand
// Channel 1 -> Front Omni Motor
// Channel 2 -> Left Omni Motor
// Channel 3 -> Right Omni Motor
// Channel 4 -> Ring Loading Platform motor
#define BOX_B_Fomni       1
#define BOX_B_Lomni       2
#define BOX_B_Romni       3
#define BOX_B_LoadPlat    4
// ///////////////////////////


// Message struct for Joy Controller <-> Master board
typedef struct {
  uint32_t started;
  int32_t id;
  int32_t encl, encr;
  int32_t x, y, z, w;
  uint8_t rt, rd, rl, rr, lt, ld, ll, lr;
  uint8_t com1, com2, com3, com4, com5;
} struct_message;

typedef struct {
  uint32_t acknowledge;
} returnStruct_t;
returnStruct_t returnPacket;

// Message struct for sub-board.
typedef struct sub_message {
  uint8_t feedMotor;
  uint8_t lockMotor;
} sub_message;

sub_message subData;

struct_message myData;

hw_timer_t *timer1 = NULL;

float RPM[4] = { 0, 0, 0, 0 };
int16_t xComponent = 0, yComponent = 0, zComponent = 0, wComponent = 0;

// FIXME : Make it work
void calculateLinearMotorSpeed() {
  if ((elevationCounter > prevElevationCounter) && !elevationRunning) {
    prevElevationMillis = millis();
    elevationDir = 1;
    BoxA.setPWM(BOX_A_LinearMotor, 4200, elevationDir);// Set motor 3 of Box A to max PWM (4200) with direction of spin up
    elevationRunning = 1;
  } else if ((elevationCounter < prevElevationCounter) && !elevationRunning) {
    prevElevationMillis = millis();
    elevationDir = 2;
    BoxA.setPWM(BOX_A_LinearMotor, 4200, elevationDir);// Set motor 3 of Box A to max PWM (4200) with direction of spin down
    elevationRunning = 1;
  }

  if (((millis() - prevElevationMillis) > elevationInterval) && elevationRunning) {

    if (elevationDir == 1) {
      prevElevationCounter++;
    } else if (elevationDir == 2) {
      prevElevationCounter--;
    } else {
      prevElevationCounter = elevationCounter;
    }

    if (prevElevationCounter == elevationCounter) {
      BoxA.setPWM(BOX_A_LinearMotor, 4200, 0);// Stop motor
    }
    elevationRunning = 0;
  }
}

// TODO : Compatibility with new CRU sub-board firmware
// INFO : Might be removed as the CRU sub-board will be controlled directly from the Joy controller
void calculateSubSystem() {
  if (!stateLT) {
    subData.lockMotor = 1;
  } else if (!stateLD) {
    subData.lockMotor = 2;
  } else {
    subData.lockMotor = 0;
  }

  if (upperRollerSpeed != 0) {
    subData.feedMotor = 1;
  } else {
    subData.feedMotor = 0;
  }
  esp_now_send(broadcastAddress, (uint8_t *)&subData, sizeof(sub_message));
}

// TODO : Real test to find the right motor speed.
void calculateRollerSpeed() {
  if (!stateRD) {// Right hand - Bottom
    upperRollerSpeed = 0;
    lowerRollerSpeed = 0;
  } else if (!stateRR) {// Right hand - Right
    upperRollerSpeed = -2000;
    lowerRollerSpeed = -2000;
  } else if (!stateRT) {// Right hand - Top
    upperRollerSpeed = -4000;
    lowerRollerSpeed = -4000;
  } else if (!stateRL) {// Right hand - Left
    upperRollerSpeed = -5200;
    lowerRollerSpeed = -5200;
  }

  //rollerSpeed

    if (myData.encl != prevEncL) {
      upperRollerSpeed += (myData.encl - prevEncL) * 100;
      lowerRollerSpeed += (myData.encl - prevEncL) * 100;
      prevEncL = myData.encl;
    }

  if (upperRollerSpeed < -5200) {
    upperRollerSpeed = -5200;
  } else if (upperRollerSpeed > -1000) {
    upperRollerSpeed = 0;
  }

  if (lowerRollerSpeed < -5200) {
    lowerRollerSpeed = -5200;
  } else if (lowerRollerSpeed > -1000) {
    lowerRollerSpeed = 0;
  }

    if (myData.encr != prevEncR) {
      elevationCounter += (myData.encr - prevEncR);
      prevEncR = myData.encr;
    }

  // Ring push (into the roller)
  // TODO : This will be moved to Sub board.
  //  if (!stateLL) {
  //    BoxA.setPWM(4, 3200, 0x01);
  //  } else if (!stateLR) {
  //    BoxA.setPWM(4, 3200, 0x02);
  //  } else {
  //    BoxA.setPWM(4, 4200, 0);
  //  }
}
// FIXME : For some reason, It didn't work - Test require
void calculateLiftSpeed() {// Calculate lead screw
  // 2100 is 24Volt/12Volt * 4200. A PWM value for 12V.

  if ((!stateCom1) && (stateCom2)) { // com 1 is pressed
    BoxA.setPWM(BOX_A_LeadScrew, 2100, 0x01);
  } else if ((stateCom1) && (!stateCom2)) { // com 2 is pressed
    BoxA.setPWM(BOX_A_LeadScrew, 2100, 0x02);
  } else {
    BoxA.setPWM(BOX_A_LeadScrew, 4200, 0x00);// Stop motor using regen-braking
  }

  // stateCom4 is left toggle switch. Reserved to do something else

}

// Calculate speed for Omni drive motors
// Status Done UNTESTED
void calculateSpeed() {

  if (myData.w > 2047 + threshold || myData.w < 2047 - threshold) {
    yComponent = map(myData.w, 0, 4095, -2048, 2047);
  } else {
    yComponent = 0;
  }


  if (myData.z > 2047 + threshold || myData.z < 2047 - threshold) {
    xComponent = map(myData.z, 0, 4095, 2047, -2048);
  } else {
    xComponent = 0;
  }

  if (myData.x > 2047 + threshold || myData.x < 2047 - threshold) {
    zComponent = map(myData.x, 0, 4095, -2048, 2047);
  } else {
    zComponent = 0;
  }

  if (myData.y > 2047 + threshold || myData.y < 2047 - threshold) {
    wComponent = map(myData.y, 0, 4095, -2048, 2047);
  } else {
    wComponent = 0;
  }

  // stateCom3 is right toggle switch
  // Select speed
  // Normal and Ramp climbing speed
  if (!stateCom3) {
    // High speed mode 75%
    speedCoefficient = 0.75;
  } else {
    // normal speed mode 60%
    speedCoefficient = 0.60;
  }
  //Serial.print("Speed Coeff %:");
  //Serial.println(speedCoefficient);

  M1 = (int16_t)(m1x * xComponent - m1z * zComponent * 0.7) / 2048.00 * 4200.00; // Front Omni motor
  M2 = (int16_t)(-m2y * yComponent * 0.88603 + m2x * xComponent * 0.88603 + m2z * zComponent * 0.7) / 2048.00 * 4200.00; // Left Omni motor
  M3 = (int16_t)(m3y * yComponent * 0.88603 + m3x * xComponent * 0.88603 + m3z * zComponent * 0.7) / 2048.00 * 4200.00; // Right Omni motor

  // After calculation. Choose the max value as Maximum speed.
  maxMotorSpeed = abs(M1);
  maxMotorSpeed = max(abs(M1), abs(M2));
  maxMotorSpeed = max(abs(M2), abs(M3));

  // In case of max speed is more than PWM Limit
  // Cap it at 4200
  if (maxMotorSpeed > 4200) {
    maxMotorSpeed = 4200;
  }

  M1 = (float)M1 / maxMotorSpeed * 4200.00 * speedCoefficient;
  M2 = (float)M2 / maxMotorSpeed * 4200.00 * speedCoefficient;
  M3 = (float)M3 / maxMotorSpeed * 4200.00 * speedCoefficient;

  // Front Omni
  if (M1 > 4200) {
    BoxB.setPWM(BOX_B_Fomni, 4200, 0x01);
  } else if (M1 > 0) {
    BoxB.setPWM(BOX_B_Fomni, M1, 0x01);
  } else if (M1 == 0) {
    BoxB.setPWM(BOX_B_Fomni, 3600, 0);
  } else if (M1 > -4200) {
    BoxB.setPWM(BOX_B_Fomni, -M1, 0x02);
  } else {
    BoxB.setPWM(BOX_B_Fomni, 4200, 0x02);
  }

  // Left Omni
  if (M2 > 4200) {
    BoxB.setPWM(BOX_B_Lomni, 4200, 0x02);
  } else if (M2 > 0) {
    BoxB.setPWM(BOX_B_Lomni, M2, 0x02);
  } else if (M2 == 0) {
    BoxB.setPWM(BOX_B_Lomni, 3600, 0);
  } else if (M2 > -4200) {
    BoxB.setPWM(BOX_B_Lomni, -M2, 0x01);
  } else {
    BoxB.setPWM(BOX_B_Lomni, 4200, 0x01);
  }

  // Right Omni
  if (M3 > 4200) {
    BoxB.setPWM(BOX_B_Romni, 4200, 0x02);
  } else if (M3 > 0) {
    BoxB.setPWM(BOX_B_Romni, M3, 0x02);
  } else if (M3 == 0) {
    BoxB.setPWM(BOX_B_Romni, 3600, 0);
  } else if (M3 > -4200) {
    BoxB.setPWM(BOX_B_Romni, -M3, 0x01);
  } else {
    BoxB.setPWM(BOX_B_Romni, 4200, 0x01);
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  firstMillis = millis();
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
  stateCom1 = myData.com1;
  stateCom2 = myData.com2;
  stateCom3 = myData.com3;
  stateCom4 = myData.com4;
  stateCom5 = myData.com5;
  if (robotStarted) {
    prevEncL = myData.encl;
    prevEncR = myData.encr;
    robotStarted = 0;
  }
  if (myData.started) {
    returnPacket.acknowledge = 1;
    esp_now_send(returnAddress, (uint8_t *)&returnPacket, sizeof(returnPacket));
    prevEncL = 0;
    prevEncR = 0;
    return;
  }
  calculateSpeed();
  calculateRollerSpeed();

  calculateLinearMotorSpeed();
  calculateLiftSpeed();

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


// Set the Kp Ki and Kd (PID gain) for each Roller motor.
void setPIDParameters() {
  BoxA.setSpeedPID(1, 1.8, 2, 0);
  BoxA.setSpeedPID(2, 1.8, 2, 0);
}

void setup() {
  setPIDParameters();
  E12BusSetup(16000000);
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  if (esp_now_init() != ESP_OK) {
    return;
  }

  //esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  memcpy(peerInfo.peer_addr, returnAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  for (uint8_t i = 0; i < 4; i++) {
    BoxB.setPWM(i + 1, 0, 0);
  }
  upperRollerSpeed = 0;
  lowerRollerSpeed = 0;
  BoxA.setSpeed(BOX_A_LeftRoller, upperRollerSpeed);
  BoxA.setSpeed(BOX_A_RightRoller, lowerRollerSpeed);
  BoxA.setPWM(BOX_A_LinearMotor, 0, 0);
  BoxA.setPWM(BOX_A_LeadScrew, 0, 0);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}

void loop() {
  calculateSubSystem();

  timeOutCounter++;

  if (timeOutCounter > 200) {
    // In case of Joy controller timed out.
    // Stop all motor

    // This for loop stops all motor on E12 Box B
    for (uint8_t i = 0; i < 4; i++) {
      BoxB.setPWM(i + 1, 0, 0);
    }
    upperRollerSpeed = 0;
    lowerRollerSpeed = 0;
    BoxA.setSpeed(BOX_A_LeftRoller, upperRollerSpeed);
    BoxA.setSpeed(BOX_A_RightRoller, lowerRollerSpeed);
    BoxA.setPWM(BOX_A_LinearMotor, 0, 0);
    BoxA.setPWM(BOX_A_LeadScrew, 0, 0);
    indConfig &= ~(1 << 5);
    peripheralWrite(0, 0xFF, 0xFF, indConfig);
  } else {
    indConfig |= (1 << 5);
    peripheralWrite(0, 0xFF, 0xFF, indConfig);
  }
  //serialReceive();

  if (cmdCounter > 0) {
    for (uint8_t j = 0; j < 4; j++) {
      BoxB.setCMD(j + 1, directControlCMD);
    }

    BoxA.setSpeed(BOX_A_LeftRoller, upperRollerSpeed);
    BoxA.setSpeed(BOX_A_RightRoller, lowerRollerSpeed);
    BoxA.setCMD(BOX_A_LeftRoller, speedControlCMD);
    BoxA.setCMD(BOX_A_RightRoller, speedControlCMD);
    BoxA.setCMD(BOX_A_LinearMotor, directControlCMD);
    BoxA.setCMD(BOX_A_LeadScrew, directControlCMD);

    BoxB.getPacket(TXPacket);// SPI channel 2
    BoxA.getPacket(TXPacket2);// SPI Channel 3
    sendData();
    cmdCounter--;
  } else {
    for (uint8_t j = 0; j < 4; j++) {
      BoxB.setCMD(j + 1, motorParameterCMD);
    }
    BoxB.getPacket(TXPacket);

    for (uint8_t j = 0; j < 4; j++) {
      BoxA.setCMD(j + 1, motorParameterCMD);
    }
    BoxA.getPacket(TXPacket2);

    sendData();
    cmdCounter = 10;
  }



  indConfig &= ~(1 << 0);
  peripheralWrite(0, 0xFF, 0xFC, indConfig);
  delay(1);
  indConfig |= (1 << 0);
  peripheralWrite(0, 0xFF, 0xFF, indConfig);
}
