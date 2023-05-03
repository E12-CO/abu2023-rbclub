//
#include <ESP32Encoder.h>
#include <WiFi.h>
#include "esp_now.h"
uint8_t broadcastAddress[] = {0x0C, 0xB8, 0x15, 0xC2, 0x42, 0x60};

ESP32Encoder encoderL;
ESP32Encoder encoderR;

#define rswtop 3
#define rswdown 19
#define rswright 21
#define rswleft 18

#define lswtop 23
#define lswdown 13  
#define lswright 5
#define lswleft 22

#define enleftA 32
#define enleftB 33

#define enrightA 16
#define enrightB 17

#define rswst 14
#define lswst 12

#define ind1 4
#define ind2 2
#define ind3 15

#define updleft 35
#define lrleft 34
#define updright 36
#define lrright 39

#define _com1 12
#define _com2 14
#define _com3 27
#define _com4 26
#define _com5 25

int32_t enl, enr, countChR, compR, countChL, compL;
int32_t stateEncodeR = 0, stateEncodeL = 0;
int statusEnR = 0, statusEnL = 0;

int buttonStateRT = HIGH, buttonStateRD = HIGH, buttonStateRL = HIGH, buttonStateRR = HIGH, buttonStateCom1 = HIGH,  buttonStateCom2 = HIGH,  buttonStateCom3 = HIGH,  buttonStateCom4 = HIGH,  buttonStateCom5 = HIGH;
int buttonStateLT = HIGH, buttonStateLD = HIGH, buttonStateLL = HIGH, buttonStateLR = HIGH;
int lastButtonStateRT = HIGH, lastButtonStateRD = HIGH, lastButtonStateRL = HIGH, lastButtonStateRR = HIGH, lastButtonStateCom1 = HIGH, lastButtonStateCom2 = HIGH, lastButtonStateCom3 = HIGH, lastButtonStateCom4 = HIGH, lastButtonStateCom5 = HIGH;
int lastButtonStateLT = HIGH, lastButtonStateLD = HIGH, lastButtonStateLL = HIGH, lastButtonStateLR = HIGH;

int rswtopState = 1, rswdownState = 1, rswleftState = 1, rswrightState = 1, com1State = 1, com2State = 1, com3State = 1, com4State = 1, com5State = 1;
int lswtopState = 1, lswdownState = 1, lswleftState = 1, lswrightState = 1;

unsigned long lastDebounceTimeRT = 0, lastDebounceTimeRD = 0, lastDebounceTimeRL = 0, lastDebounceTimeRR = 0, lastDebounceTimeCom1 = 0, lastDebounceTimeCom2 = 0, lastDebounceTimeCom3 = 0, lastDebounceTimeCom4 = 0, lastDebounceTimeCom5 = 0;
unsigned long lastDebounceTimeLT = 0, lastDebounceTimeLD = 0, lastDebounceTimeLL = 0, lastDebounceTimeLR = 0;

typedef struct {
  uint32_t started;
  uint32_t cycleMillis;
  int32_t id;
  int32_t encl, encr;
  int32_t x, y, z, w;
  uint8_t rt, rd, rl, rr, lt, ld, ll, lr;
  uint8_t com1, com2, com3, com4, com5;
} packet_t;

packet_t packet = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0 };

typedef struct {
  uint32_t acknowledge;
  float speedCoefficient;
  uint32_t prevCycleMillis;
} returnStruct_t;
returnStruct_t myData;

void sentExecuted(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Successfully Transmitted" : "Failed To Transmitted");
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  if (myData.acknowledge == 1) {
    packet.started = 0;
  }
  //Serial.println("A");
  //Serial.println(millis()-myData.prevCycleMillis);
  //Serial.println(packet.started);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(rswst, INPUT_PULLUP);
  pinMode(lswst, INPUT_PULLUP);

  pinMode(rswtop, INPUT_PULLUP);
  pinMode(rswdown, INPUT_PULLUP);
  pinMode(rswright, INPUT_PULLUP);
  pinMode(rswleft, INPUT_PULLUP);

  pinMode(lswtop, INPUT_PULLUP);
  pinMode(lswdown, INPUT_PULLUP);
  pinMode(lswright, INPUT_PULLUP);
  pinMode(lswleft, INPUT_PULLUP);

  pinMode(enleftA, INPUT);
  pinMode(enleftB, INPUT);
  pinMode(enrightA, INPUT);
  pinMode(enrightB, INPUT);

  pinMode(updleft, INPUT);
  pinMode(lrleft, INPUT);
  pinMode(updright, INPUT);
  pinMode(lrright, INPUT);

  pinMode(ind1, OUTPUT);
  pinMode(ind2, OUTPUT);
  pinMode(ind3, OUTPUT);
  digitalWrite(ind1, 0);
  digitalWrite(ind2, 0);
  digitalWrite(ind3, 0);


  pinMode(_com1, INPUT_PULLUP);
  pinMode(_com2, INPUT_PULLUP);
  pinMode(_com3, INPUT_PULLUP);
  pinMode(_com4, INPUT_PULLUP);
  pinMode(_com5, INPUT_PULLUP);

  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoderL.attachHalfQuad(enleftA, enleftB);
  encoderR.attachHalfQuad(enrightA, enrightB);

  //setstartEncode
  encoderL.setCount(0);
  encoderR.setCount(0);
  stateEncodeR = 0;
  stateEncodeL = 0;
  //encoder2.clearCount();
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  while (esp_now_init() != ESP_OK) {
    digitalWrite(ind1, HIGH);
    delay(100);
    digitalWrite(ind1, LOW);
    delay(100);
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(sentExecuted);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  Serial.begin(1000000);
  packet.started = 1;
}

void loop() {
  // put your main code here, to run repeatedly:
  EncodeRemote();
  DebounceSW();
  packet.id = 0;  //Jame
  
  packet.rt = rswtopState;
  packet.rd = rswdownState;
  packet.rl = rswleftState;
  packet.rr = rswrightState;
  packet.lt = lswtopState;
  packet.ld = lswdownState;
  packet.ll = lswleftState;
  packet.lr = lswrightState;
  packet.com1 = com1State;
  packet.com2 = com2State;
  packet.com3 = com3State;
  packet.com4 = com4State;
  packet.com5 = com5State;
  packet.encl = enl;
  packet.encr = enr;
  packet.x = analogRead(updleft);
  packet.y = analogRead(lrleft);
  packet.z = analogRead(updright);
  packet.w = analogRead(lrright);
  packet.cycleMillis = millis();
/*
  Serial.println(packet.x);
  Serial.print(" ");
  Serial.print(packet.y);
  Serial.print(" ");
  Serial.print(packet.z);
  Serial.print(" ");
  Serial.println(packet.w);
  Serial.println(packet.started);*/
  if (esp_now_send(broadcastAddress, (uint8_t *)&packet, sizeof(packet)) == ESP_OK) {
    digitalWrite(ind2, HIGH);
    delay(1);
    digitalWrite(ind2, LOW);
  } else {
    //digitalWrite(2,LOW);
  }
  delay(10);
 // Serial.println(millis());

  /*
Serial.print(rswtopState);
Serial.print(" "); 
Serial.print (rswdownState);
Serial.print (" ");
Serial.print (rswrightState);
Serial.print(" ");
Serial.print (rswleftState);
Serial.print(" ");
Serial.print (lswtopState);
Serial.print(" ");
Serial.print (lswdownState);
Serial.print(" ");
Serial.print (lswrightState);
Serial.print(" ");
Serial.print (lswleftState);
Serial.print(" ");
Serial.print (String(enl));
Serial.print(" ");
Serial.print (String(enr));
Serial.print(" ");
*/
  /*
Serial.print (digitalRead(lswst));
Serial.print(" ");
Serial.print (digitalRead(rswst));
Serial.print(" "); 

Serial.print (analogRead(updleft));
Serial.print(" ");
Serial.print (analogRead(lrleft));
Serial.print(" ");
Serial.print (analogRead(updright));
Serial.print(" ");
Serial.print (analogRead(lrright));
Serial.println(" ");
*/
}

void EncodeRemote() {


  enr = ((int32_t)encoderR.getCount() / 2);

  enl = ((int32_t)encoderL.getCount() / 2);
}


void DebounceSW() {
  // rtop
  if (digitalRead(rswtop) != lastButtonStateRT) {
    lastDebounceTimeRT = millis();
  }

  if ((millis() - lastDebounceTimeRT) > 50) {

    if (digitalRead(rswtop) != buttonStateRT) {
      buttonStateRT = digitalRead(rswtop);
      if (buttonStateRT == LOW) {
        rswtopState = 0;
      } else {
        rswtopState = 1;
      }
    }
  }

  lastButtonStateRT = digitalRead(rswtop);

  //rdown
  if (digitalRead(rswdown) != lastButtonStateRD) {
    lastDebounceTimeRD = millis();
  }

  if ((millis() - lastDebounceTimeRD) > 50) {

    if (digitalRead(rswdown) != buttonStateRD) {
      buttonStateRD = digitalRead(rswdown);
      if (buttonStateRD == LOW) {
        rswdownState = 0;
      } else {
        rswdownState = 1;
      }
    }
  }

  lastButtonStateRD = digitalRead(rswdown);

  //rleft
  if (digitalRead(rswleft) != lastButtonStateRL) {
    lastDebounceTimeRL = millis();
  }

  if ((millis() - lastDebounceTimeRD) > 50) {

    if (digitalRead(rswleft) != buttonStateRL) {
      buttonStateRL = digitalRead(rswleft);
      if (buttonStateRL == LOW) {
        rswleftState = 0;
      } else {
        rswleftState = 1;
      }
    }
  }

  lastButtonStateRL = digitalRead(rswleft);

  //rright
  if (digitalRead(rswright) != lastButtonStateRR) {
    lastDebounceTimeRR = millis();
  }

  if ((millis() - lastDebounceTimeRR) > 50) {

    if (digitalRead(rswright) != buttonStateRR) {
      buttonStateRR = digitalRead(rswright);
      if (buttonStateRR == LOW) {
        rswrightState = 0;
      } else {
        rswrightState = 1;
      }
    }
  }

  lastButtonStateRR = digitalRead(rswright);


  // ltop
  if (digitalRead(lswtop) != lastButtonStateLT) {
    lastDebounceTimeLT = millis();
  }

  if ((millis() - lastDebounceTimeLT) > 50) {

    if (digitalRead(lswtop) != buttonStateLT) {
      buttonStateLT = digitalRead(lswtop);
      if (buttonStateLT == LOW) {
        lswtopState = 0;
      } else {
        lswtopState = 1;
      }
    }
  }

  lastButtonStateLT = digitalRead(lswtop);

  //ldown
  if (digitalRead(lswdown) != lastButtonStateLD) {
    lastDebounceTimeLD = millis();
  }

  if ((millis() - lastDebounceTimeLD) > 50) {

    if (digitalRead(lswdown) != buttonStateLD) {
      buttonStateLD = digitalRead(lswdown);
      if (buttonStateLD == LOW) {
        lswdownState = 0;
      } else {
        lswdownState = 1;
      }
    }
  }

  lastButtonStateLD = digitalRead(lswdown);

  //lleft
  if (digitalRead(lswleft) != lastButtonStateLL) {
    lastDebounceTimeLL = millis();
  }

  if ((millis() - lastDebounceTimeLL) > 50) {

    if (digitalRead(lswleft) != buttonStateLL) {
      buttonStateLL = digitalRead(lswleft);
      if (buttonStateLL == LOW) {
        lswleftState = 0;
      } else {
        lswleftState = 1;
      }
    }
  }

  lastButtonStateLL = digitalRead(lswleft);

  //lright
  if (digitalRead(lswright) != lastButtonStateLR) {
    lastDebounceTimeLR = millis();
  }

  if ((millis() - lastDebounceTimeLR) > 50) {

    if (digitalRead(lswright) != buttonStateLR) {
      buttonStateLR = digitalRead(lswright);
      if (buttonStateLR == LOW) {
        lswrightState = 0;
      } else {
        lswrightState = 1;
      }
    }
  }

  lastButtonStateLR = digitalRead(lswright);

  if (digitalRead(lswright) != lastButtonStateLR) {
    lastDebounceTimeLR = millis();
  }

  if ((millis() - lastDebounceTimeLR) > 50) {

    if (digitalRead(lswright) != buttonStateLR) {
      buttonStateLR = digitalRead(lswright);
      if (buttonStateLR == LOW) {
        lswrightState = 0;
      } else {
        lswrightState = 1;
      }
    }
  }

  lastButtonStateLR = digitalRead(lswright);

  //COM

  if (digitalRead(_com1) != lastButtonStateCom1) {
    lastDebounceTimeCom1 = millis();
  }

  if ((millis() - lastDebounceTimeCom1) > 50) {

    if (digitalRead(_com1) != buttonStateCom1) {
      buttonStateCom1 = digitalRead(_com1);
      if (buttonStateCom1 == LOW) {
        com1State = 0;
      } else {
        com1State = 1;
      }
    }
  }

  lastButtonStateCom1 = digitalRead(_com1);

  if (digitalRead(_com2) != lastButtonStateCom2) {
    lastDebounceTimeCom2 = millis();
  }

  if ((millis() - lastDebounceTimeCom2) > 50) {

    if (digitalRead(_com2) != buttonStateCom2) {
      buttonStateCom2 = digitalRead(_com2);
      if (buttonStateCom2 == LOW) {
        com2State = 0;
      } else {
        com2State = 1;
      }
    }
  }

  lastButtonStateCom2 = digitalRead(_com2);

  if (digitalRead(_com3) != lastButtonStateCom3) {
    lastDebounceTimeCom3 = millis();
  }

  if ((millis() - lastDebounceTimeCom3) > 50) {

    if (digitalRead(_com3) != buttonStateCom3) {
      buttonStateCom3 = digitalRead(_com3);
      if (buttonStateCom3 == LOW) {
        com3State = 0;
      } else {
        com3State = 1;
      }
    }
  }

  lastButtonStateCom3 = digitalRead(_com3);

  if (digitalRead(_com4) != lastButtonStateCom4) {
    lastDebounceTimeCom4 = millis();
  }

  if ((millis() - lastDebounceTimeCom4) > 50) {

    if (digitalRead(_com4) != buttonStateCom4) {
      buttonStateCom4 = digitalRead(_com4);
      if (buttonStateCom4 == LOW) {
        com4State = 0;
      } else {
        com4State = 1;
      }
    }
  }

  lastButtonStateCom4 = digitalRead(_com4);

  if (digitalRead(_com5) != lastButtonStateCom5) {
    lastDebounceTimeCom5 = millis();
  }

  if ((millis() - lastDebounceTimeCom5) > 50) {

    if (digitalRead(_com5) != buttonStateCom5) {
      buttonStateCom5 = digitalRead(_com5);
      if (buttonStateCom5 == LOW) {
        com5State = 0;
      } else {
        com5State = 1;
      }
    }
  }

  lastButtonStateCom5 = digitalRead(_com5);

}
