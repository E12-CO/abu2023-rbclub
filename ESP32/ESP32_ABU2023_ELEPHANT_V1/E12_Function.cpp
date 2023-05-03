#include "E12_Function.h"

void E12BusSetup(uint32_t speed) {
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


void writeChipSelect(uint8_t config) {
  GPIO.out_w1tc = (1 << CS_LATCH);
  for (uint8_t i = 0; i < 8; i++) {
    GPIO.out_w1tc = (1 << CS_CLOCK);
    GPIO.out_w1ts = ((config & 0b1 << i) && 1) << CS_DATA;
    GPIO.out_w1tc = (!((config & 0b1 << i) && 1)) << CS_DATA;
    GPIO.out_w1ts = (1 << CS_CLOCK);
  }
  GPIO.out_w1ts = (1 << CS_LATCH);
}

void startChipSelect() {
  GPIO.out_w1tc = (1 << CS_LATCH);
  GPIO.out_w1tc = (1 << CS_CLOCK);
  GPIO.out_w1tc = (1 << CS_DATA);
  GPIO.out_w1ts = (1 << CS_CLOCK);
  GPIO.out_w1ts = (1 << CS_LATCH);
}

void shiftChipSelect() {
  GPIO.out_w1tc = (1 << CS_LATCH);
  GPIO.out_w1tc = (1 << CS_CLOCK);
  GPIO.out_w1ts = (1 << CS_DATA);
  GPIO.out_w1ts = (1 << CS_CLOCK);
  GPIO.out_w1ts = (1 << CS_LATCH);
}

void sendData() {
  writeChipSelect(0b11111110);
  delayMicroseconds(1);
  SPI.transferBytes(TXPacket, RXPacket, packetSize + 1);
  delayMicroseconds(1);
  writeChipSelect(0xFF);
  delay(1);
  writeChipSelect(0b11111101);
  delayMicroseconds(1);
  SPI.transferBytes(TXPacket2, RXPacket2, packetSize + 1);
  delayMicroseconds(1);
  writeChipSelect(0xFF);
  delay(1);
  writeChipSelect(0b11111011);
  delayMicroseconds(1);
  SPI.transferBytes(TXPacket3, RXPacket3, packetSize + 1);
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

void updateSpeed() {
  if (geegeeMode) {
    
    for (uint8_t i = 0; i < 4; i++) {
      if(calculatedPWM[i] > 0){
        Q1.setPWM(i + 1, calculatedPWM[i],0x02);
      }
      else if (calculatedPWM[i] < 0){
        Q1.setPWM(i + 1, -calculatedPWM[i],0x01);
      }
      else{
        Q1.setPWM(i + 1, 3600,0x00);
      }
      
    }
  }
  else{
    for (uint8_t i = 0; i < 4; i++) {
      Q1.setSpeed(i + 1, calculatedSpeed[i]);
    }
  }
}


void serialReceive() {
  while (Serial.available() > 0) {
    String temp = Serial.readString();
    if (temp[1] == 'p') {
      if (temp[3] == '1') {
        Q1.setSpeedP(1, (temp.substring(5)).toFloat());
      } else if (temp[3] == '2') {
        Q1.setSpeedP(2, (temp.substring(5)).toFloat());
      } else if (temp[3] == '3') {
        Q1.setSpeedP(3, (temp.substring(5)).toFloat());
      } else if (temp[3] == '4') {
        Q1.setSpeedP(4, (temp.substring(5)).toFloat());
      } else if (temp[3] == 'a') {
        Q1.setSpeedP(1, (temp.substring(5)).toFloat());
        Q1.setSpeedP(2, (temp.substring(5)).toFloat());
        Q1.setSpeedP(3, (temp.substring(5)).toFloat());
        Q1.setSpeedP(4, (temp.substring(5)).toFloat());
      }
    }
    if (temp[1] == 'i') {
      if (temp[3] == '1') {
        Q1.setSpeedI(1, (temp.substring(5)).toFloat());
      } else if (temp[3] == '2') {
        Q1.setSpeedI(2, (temp.substring(5)).toFloat());
      } else if (temp[3] == '3') {
        Q1.setSpeedI(3, (temp.substring(5)).toFloat());
      } else if (temp[3] == '4') {
        Q1.setSpeedI(4, (temp.substring(5)).toFloat());
      } else if (temp[3] == 'a') {
        Q1.setSpeedI(1, (temp.substring(5)).toFloat());
        Q1.setSpeedI(2, (temp.substring(5)).toFloat());
        Q1.setSpeedI(3, (temp.substring(5)).toFloat());
        Q1.setSpeedI(4, (temp.substring(5)).toFloat());
      }
    }
    if (temp[1] == 'd') {
      if (temp[3] == '1') {
        Q1.setSpeedD(1, (temp.substring(5)).toFloat());
      } else if (temp[3] == '2') {
        Q1.setSpeedD(2, (temp.substring(5)).toFloat());
      } else if (temp[3] == '3') {
        Q1.setSpeedD(3, (temp.substring(5)).toFloat());
      } else if (temp[3] == '4') {
        Q1.setSpeedD(4, (temp.substring(5)).toFloat());
      } else if (temp[3] == 'a') {
        Q1.setSpeedD(1, (temp.substring(5)).toFloat());
        Q1.setSpeedD(2, (temp.substring(5)).toFloat());
        Q1.setSpeedD(3, (temp.substring(5)).toFloat());
        Q1.setSpeedD(4, (temp.substring(5)).toFloat());
      }
    }
    if (temp[1] == 's') {
      if (temp[3] == '1') {
        mSpeed[0] = temp.substring(5).toFloat();
      } else if (temp[3] == '2') {
        mSpeed[1] = temp.substring(5).toFloat();
      } else if (temp[3] == '3') {
        mSpeed[2] = temp.substring(5).toFloat();
      } else if (temp[3] == '4') {
        mSpeed[3] = temp.substring(5).toFloat();
      } else if (temp[3] == 'a') {
        mSpeed[0] = temp.substring(5).toFloat();
        mSpeed[1] = temp.substring(5).toFloat();
        mSpeed[2] = temp.substring(5).toFloat();
        mSpeed[3] = temp.substring(5).toFloat();
      }
    }
    if (temp[1] == 'n') {
      if (temp[3] == '1') {
        speedFlag[0] = (temp.substring(5)).toInt();
      } else if (temp[3] == '2') {
        speedFlag[1] = (temp.substring(5)).toInt();
      } else if (temp[3] == '3') {
        speedFlag[2] = (temp.substring(5)).toInt();
      } else if (temp[3] == '4') {
        speedFlag[3] = (temp.substring(5)).toInt();
      } else if (temp[3] == 'a') {
        speedFlag[0] = (temp.substring(5)).toInt();
        speedFlag[1] = (temp.substring(5)).toInt();
        speedFlag[2] = (temp.substring(5)).toInt();
        speedFlag[3] = (temp.substring(5)).toInt();
      }
    }
    updateSpeed();
    displayParameter();


    if (temp[0] == 'm') {
      Serial.println(WiFi.macAddress());
    }
    if (temp[1] == 'a') {

      for (uint16_t i = 0; i < 12; i++) {

        Serial.print(i);
        Serial.print(" ");
        Serial.println(RXPacket[i]);
      }
    }

    if (temp[1] == 'r') {
      displayEnable = (temp.substring(3)).toInt();
    }
    if (temp[0] == 'e' && temp[1] == 'n') {
      Serial.println(timeOutCounter);
    }
  }
}

void displayParameter() {
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.print("kp : ");
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(Q1.getSpeedP(i + 1), 10);
    Serial.print(" ");
  }
  Serial.println("");
  Serial.print("ki : ");
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(Q1.getSpeedI(i + 1), 10);
    Serial.print(" ");
  }
  Serial.println("");
  Serial.print("kd : ");
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(Q1.getSpeedD(i + 1), 10);
    Serial.print(" ");
  }
  Serial.println("");
  Serial.print("ks : ");
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(Q1.getSpeed(i + 1), 10);
    Serial.print(" ");
  }
  Serial.println("");
  Serial.print("kn : ");
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(speedFlag[i], 10);
    Serial.print("            ");
  }
  Serial.println("");
  Serial.println("");
}
