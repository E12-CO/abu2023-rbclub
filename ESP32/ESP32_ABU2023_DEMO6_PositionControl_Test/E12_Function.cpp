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

void processRollerSpeed();


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
  writeChipSelect(0xFE);
  delayMicroseconds(1);
  SPI.transferBytes(TXPacket, RXPacket, packetSize + 1);
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
  for (uint8_t i = 0; i < 4; i++) {
    Q1.setSpeed(i + 1, mSpeed[i]);
  }
}

void updatePos(){
  for (uint8_t i = 0; i < 4; i++) {
    Q1.setPos(i + 1, mPos[i]);
  }
}
void serialReceive() {
  while (Serial.available() > 0) {
    String temp = Serial.readString();
    if (temp[0] == 'k') {
      if (temp[1] == 'p') {
        if (temp[2] == 's') {
          if (temp[4] == '1') {
            Q1.setSpeedP(1, (temp.substring(6)).toFloat());
          } else if (temp[4] == '2') {
            Q1.setSpeedP(2, (temp.substring(6)).toFloat());
          } else if (temp[4] == '3') {
            Q1.setSpeedP(3, (temp.substring(6)).toFloat());
          } else if (temp[4] == '4') {
            Q1.setSpeedP(4, (temp.substring(6)).toFloat());
          } else if (temp[4] == 'a') {
            Q1.setSpeedP(1, (temp.substring(6)).toFloat());
            Q1.setSpeedP(2, (temp.substring(6)).toFloat());
            Q1.setSpeedP(3, (temp.substring(6)).toFloat());
            Q1.setSpeedP(4, (temp.substring(6)).toFloat());
          }
        } else if (temp[2] == 'p') {
          if (temp[4] == '1') {
            Q1.setPosP(1, (temp.substring(6)).toFloat());
          } else if (temp[4] == '2') {
            Q1.setPosP(2, (temp.substring(6)).toFloat());
          } else if (temp[4] == '3') {
            Q1.setPosP(3, (temp.substring(6)).toFloat());
          } else if (temp[4] == '4') {
            Q1.setPosP(4, (temp.substring(6)).toFloat());
          } else if (temp[4] == 'a') {
            Q1.setPosP(1, (temp.substring(6)).toFloat());
            Q1.setPosP(2, (temp.substring(6)).toFloat());
            Q1.setPosP(3, (temp.substring(6)).toFloat());
            Q1.setPosP(4, (temp.substring(6)).toFloat());
          }
        }
      }
      else if (temp[1] == 'i') {
        if (temp[2] == 's') {
          if (temp[4] == '1') {
            Q1.setSpeedI(1, (temp.substring(6)).toFloat());
          } else if (temp[4] == '2') {
            Q1.setSpeedI(2, (temp.substring(6)).toFloat());
          } else if (temp[4] == '3') {
            Q1.setSpeedI(3, (temp.substring(6)).toFloat());
          } else if (temp[4] == '4') {
            Q1.setSpeedI(4, (temp.substring(6)).toFloat());
          } else if (temp[4] == 'a') {
            Q1.setSpeedI(1, (temp.substring(6)).toFloat());
            Q1.setSpeedI(2, (temp.substring(6)).toFloat());
            Q1.setSpeedI(3, (temp.substring(6)).toFloat());
            Q1.setSpeedI(4, (temp.substring(6)).toFloat());
          }
        } else if (temp[2] == 'p') {
          if (temp[4] == '1') {
            Q1.setPosI(1, (temp.substring(6)).toFloat());
          } else if (temp[4] == '2') {
            Q1.setPosI(2, (temp.substring(6)).toFloat());
          } else if (temp[4] == '3') {
            Q1.setPosI(3, (temp.substring(6)).toFloat());
          } else if (temp[4] == '4') {
            Q1.setPosI(4, (temp.substring(6)).toFloat());
          } else if (temp[4] == 'a') {
            Q1.setPosI(1, (temp.substring(6)).toFloat());
            Q1.setPosI(2, (temp.substring(6)).toFloat());
            Q1.setPosI(3, (temp.substring(6)).toFloat());
            Q1.setPosI(4, (temp.substring(6)).toFloat());
          }
        }
      }
      else if (temp[1] == 'd') {
        if (temp[2] == 's') {
          if (temp[4] == '1') {
            Q1.setSpeedD(1, (temp.substring(6)).toFloat());
          } else if (temp[4] == '2') {
            Q1.setSpeedD(2, (temp.substring(6)).toFloat());
          } else if (temp[4] == '3') {
            Q1.setSpeedD(3, (temp.substring(6)).toFloat());
          } else if (temp[4] == '4') {
            Q1.setSpeedD(4, (temp.substring(6)).toFloat());
          } else if (temp[4] == 'a') {
            Q1.setSpeedD(1, (temp.substring(6)).toFloat());
            Q1.setSpeedD(2, (temp.substring(6)).toFloat());
            Q1.setSpeedD(3, (temp.substring(6)).toFloat());
            Q1.setSpeedD(4, (temp.substring(6)).toFloat());
          }
        } else if (temp[2] == 'p') {
          if (temp[4] == '1') {
            Q1.setPosD(1, (temp.substring(6)).toFloat());
          } else if (temp[4] == '2') {
            Q1.setPosD(2, (temp.substring(6)).toFloat());
          } else if (temp[4] == '3') {
            Q1.setPosD(3, (temp.substring(6)).toFloat());
          } else if (temp[4] == '4') {
            Q1.setPosD(4, (temp.substring(6)).toFloat());
          } else if (temp[4] == 'a') {
            Q1.setPosD(1, (temp.substring(6)).toFloat());
            Q1.setPosD(2, (temp.substring(6)).toFloat());
            Q1.setPosD(3, (temp.substring(6)).toFloat());
            Q1.setPosD(4, (temp.substring(6)).toFloat());
          }
        }
      }
      else if (temp[1] == 'm') {
        if (temp[3] == '1') {
          mode[0] = temp.substring(5).toInt();
        } else if (temp[3] == '2') {
          mode[1] = temp.substring(5).toInt();
        } else if (temp[3] == '3') {
          mode[2] = temp.substring(5).toInt();
        } else if (temp[3] == '4') {
          mode[3] = temp.substring(5).toInt();
        } else if (temp[3] == 'a') {
          mode[0] = temp.substring(5).toInt();
          mode[1] = temp.substring(5).toInt();
          mode[2] = temp.substring(5).toInt();
          mode[3] = temp.substring(5).toInt();
        }
      }
    } 
    else if (temp[0] == 's') {
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
      } else if (temp[1] == 'p') {
        if (temp[3] == '1') {
          mPos[0] = strtoll(temp.substring(5).c_str(), NULL, 10);
        } else if (temp[3] == '2') {
          mPos[1] = strtoll(temp.substring(5).c_str(), NULL, 10);
        } else if (temp[3] == '3') {
          mPos[2] = strtoll(temp.substring(5).c_str(), NULL, 10);
        } else if (temp[3] == '4') {
          mPos[3] = strtoll(temp.substring(5).c_str(), NULL, 10);
        } else if (temp[3] == 'a') {
          mPos[0] = strtoll(temp.substring(5).c_str(), NULL, 10);
          mPos[1] = strtoll(temp.substring(5).c_str(), NULL, 10);
          mPos[2] = strtoll(temp.substring(5).c_str(), NULL, 10);
          mPos[3] = strtoll(temp.substring(5).c_str(), NULL, 10);
        }
      }
    }
    updateSpeed();
    displayParameter();


    if (temp[0] == 'm') {
      Serial.println(WiFi.macAddress());
    }

    if (temp[1] == 'r') {
      displayEnable = (temp.substring(3)).toInt();
    }
  }
}

void displayParameter() {
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("Speed Control");
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
  Serial.print("speed : ");
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(Q1.getSpeed(i + 1), 10);
    Serial.print(" ");
  }
  Serial.println("");

  Serial.println("Position Control");
  Serial.print("kp : ");
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(Q1.getPosP(i + 1), 10);
    Serial.print(" ");
  }
  Serial.println("");
  Serial.print("ki : ");
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(Q1.getPosI(i + 1), 10);
    Serial.print(" ");
  }
  Serial.println("");
  Serial.print("kd : ");
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(Q1.getPosD(i + 1), 10);
    Serial.print(" ");
  }
  Serial.println("");
  Serial.print("position : ");
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(Q1.getPos(i + 1), 10);
    Serial.print(" ");
  }
  Serial.println("");
  Serial.print("Mode : ");
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(mode[i]);
    Serial.print(" ");
  }
  Serial.println("");
}
