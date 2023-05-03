
#include "CRU.h"
#include <esp_now.h>
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
uint32_t timeOutCounter = 0;

typedef struct struct_message {
  uint8_t feedMotor;
  uint8_t lockMotor;
} struct_message;

struct_message myData;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  timeOutCounter = 0;
  CRU.writeInd(0x02);
  CRU.update();
}
 
void setup() {
  Serial.begin(115200);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  CRU.init();

  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  if(myData.feedMotor){
    CRU.writeMotor(0,4095,0x01);
    CRU.update();
  }
  else{
    CRU.writeMotor(0,0,0x00);
    CRU.update();
  }

  if(myData.lockMotor == 2){
    CRU.writeMotor(1,3600,0x01);
    CRU.update();
  }
  else if (myData.lockMotor == 1){
    CRU.writeMotor(1,3600,0x02);
    CRU.update();
  }
  else{
    CRU.writeMotor(1,3600,0);
    CRU.update();
  }
  timeOutCounter++;
  if(timeOutCounter > 250){
    for(uint8_t i = 0;i<4;i++){
      CRU.writeMotor(i,0,0);
    }
    CRU.writeInd(0x03);
    CRU.update();
  }
  delay(1);



}
