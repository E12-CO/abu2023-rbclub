#include "CRU.h"
#include <esp_now.h>
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"


typedef struct struct_message {
  uint8_t gripper;
} struct_message;

struct_message myData;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.println("A");
  if(myData.gripper == 0){
    CRU.writeMicroseconds(0,500);
    CRU.writeMicroseconds(1,2500);
  }
  else if(myData.gripper == 1){
    CRU.writeMicroseconds(0,1000);
    CRU.writeMicroseconds(1,2000);
  }
  else if(myData.gripper == 2){
    CRU.writeMicroseconds(0,1500);
    CRU.writeMicroseconds(1,1500);
  }
  
  else if(myData.gripper == 3){
    CRU.writeMicroseconds(0,2000);
    CRU.writeMicroseconds(1,1000);
  }
  
  else if(myData.gripper == 4){
    CRU.writeMicroseconds(0,2300);
    CRU.writeMicroseconds(1,700);
  }
  
  
}
 
void setup() {
  Serial.begin(115200);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  CRU.init();

  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  delay(1);
}