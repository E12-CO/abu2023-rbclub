#include <esp_now.h>
#include <WiFi.h>

uint8_t stateRT = 1,stateRD = 1,stateRL = 1,stateRR = 1;
uint8_t stateLT = 1,stateLD = 1,stateLL = 1,stateLR = 1;

uint8_t laststateRT = 1,laststateRD = 1,laststateRL = 1,laststateRR = 1;
uint8_t laststateLT = 1,laststateLD = 1,laststateLL = 1,laststateLR = 1;

byte incomingByte;
double sensorData[6] = {0, 0, 0, 0, 0, 0};
int ind1 = LOW;

typedef struct struct_message {
  int32_t id;
   uint8_t rt,rd,rl,rr,lt,ld,ll,lr;
  int32_t encl,encr;
  int32_t x,y,z,w;
} struct_message;

struct_message myData;

int master = 1;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  if(myData.id == 1){
    if(myData.rt == 1)//placeholder
    {
      //do Palm
    }
    else{
      //do Jame
    }
    if(master = 1){
      jame();//placeholder
    }
  }
  else if(myData.id == 2 && master == 2){
    palm();
  }
  else{
    Serial.println("Error ID");
  }

}

void palm(){
  stateRT = myData.rt;
stateRD = myData.rd;
stateRL = myData.rl;
stateRR = myData.rr;
stateLT = myData.lt;
stateLD = myData.ld;
stateLL = myData.ll;
stateLR = myData.lr;

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
*/
Serial.println("I receive from Palm!");
Serial.print (myData.x);
Serial.print(" ");
Serial.print (myData.y);
Serial.print(" ");
Serial.print (myData.z);
Serial.print(" ");
Serial.print (myData.w);
Serial.println(" ");
Serial.println(" ");
}

void jame(){
  stateRT = myData.rt;
stateRD = myData.rd;
stateRL = myData.rl;
stateRR = myData.rr;
stateLT = myData.lt;
stateLD = myData.ld;
stateLL = myData.ll;
stateLR = myData.lr;

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
*/
Serial.println("I receive from Jame!");
Serial.print (myData.x);
Serial.print(" ");
Serial.print (myData.y);
Serial.print(" ");
Serial.print (myData.z);
Serial.print(" ");
Serial.print (myData.w);
Serial.println(" ");
Serial.println(" ");
}
 
void setup() {
  Serial.begin(115200);
  //pinMode(LED_BUILTIN, OUTPUT);  
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {

  if(stateRT != laststateRT){
    laststateRT = stateRT;

      if(stateRT != LOW){
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
   
  /*
   if (Serial.available()) {
    incomingByte = Serial.read();
    //Serial.println(incomingByte);
   //incomingByte.trim();  
   if (incomingByte == 97){
    ind1 = !ind1;
    digitalWrite(LED_BUILTIN, ind1);
    }
  }
 */ 
}
