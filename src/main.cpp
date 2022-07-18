#include <Arduino.h>
#include <esp_now.h>
#include <Adafruit_INA260.h>

#ifdef ESP32
  #include <WiFi.h>
#endif

// #define test

#define debug
#define BusVoltage


#define PorteArriere




#define TensionSeuil 10

uint8_t receiverMAC[] = {0x4c,0x11,0xae,0x9d,0x6e,0x6c}; // TracCar MAC Adress 4c:11:ae:9d:6e:6c






Adafruit_INA260 ina260_1 = Adafruit_INA260();



#define Led_esp 2  
boolean ledState = false;

#define RETRY_INTERVAL 5000
#define SEND_INTERVAL 1000 
#define N 10 // nombre de tags // il faut initialiser les valeurs du tableau winnerRSSI[N] par -255  à chaque fois cette valeur est changée 






unsigned long previousMillis;
unsigned long currentMillis;

unsigned long sentStartTime;
unsigned long lastSentTime;

typedef struct message {
char PD1='0';
char PD2='0';
char PD3='0';
int TotalDistance = -1;
int FuelTank = -1;
int TotalHours = -1;
int TotalFuelused =-1;
int CoolantTemp =-1;
}message;
message bus; // créer une structure message nommé bus

int voltage=0;

char refPD1='0';
bool send=false;
bool SendOK=false;



void blinkLed(uint16_t time_Out,uint16_t ms){
  previousMillis=millis();
  while((millis()-previousMillis)<time_Out){
    ledState = !ledState;
    digitalWrite(Led_esp,ledState);
    delay(ms);
  }
}
void sendData() {
  esp_now_send(receiverMAC,(uint8_t *) &bus, sizeof(bus)); // NULL means send to all peers
}
void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  if(status == ESP_NOW_SEND_SUCCESS){
    #ifdef debug
      Serial.println("points sents");
    #endif
    SendOK=true;
    }else{
    #ifdef debug
      Serial.println("points not sents");
    #endif
    SendOK=false;
    }
}


void setup() {
  #ifdef debug
  Serial.begin(115200);
  while(!Serial);
  #endif
  if (!ina260_1.begin()){
    #ifdef debug
    Serial.println("Couldn't find INA260 chip");
    #endif
    esp_restart();
  }
  else{
    #ifdef debug
    Serial.println("Found INA260 chip");
    #endif
  }
  

  WiFi.mode(WIFI_STA); // set the wifi mode as Station
  if (esp_now_init() != ESP_OK) {
    #ifdef debug
    Serial.println("ESP_Now init failed...");
    #endif
    delay(RETRY_INTERVAL);
    esp_restart();
  }

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info  receiverinfo;
    memcpy(receiverinfo.peer_addr, receiverMAC, 6);
    receiverinfo.channel=0;
    receiverinfo.encrypt = false;

    // add the receiver module 
    if( esp_now_add_peer(&receiverinfo) != ESP_OK){
      #ifdef debug
      Serial.println("Failed to add the receiver module");
      #endif
      esp_restart();
    }
  }

void loop() {

  #ifdef PorteArriere
  uint8_t compteur=0;
  for(int i=0;i<10;i++){
    voltage=ina260_1.readBusVoltage();
    voltage=voltage/1000;
    if(voltage>TensionSeuil){
      compteur++;
    }
    delay(5);
  }

  if(compteur>5){
    bus.PD1='1';
    #ifdef debug
    Serial.println("Levée de la porte arrière ON");
    #endif
  }else{
    #ifdef debug
    Serial.println("Levée de la porte arrière OFF");
    #endif
    bus.PD1='0';
  }

  if(refPD1!=bus.PD1){
    refPD1=bus.PD1;
    send=true;
  }
  if(send){
    send=false;
    for(int i=0;i<5;i++){
      sendData();
      delay(500);
      if(SendOK){
        blinkLed(500,25);
        delay(3000);
        break;
      }
    }
  }
  delay(2000);
  #endif
}