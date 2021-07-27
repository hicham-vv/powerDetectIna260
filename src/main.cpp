#include <Arduino.h>
#include <esp_now.h>
#include <Adafruit_INA260.h>

#ifdef ESP32
  #include <WiFi.h>
#endif

// #define debug
uint8_t receiverMAC[] = {0x9C, 0x9C, 0x1F, 0xD8, 0x22, 0x64}; // TracCar MAC Adress


Adafruit_INA260 ina260 = Adafruit_INA260();

#define RETRY_INTERVAL 5000
#define SEND_INTERVAL 1000 
#define N 10 // nombre de tags // il faut initialiser les valeurs du tableau winnerRSSI[N] par -255  à chaque fois cette valeur est changée 

unsigned long previousMillis;
unsigned long currentMillis;

unsigned long sentStartTime;
unsigned long lastSentTime;

typedef struct message {
int current=0; // initialisation des valeurs winners RSSI 
int voltage=0; // initialisation des valeurs winners RSSI 
int power=0; // initialisation des valeurs winners RSSI 
}message;
message bus; // créer une structure message nommé bus

void sendData() {
  sentStartTime = micros();
  esp_now_send(receiverMAC,(uint8_t *) &bus, sizeof(bus)); // NULL means send to all peers
}


void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  if(status == ESP_NOW_SEND_SUCCESS){Serial.println("points sents");}
}

void setup() {
  #ifdef debug
  Serial.begin(115200);
  while(!Serial);
  #endif
  if (!ina260.begin()){
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
    ESP.restart();
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
  bus.current=ina260.readCurrent();
  bus.voltage=ina260.readBusVoltage();
  bus.power=ina260.readPower();
  sendData();
  delay(10000);
}