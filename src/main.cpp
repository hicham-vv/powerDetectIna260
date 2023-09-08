#include <Arduino.h>
#include <esp_now.h>
#include <Adafruit_INA260.h>
#include <esp_wifi.h>

#include "esp_task_wdt.h"
#ifdef ESP32
  #include <WiFi.h>
#endif

// #define test


#define debug
#define Repeater // define had la ligne fin tanabghiw nwasslo data dial la carte TracCAN b had la carte powerdetect


#define PorteArriere // Pour lire daa de la porte Arriere
// #define FuelLV





uint8_t selfMAC[] = {0x00, 0xBB, 0x00, 0x00, 0x98, 0x00}; // Mac adress de la carte Mére lui même 
uint8_t receiverMAC[] = {0x00, 0xAA, 0x00, 0x00, 0x98, 0x00}; // Master MAC Adress 
#ifdef Repeater
uint8_t CanSenderMac[] = {0x00, 0xCC, 0x00, 0x00, 0x33, 0x03}; // Adress CAN ila ila definiti Repeater 24:0a:c4:08:4f:64
#endif


Adafruit_INA260 ina260_1 = Adafruit_INA260();

esp_now_peer_info_t peerInfo;

#define BusVoltage
#define TensionSeuil 10
#define Led_esp 2  
boolean ledState = false;

#define RETRY_INTERVAL 5000
#define SEND_INTERVAL 1000 
#define N 10 // nombre de tags // il faut initialiser les valeurs du tableau winnerRSSI[N] par -255  à chaque fois cette valeur est changée 






unsigned long previousMillis;
unsigned long currentMillis;

unsigned long sentStartTime;
unsigned long lastSentTime;

#define cSize 24 // Trame Powerdetect size

typedef struct message {
  char V1 = '0';
  char V2 = '0';
  char V3 = '0';
  int TotalDistance =-1;
  int FuelTank =-1;
  int TotalHours =-1;
  int TotalFuelused =-1;
  int CoolantTemp =-1;
  int RPM =-1;
  int BrakePP=-1;
  int AccPP=-1;

  char Nsensor[cSize+1]= {'0','0','0','0','0','0','0','0',
                          '0','0','0','0','0','0','0','0',
                          '0','0','0','0','0','0','0','0'};
  int CAN9=-1;
  int CAN10=-1;
}message;
message bus; // créer une structure message nommé bus

int voltage=0;

char refPD1='0';
bool send=false;
bool SendOK=false;
int waterlv=0;
int Mwaterlv=0;

#ifdef Repeater
bool SendCandata=false;
#endif


char refPD[cSize+1]= {'0','0','0','0','0','0','0','0',
                      '0','0','0','0','0','0','0','0',
                      '0','0','0','0','0','0','0','0'};
char trame[cSize+1]= {'0','0','0','0','0','0','0','0',
                      '0','0','0','0','0','0','0','0',
                      '0','0','0','0','0','0','0','0'};


bool compareArrays(char a[], char b[], int n);
void copyArrays(char a[], char b[], int n);
void blinkLed(uint16_t time_Out,uint16_t ms);


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

#ifdef Repeater
  void OnDataRecv(const uint8_t * senderMac, const uint8_t *incomingData, int len) {
    memcpy(&bus, incomingData, len);
    #ifdef debug
    Serial.printf("Transmitter MacAddr: %02x:%02x:%02x:%02x:%02x:%02x \n", senderMac[0], senderMac[1], senderMac[2], senderMac[3], senderMac[4], senderMac[5]);
    #endif
    uint8_t iCan=0;
    for(int i=0;i<6;i++){
      if(senderMac[i]==CanSenderMac[i]){
        iCan++;
      }
    }
    if(iCan==6){
      #ifdef debug
      Serial.println("\n*****************CAN DATA***************\n");
      Serial.print("Fuel Lv=");Serial.print(bus.FuelTank);Serial.println(" %");
      Serial.print("Total Distance=");Serial.print(bus.TotalDistance);Serial.println(" Km");
      Serial.print("Total Hours=");Serial.print(bus.TotalHours);Serial.println(" H");
      Serial.print("Total Fuel used=");Serial.print(bus.TotalFuelused);Serial.println(" L");
      Serial.print("Coolant Temperature=");Serial.print(bus.CoolantTemp);Serial.println(" C");
      Serial.print("RPM=");Serial.print(bus.RPM);Serial.println(" rpm");
      Serial.print("BrakePP=");Serial.print(bus.BrakePP);Serial.println(" %");
      Serial.print("AccPP=");Serial.print(bus.AccPP);Serial.println(" %");
      Serial.print("Weight=");Serial.print(bus.CAN10);Serial.println(" Kg");
      Serial.println("********************************\n");
      #endif
      SendCandata=true;
    }
  }
#endif


void setup() {


  esp_task_wdt_init(60, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

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

  esp_task_wdt_reset();
  

  WiFi.mode(WIFI_STA); // set the wifi mode as Station
  esp_wifi_set_mac(WIFI_IF_STA, &selfMAC[0]);
  if (esp_now_init() != ESP_OK) {
    #ifdef debug
    Serial.println("ESP_Now init failed...");
    #endif
    delay(3000);
    esp_restart();
  }else{
    #ifdef debug
    Serial.println("ESP_Now init OK...");
    #endif

  }

    #ifdef Repeater
    esp_now_register_recv_cb(OnDataRecv);
    #endif

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    esp_restart();
  }
  }

void loop() {

    #ifdef PorteArriere
    uint8_t compteur=0;
    for(int i=0;i<6;i++){
      voltage=ina260_1.readBusVoltage();
      voltage=voltage/1000;
      if(voltage>TensionSeuil){
        compteur++;
      }
      delay(5);
    }

    if(compteur>2){
      trame[15]='1';
      #ifdef debug
      Serial.println("Levée de la porte arrière ON");
      #endif
    }else{
      #ifdef debug
      Serial.println("Levée de la porte arrière OFF");
      #endif
      trame[15]='0';
    }

  bool compare=compareArrays(refPD,trame,cSize);
  if(!compare){
      copyArrays(refPD,trame,cSize);
      copyArrays(bus.Nsensor,trame,cSize);
      #ifdef Repeater
      esp_now_unregister_recv_cb();
      #endif
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
      #ifdef Repeater
      esp_now_register_recv_cb(OnDataRecv);
      #endif
    }


    // Send data of can
    #ifdef Repeater
      if(SendCandata){
        esp_now_unregister_recv_cb();
        delay(100);
        bus.V1='1';
        for(int i=0;i<3;i++){
          sendData();
          delay(1500);
          if(SendOK){
            blinkLed(500,25);
            break;
          }
        }
        bus.V1='0';
        SendCandata=false;
        esp_now_register_recv_cb(OnDataRecv);
      }
    #endif
    delay(2000);



    #endif


#ifdef FuelLV
int comp=0;

  for(int i=0;i<100;i++){
    waterlv=ina260_1.readBusVoltage();
    Mwaterlv=Mwaterlv+waterlv;
    comp++;
    delay(10);
  }

  Mwaterlv=Mwaterlv/comp; // la moyenne



  Serial.print("Moyenne=");Serial.println(Mwaterlv);


  bus.FuelTank=Mwaterlv;
  Serial.print("Fuel Lv=");Serial.print(bus.FuelTank);Serial.println(" mV");

  
  #ifdef debug
  Serial.println("\nSending Data\n");
  #endif
  for(int i=0;i<3;i++){
    sendData();
    delay(1500);
    if(SendOK){
      blinkLed(500,25);
      break;
    }
  }

  esp_task_wdt_reset();
  delay(10000);


#endif

}



void blinkLed(uint16_t time_Out,uint16_t ms){
  previousMillis=millis();
  while((millis()-previousMillis)<time_Out){
    ledState = !ledState;
    digitalWrite(Led_esp,ledState);
    delay(ms);
  }
}

bool compareArrays(char a[], char b[], int n){
  for (int i=0; i<n; ++i){
    if (a[i] != b[i]){
        return false;
    }
  }
  return true;
}

void copyArrays(char a[], char b[], int n){
  for (int i=0; i<n; ++i){
    a[i] = b[i];
  }
}