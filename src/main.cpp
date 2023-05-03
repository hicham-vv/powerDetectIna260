#include <Arduino.h>
#include <esp_now.h>
#include <Adafruit_INA260.h>
#include <math.h>
#include "SparkFunSX1509.h"
#include "esp_task_wdt.h"
#include <driver/dac.h>
#ifdef ESP32
  #include <WiFi.h>
#endif
#include <esp_wifi.h>


 

#define Repeater // define had la ligne fin tanabghiw nwasslo data dial la carte TracCAN b had la carte powerdetectV5

#define debug
#define BOM
// #define BenneSat


uint8_t selfMACAddress[] = {0x00, 0xBB, 0x00, 0x00, 0x51, 0x03}; // hadi bach ndefiniw l MAC adress dial la carte li an


uint8_t receiverMAC[] = {0x00, 0xAA, 0x00, 0x00, 0x51, 0x03}; // Master MAC Adress 4c:11:ae:9d:6e:54

#ifdef Repeater
uint8_t CanSenderMac[] = {0x00, 0xCC, 0x00, 0x00, 0x51, 0x03}; // Adress CAN ila ila definiti Repeater 30:C6:F7:30:96:98
#endif


  /******** BLock SX01 ********/

  #define  BD  0   // Brosse Droite
  #define  AD  1   // Aspirateur Droite
  #define  BG  2   // Brosse Gauche
  #define  AG  3   // Aspirateur Gauche
  #define  BC  4   // Brosse Centrale
  #define  AC  5   // Aspirateur Central
  #define  LC  6   // Levée du caisson
  #define  AMA 7   // Aspirateur Manuel arrière
  #define  AP  8   // Activation de la pompe
  #define  ARD 9   // Aroseur devant
  #define  LAB 10  // Lavage Bac
  #define  VES 11  // Vidade Eau sale
  #define  OPA 12  // Ouverture porte arriere
  #define  CLC 13  // Cycle LC
  #define  COM 14  // Compactation
  #define  PT  15  // Pousée Tablier


#ifdef Repeater
bool SendCandata=false;
#endif

bool SendOK=false; // 

SX1509 io_1; // Create an SX1509_1 object to be used throughout
const byte SX1509_ADDRESS_1 = 0x3E;  // SX1509_1 I2C address

#define Led_esp 2
#define Reg_Enable 15
#define RLY1_Enable 26
boolean ledState = false;

unsigned long previousMillis;

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


bool voltage=false;
char refPD[cSize+1]= {'0','0','0','0','0','0','0','0',
                      '0','0','0','0','0','0','0','0',
                      '0','0','0','0','0','0','0','0'};
char trame[cSize+1]= {'0','0','0','0','0','0','0','0',
                      '0','0','0','0','0','0','0','0',
                      '0','0','0','0','0','0','0','0'};
int refWaterLV=0;
bool send=false;



#define Attiny_Int 5 
boolean attiny_intState = false;

hw_timer_t *My_timer = NULL;
void IRAM_ATTR  feedAttiny();









// bool CheckVoltage(uint8_t pos, uint8_t sx);
bool compareArrays(char a[], char b[], int n);
void copyArrays(char a[], char b[], int n);
void blinkLed(uint16_t time_Out,uint16_t ms);
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
      Serial.println("********************************\n");
      #endif
      SendCandata=true;
    }
  }
#endif
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
  pinMode(RLY1_Enable,OUTPUT);
  digitalWrite(RLY1_Enable,LOW);
  delay(500);
  esp_task_wdt_init(25, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  pinMode(Attiny_Int,OUTPUT); // Attiny feed  Mode output

  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &feedAttiny, true);
  timerAlarmWrite(My_timer, 15000000, true); // ISR every 15s
  timerAlarmEnable(My_timer);

  #ifdef debug
  Serial.begin(115200);
  while(!Serial);
  #endif

  delay(500);
  pinMode(Led_esp,OUTPUT);

  pinMode(Reg_Enable,OUTPUT);
  digitalWrite(Reg_Enable,LOW);
  delay(1500);
  pinMode(Reg_Enable,INPUT);
  delay(500);

  esp_task_wdt_reset();

  if(!Wire.begin(21,22,100000)){
    blinkLed(1000,25); // for debugging
    #ifdef debug
    Serial.println("I2C communication can't Begin!!! Check Qwiic Power voltage");
    #endif
    esp_restart();
  }else{
    #ifdef debug
    Serial.println("I2C communication OK");
    #endif
  }
  delay(500);

  if (!io_1.begin(SX1509_ADDRESS_1)){
    #ifdef debug
    Serial.println("SX1509_1 Failed to communicate.");
    #endif
    blinkLed(2000,500);
    esp_restart();
  }
  else{
    #ifdef debug
    Serial.println("SX1509_1 begin");
    #endif
  }

  // Block SX1 // 
  io_1.pinMode(BD, INPUT);  // 0
  io_1.pinMode(AD, INPUT);  // 1
  io_1.pinMode(BG, INPUT);  // 2
  io_1.pinMode(AG, INPUT);  // 3
  io_1.pinMode(BC, INPUT);  // 4
  io_1.pinMode(AC, INPUT);  // 5
  io_1.pinMode(LC, INPUT);  // 6
  io_1.pinMode(AMA, INPUT); // 7
  io_1.pinMode(AP, INPUT);  // 8
  io_1.pinMode(ARD, INPUT); // 9
  io_1.pinMode(LAB, INPUT); // 10
  io_1.pinMode(VES, INPUT); // 11
  io_1.pinMode(OPA, INPUT); // 12
  io_1.pinMode(CLC, INPUT); // 13
  io_1.pinMode(COM, INPUT); // 14
  io_1.pinMode(PT, INPUT);  // 15

  esp_task_wdt_reset();

  WiFi.mode(WIFI_STA); // set the wifi mode as Station
  esp_wifi_set_mac(WIFI_IF_STA, &selfMACAddress[0]);
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
  esp_task_wdt_reset();
  }

void loop() {

  uint8_t compteur=0;
  
  for(int i=0;i<4;i++){
    voltage=io_1.digitalRead(AP);
    if(!voltage){
      compteur++;
    }
  }
  if(compteur>2){
    trame[9]='1'; // Position Trame Platform
    #ifdef debug
    Serial.println("Cardan ON");
    #endif
  }else{
    #ifdef debug
    Serial.println("Cardan OFF");
    #endif
    trame[9]='0'; // Position Trame Platform
  }

  compteur=0;
  for(int i=0;i<4;i++){
    voltage=io_1.digitalRead(LC);
    if(!voltage){
      compteur++;
    }
  }
  if(compteur>2){
    trame[7]='1'; // Position Trame Platform
    #ifdef debug
    Serial.println("Levée du caisson ON");
    #endif
  }else{
    #ifdef debug
    Serial.println("Levée du caisson OFF");
    #endif
    trame[7]='0'; // Position Trame Platform
  }

  compteur=0;
  for(int i=0;i<4;i++){
    voltage=io_1.digitalRead(OPA);
    if(!voltage){
      compteur++;
    }
  }
  if(compteur>2){
    trame[15]='1'; // Position Trame Platform
    #ifdef debug
    Serial.println("Ouverture de la porte arrière ON");
    #endif
  }else{
    #ifdef debug
    Serial.println("Ouverture de la porte arrière OFF");
    #endif
    trame[15]='0'; // Position Trame Platform
  }

  compteur=0;
  for(int i=0;i<4;i++){
    voltage=io_1.digitalRead(CLC);
    if(!voltage){
      compteur++;
    }
  }
  if(compteur>2){
    trame[16]='1'; // Position Trame Platform
    #ifdef debug
    Serial.println("Cycle LC ON"); 
    #endif
  }else{
    trame[16]='0'; // Position Trame Platform
    #ifdef debug
    Serial.println("Cycle LC OFF");
    #endif
  }


  compteur=0;
  for(int i=0;i<4;i++){
    voltage=io_1.digitalRead(COM);
    if(!voltage){
      compteur++;
    }
  }
  if(compteur>2){
    trame[17]='1'; // Position Trame Platform
    #ifdef debug
    Serial.println("Compactation ON");
    #endif

  }else{
    trame[17]='0'; // Position Trame Platform
    #ifdef debug
    Serial.println("Compactation OFF\n");
    #endif
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
    #ifdef debug
    Serial.println("\nSending Data\n");
    #endif
    for(int i=0;i<3;i++){
      sendData();
      delay(1500);
      if(SendOK){
        blinkLed(500,25);
        // delay(1500);
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
  #ifdef debug
  Serial.println("*******************************");
  #endif
  delay(1000);
  esp_task_wdt_reset();

}


void IRAM_ATTR  feedAttiny(){
    attiny_intState=!attiny_intState;
    digitalWrite(Attiny_Int,attiny_intState);
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

/*
bool CheckVoltage(uint8_t pos, uint8_t sx){
  
  uint8_t compteur=0;
  // check if SX 1 or SX2
  if(sx==1){
    for(int i=0;i<30;i++){
      voltage=io_1.digitalRead(pos);
      if(!voltage){
        compteur++;
      }
      delay(5);
    }

    if(compteur>25){
      bus.Nsensor[pos]='1';
      return true;
    }else{
      bus.Nsensor[pos]='0';
      return false;
    }
  }

  if(sx==2){
    for(int i=0;i<30;i++){
      voltage=io_2.digitalRead(pos);
      if(!voltage){
        compteur++;
      }
      delay(5);
    }
    if(compteur>25){
      bus.Nsensor[pos+16]='1';
      return true;
    }else{
      bus.Nsensor[pos+16]='0';
      return false;
    }
  }
  return false;
}
*/