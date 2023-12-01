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
#include "ArduinoHttpClient.h"
#include <Update.h>
#include <Preferences.h>
#include "FS.h"
#include "SD.h"
#include "SPIFFS.h"
#include <ArduinoOTA.h>


#define debug


#define Repeater // define had la ligne fin tanabghiw nwasslo data dial la carte TracCAN b had la carte powerdetectV5

#define NiveauEau // define cette ligne ila kana ana9raw niveau d'eau dial Citerne

#define Karsher // Define cette ligne pour detecter le fonctionnement du Karsher

#define DynamiqueKarsher // Define had la ligne fin ikon l karsher Dynamique, sinon  desactiviha





uint8_t selfMAC[] = {0x00, 0xBB, 0x00, 0x00, 0x33, 0x03};

uint8_t receiverMAC[] = {0x00, 0xAA, 0x00, 0x00, 0x33, 0x03}; // Master MAC Adress 30:c6:f7:20:d3:50

#ifdef Repeater
uint8_t CanSenderMac[] = {0x00, 0xCC, 0x00, 0x00, 0x33, 0x03}; // Adress CAN ila ila definiti Repeater 24:0a:c4:08:4f:64
#endif







const char ota_resource[] = "/ARER3303/TRACPWR/firmware.bin"; // Endpoint where the firmware.bin file is stored in the OTA server
/******************* Bosch GLM configuration ******************/

// #define GLM                                        //à définir

#ifdef GLM
#include "BluetoothSerial.h"

#define SEND_LCD                                   //à définir pour l'envoi de la distance
#define RELAY_PIN 3
#define POWERUP_PIN 4                              //RED WIRE     (8 to the right mn lt7t)
#define BT_PIN 25                                  //WHITE WIRE   (5 to the right)


// #define DISTANCE                                // to send distance to LCD screen
#define POURCENTAGE                                // to send load percentage to LCD screen


#define LOAD_UPPER_LIMIT 401                       // correspond à une charge de poids de déchets minimale (camion vide)
#define LOAD_LOWER_LIMIT 350                       // charge élevée

// #define SEND_LOAD                               // to send load percentage to server, otherwise send distance in mm * 100
// #define RELAY                                   // to activate relay 

#define BT_SLEEP_TIME                 3            //in (minutes)
#define DISTANCE_THRESHOLD            2            //in (meters)
#define RELAY_UPTIME                  1            //in (minutes)
#define NUMBER_OF_SAMPLES             3            //nombre de mesurements à prendre pour GLM

#define DISTANCE_THRESHOLD_T          DISTANCE_THRESHOLD * 1000
#define BT_SLEEP_TIME_T               BT_SLEEP_TIME * 60 * 1000
#define RELAY_UPTIME_T                RELAY_UPTIME * 60 * 1000

#define SEUIL                         20

uint8_t lcd_address[6] = {0x24, 0x0a, 0xc4, 0x08, 0x4f, 0x64};  //Addresse MAC de carte ESP32 LCD display
typedef struct{
  uint8_t i;
  float distance = 0;
}LCD_message;
LCD_message lcd_payload;

BluetoothSerial SerialBT;
uint8_t bosch_address[6] = {0x00, 0x13, 0x43, 0xa6, 0xe5, 0x84}; // Adresse MAC de BOSCH GLM
int distanceGLM = 0;


float compute_weight(float distance);
#endif

#define INA260_I2CADDR_1 0x44 // INA260 I2c Adress 0x44 A1=1/A0=0
#define INA260_I2CADDR_2 0x40 // INA260 I2c Adress 0x44 A1=1/A0=0



Adafruit_INA260 ina260_1 = Adafruit_INA260();

Adafruit_INA260 ina260_2 = Adafruit_INA260();


/*********************************************************/
Preferences preferences;
WiFiClient Wificlient;
// #define OTA_WIFI_SSID "hamid"
// #define OTA_WIFI_PASSWORD "ifran123"

#define FIRMWARE_VER 23052301
#define OTA_URL "info.geodaki.com"
#define OTA_SERVER_PORT 100
int ota_port = OTA_SERVER_PORT;




uint32_t firmware_version = FIRMWARE_VER;
uint32_t knownCRC32 = 0x6f50d767;
uint32_t knownFileSize = 1024; // In case server does not send it

#define DEBUG_FLAG 1

#define R1_PIN 26

/* OTA Functions*/

const char* ssid = "insight";
const char* password = "insight";

bool CheckWiFiOta();
void printPercent(uint32_t readLength, uint32_t contentLength);



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
  #define  BA 9    // Brosse devant  // On switch entre le pin de Aroseur devant  et la brosse devant  / mais ils ont pas les mêmes positions dans la trame 
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
boolean ledState = false;

unsigned long previousMillis;

#define cSize 24 // Trame Powerdetect size

esp_now_peer_info_t peerInfo;
esp_now_peer_info_t peerInfo2;



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
  // char a[10];

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




/* FreeRTOS tasks */
void BluetoothTask(void *pvParameters);
void MainTask(void *pvParameters);
TaskHandle_t MainTask_handle;


bool connectWifi();



float getDistance();
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
      Serial.print("Weight=");Serial.print(bus.CAN10);Serial.println(" Kg");
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
  delay(500);
  // esp_task_wdt_init(45, true); //enable panic so ESP32 restarts
  // esp_task_wdt_add(NULL); //add current thread to WDT watch
  pinMode(Attiny_Int,OUTPUT); // Attiny feed  Mode output

  pinMode(R1_PIN, OUTPUT);
  digitalWrite(R1_PIN, LOW);

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


if (!SPIFFS.begin(true)){
    #ifdef debug
    Serial.println("An Error has occurred while mounting SPIFFS");
    #endif
    blinkLed(2000,250);
    esp_restart();
  }
/*****************OTA WiFi Check*************/
// Serial.println(ESP.getFlashChipSize());
// Serial.println("Checking OTA firmware 1 test..");
// if(connectWifi()){
//   if(CheckWiFiOta()){
//     Serial.println("done..");
//     // WiFi.disconnect();
//   }
//   else{
//     Serial.println("fail");
//   }
// }
// WiFi.disconnect();
// WiFi.mode(WIFI_OFF);


 /***********************************/

  // esp_task_wdt_reset();

  #ifdef LCD
  lcd.init();
  lcd.backlight();
  #endif

  #ifdef GLM
  pinMode(RELAY_PIN, OUTPUT);
  #ifdef debug
  Serial.println("Turning off GLM");
  #endif
  
  pinMode(POWERUP_PIN, OUTPUT);
  digitalWrite(POWERUP_PIN, LOW);
  delay(3000);
  pinMode(POWERUP_PIN, INPUT);
  #endif

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
  io_1.pinMode(BA, INPUT); // 9
  io_1.pinMode(LAB, INPUT); // 10
  io_1.pinMode(VES, INPUT); // 11
  io_1.pinMode(OPA, INPUT); // 12
  io_1.pinMode(CLC, INPUT); // 13
  io_1.pinMode(COM, INPUT); // 14
  io_1.pinMode(PT, INPUT);  // 15

  #ifdef NiveauEau
  if (!ina260_1.begin(INA260_I2CADDR_1)){
    #ifdef debug
    Serial.println("Couldn't find INA260 chip 1");
    #endif
    esp_restart();
  }
  else{
    #ifdef debug
    Serial.println("Found INA260 chip 1");
    #endif
  }
  delay(250);
  #endif
  #ifdef Karsher
  if(!ina260_2.begin(INA260_I2CADDR_2)){
    #ifdef debug
    Serial.println("Couldn't find INA260 chip 2");
    #endif
    esp_restart();
  }
  else{
    #ifdef debug
    Serial.println("Found INA260 chip 2");
    #endif
  }
  delay(250);
  #endif


  // esp_task_wdt_reset();

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

    // esp_now_peer_info  receiverinfo;
    // memcpy(receiverinfo.peer_addr, receiverMAC, 6);
    // receiverinfo.channel=0;
    // receiverinfo.encrypt = false;

    // // add the receiver module 
    // if( esp_now_add_peer(&receiverinfo) != ESP_OK){
    //   #ifdef debug
    //   Serial.println("Failed to add the receiver module");
    //   #endif
    //   esp_restart();
    // }

  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    esp_restart();
  }

  /* Adding LCD display ESP32 to peers */
  #ifdef GLM
  memcpy(peerInfo2.peer_addr, lcd_address, 6);
  peerInfo2.channel = 0;  
  peerInfo2.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo2) != ESP_OK){
    Serial.println("Failed to add peer 2");
    esp_restart();
  }
  #endif

  // esp_task_wdt_reset();

  xTaskCreatePinnedToCore(MainTask, "main", 10000, NULL, 4, &MainTask_handle, 0);
  #ifdef GLM
  xTaskCreatePinnedToCore(BluetoothTask, "bluetooth", 10000, NULL, 3, NULL, 1);
  #endif
  }

void loop() {
  delay(1000);
  vTaskDelete(NULL);
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


#ifdef GLM
float getDistance(){
  SerialBT.write((const uint8_t*)"\xC0\x40\x00\xEE", 4 );

  uint8_t buffer[1024];
  int len = SerialBT.readBytes(buffer, sizeof(buffer));

  if (len > 0 && buffer[0] == 0x00){
    uint32_t distance_raw = *((uint32_t*) (buffer + 2));
    float distance_mm = (float) distance_raw * 0.05f;
    return distance_mm;
  }
  else{
    return 0;
  }
}
#endif


void MainTask(void *pvParameters){

  delay(500);
  Serial.println("Main Task Begin");
  uint8_t compteur=0;
  int Mwaterlv=0;
  int comp=0;
  int karsher=0;
  int Mkarsher=0;
  while(true){





  compteur = 0;
  for(int i = 0; i < 4; i ++){
    voltage = io_1.digitalRead(LAB);
    if(!voltage){
      compteur++;
    }
  }
  if(compteur > 2){
    trame[13]='1';
    #ifdef debug
    Serial.println("Lavage Bac ON");
    #endif
  }
  else{
    #ifdef debug
    Serial.println("Lavage Bac OFF");
    #endif
    trame[13] = '0';
  }

  compteur = 0;
  for(int i = 0; i < 4; i ++){
    voltage = io_1.digitalRead(VES);
    if(!voltage){
      compteur++;
    }
  }
  if(compteur > 2){
    trame[14]='1';
    #ifdef debug
    Serial.println("Vidage d'eau Sale ON");
    #endif
  }
  else{
    #ifdef debug
    Serial.println("Vidage d'eau Sale OFF");
    #endif
    trame[14]='0';
  }

  compteur = 0;
  for(int i = 0; i < 4; i ++){
    voltage = io_1.digitalRead(BD);
    if(!voltage){
      compteur++;
    }
  }
  if(compteur > 2){
    trame[0]='1';
    #ifdef debug
    Serial.println("Brosse droite ON");
    #endif
  }
  else{
    #ifdef debug
    Serial.println("Brosse droite OFF");
    #endif
    trame[0] = '0';
  }



  compteur = 0;
  for(int i = 0; i < 4; i ++){
    voltage = io_1.digitalRead(AD);
    if(!voltage){
      compteur++;
    }
  }
  if(compteur > 2){
    trame[1]='1';
    #ifdef debug
    Serial.println("Aspirateur droite ON");
    #endif
  }
  else{
    #ifdef debug
    Serial.println("Aspirateur droite OFF");
    #endif
    trame[1] = '0';
  }




  compteur = 0;
  for(int i = 0; i < 4; i ++){
    voltage = io_1.digitalRead(BG);
    if(!voltage){
      compteur++;
    }
  }
  if(compteur > 2){
    trame[2]='1';
    #ifdef debug
    Serial.println("Brosse gauche ON");
    #endif
  }
  else{
    #ifdef debug
    Serial.println("Brosse gauche OFF");
    #endif
    trame[2] = '0';
  }



  compteur = 0;
  for(int i = 0; i < 4; i ++){
    voltage = io_1.digitalRead(AG);
    if(!voltage){
      compteur++;
    }
  }
  if(compteur > 2){
    trame[3]='1';
    #ifdef debug
    Serial.println("Aspirateur gauche ON");
    #endif
  }
  else{
    #ifdef debug
    Serial.println("Aspirateur gauche OFF");
    #endif
    trame[3] = '0';
  }

  compteur = 0;
   for(int i = 0; i < 4; i ++){
    voltage = io_1.digitalRead(BC);
    if(!voltage){
      compteur++;
    }
  }
  if(compteur > 2){
    trame[4]='1';
    #ifdef debug
    Serial.println("Brosse centrale ON");
    #endif
  }
  else{
    #ifdef debug
    Serial.println("Brosse centrale OFF");
    #endif
    trame[4] = '0';
  }



  compteur = 0;
   for(int i = 0; i < 4; i ++){
    voltage = io_1.digitalRead(AC);
    if(!voltage){
      compteur++;
    }
  }
  if(compteur > 2){
    trame[5]='1';
    #ifdef debug
    Serial.println("Aspirateur centrale ON");
    #endif
  }
  else{
    #ifdef debug
    Serial.println("Aspirateur centrale OFF");
    #endif
    trame[5] = '0';
  }


  compteur= 0;
  for(int i=0;i<4;i++){
    voltage=io_1.digitalRead(BA);
    if(!voltage){
      compteur++;
    }
  }
  if(compteur>2){
    trame[11]='1'; // Position Trame Platform
    #ifdef debug
    Serial.println("Brosse devant ON");
    #endif
  }else{
    #ifdef debug
    Serial.println("Brosse devant OFF");
    #endif
    trame[11]='0'; // Position Trame Platform
  }



  compteur= 0;
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
    trame[6]='1'; // Position Trame Platform
    #ifdef debug
    Serial.println("Levée du caisson ON");
    #endif
  }else{
    #ifdef debug
    Serial.println("Levée du caisson OFF");
    #endif
    trame[6]='0'; // Position Trame Platform
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


  #ifdef NiveauEau
  int waterlv=0;
  comp=0;
  for(int i=0;i<80;i++){
    waterlv=ina260_1.readBusVoltage();
    // Serial.println(waterlv);
    if(waterlv>490 && waterlv<3100){
      Mwaterlv=Mwaterlv+waterlv;
      comp++;
    }
    delay(10);
  }
  if(comp!=0){
    Mwaterlv=Mwaterlv/comp;
  }else{
    Mwaterlv=0;
  }
  if(Mwaterlv>=500){
  Mwaterlv=Mwaterlv-500;
    Mwaterlv=Mwaterlv/0.8;
  }else{
    Mwaterlv=0;
    Serial.print("Water Lv is LOW");
  }
  Serial.print("Moyenne en mm=");Serial.println(Mwaterlv);
  bus.CAN9=Mwaterlv;
  #endif

  
  #ifdef Karsher
    comp=1;
    for(int i=0;i<30;i++){
      karsher=ina260_2.readBusVoltage();
      Mkarsher=Mkarsher+karsher;
      // Serial.println(karsher);
      comp++;
      delay(10);
    }

  Mkarsher=Mkarsher/comp;
  Serial.print("Mkarsher=");
  Serial.println(Mkarsher);

  #ifdef  DynamiqueKarsher
  if(Mkarsher<700){
    Serial.println("Karsher OFF");
    trame[20]='0';
    MkarsherPrec=Mkarsher;
  }else{
    int16_t deltaMean=Mkarsher-MkarsherPrec;
    Serial.println(deltaMean);
    if(deltaMean<-100){
      Serial.println("Karsher ON");
      trame[20]='1';
    }else{
      MkarsherPrec=Mkarsher;
      Serial.println("Karsher OFF");
      trame[20]='0';
    }
  }
  #else
    if(Mkarsher>700){
    Serial.println("Karsher ON");
    trame[20]='1';
    }
    if(Mkarsher<=700){
      Serial.println("Karsher OFF");
      trame[20]='0';
    }
  #endif
  #endif



  int deltaLv;
  deltaLv=Mwaterlv-refWaterLV;
  if(abs(deltaLv)>50){
    bus.CAN9=Mwaterlv;
    refWaterLV=Mwaterlv;
    send=true;
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
    #ifdef GLM
    bus.CAN10 = distanceGLM;
    #endif
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
      #ifdef GLM
      bus.CAN10 = distanceGLM;
      #endif
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
  vTaskDelay(1000/portTICK_PERIOD_MS);
  // esp_task_wdt_reset();
  }
}

#ifdef GLM
float compute_weight(float distance){
  float res= 0;
  if(distance < 350){
    distance = 350;
  }
  if(distance > 401){
    distance = 401;
  }
  res = ((LOAD_UPPER_LIMIT - distance)/(LOAD_UPPER_LIMIT - LOAD_LOWER_LIMIT))*100;
  return res;
}
#endif

#ifdef GLM
void BluetoothTask(void *pvParameters){
  #ifdef debug
  Serial.println("Starting BT task");
  #endif
  float temp_last_distance = 0;
  bool relay_flag = 0;
  bool seuil_check = 0;
  while(1){
  if(!SerialBT.begin("TRACPWR", true) ) {
    #ifdef debug
    Serial.println("========== serialBT failed!");
    #endif
    vTaskDelete(NULL);
  }
  #ifdef debug
  Serial.println("Turning on GLM..");
  #endif
  pinMode(POWERUP_PIN, OUTPUT);
  digitalWrite(POWERUP_PIN, LOW);
  delay(100);
  pinMode(POWERUP_PIN, INPUT);
  delay(3500);


  // Activating Bluetooth using BT_PIN, two impulses

  pinMode(BT_PIN, OUTPUT);

  digitalWrite(BT_PIN, LOW);
  delay(50);
  digitalWrite(BT_PIN, HIGH);
  delay(50);
  digitalWrite(BT_PIN, LOW);
  delay(50/portTICK_PERIOD_MS);

  pinMode(BT_PIN, INPUT);

  #ifdef debug
  Serial.println("Connecting to GLM..");
  #endif

  // taskENTER_CRITICAL(NULL);
  vTaskSuspend(MainTask_handle);
  if(SerialBT.connect(bosch_address)){
  #ifdef debug
  Serial.println("Getting distance..");
  #endif
  float sum_temp = 0;
  uint8_t i = 0;


  /* get average of 5 measurements */
  while(i < NUMBER_OF_SAMPLES){
    float temp;
    temp = getDistance();
    delay(50);
    if (temp != 0){
      sum_temp += temp;
      i++;
    }
    delay(50);
  }
  SerialBT.end();
  vTaskResume(MainTask_handle);
  // taskEXIT_CRITICAL(NULL);

  // éliminer irrégularités au niveau du capteur
  float average_value = (float) sum_temp/NUMBER_OF_SAMPLES;
  if (!seuil_check){
    temp_last_distance = average_value;
    seuil_check = 1;
  }
  if(seuil_check){
    if((average_value - temp_last_distance) > 0 && (average_value - temp_last_distance) < SEUIL ){
      average_value = temp_last_distance;
    }
    else{
      temp_last_distance = average_value;
    }
  }


  #ifndef SEND_LOAD
  distanceGLM = (int) (average_value * 100);
  #endif

  #ifdef debug
  Serial.print("CAN10 value to be sent: ");
  Serial.println(distanceGLM);
  #endif


  #ifdef SEND_LCD

  #ifdef DISTANCE
  lcd_payload.i = 0;
  lcd_payload.distance = average_value;
  #endif

  #ifdef POURCENTAGE
  lcd_payload.i= 1;
  float temp;
  temp = compute_weight(average_value);
  #ifdef debug
  Serial.println("Current weight load (%): ");
  Serial.print(temp);
  #endif
  lcd_payload.distance = temp;
  #ifdef SEND_LOAD
  distanceGLM = (int) (temp * 100);
  #endif
  #endif

  #ifdef POIDS
  lcd_payload.i= 2;
  #endif

  esp_err_t result = esp_now_send(lcd_address, (uint8_t *) &lcd_payload, sizeof(lcd_payload));
  if (result == ESP_OK) {
    #ifdef debug
    Serial.println("Sent with success");
    #endif
  }
  else {
    #ifdef debug
    Serial.println("Error sending the data");
    #endif
  }
  #endif


  #ifdef LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Distance (mm): ");
  lcd.setCursor(0,1);
  lcd.print(average_value);
  #endif

  #ifdef debug
  Serial.print("Distance (mm): ");
  Serial.println(average_value);
  #endif

  #ifdef debug
  Serial.println("Turning off GLM");
  #endif
  
  pinMode(POWERUP_PIN, OUTPUT);
  digitalWrite(POWERUP_PIN, LOW);
  delay(3000);
  pinMode(POWERUP_PIN, INPUT);


  /* relay control */

  #ifdef RELAY
  if(average_value <= DISTANCE_THRESHOLD_T){
    relay_flag = 1;
    #ifdef debug
    Serial.println("Activating RELAY..");
    #endif
    digitalWrite(RELAY_PIN, HIGH);
    delay(RELAY_UPTIME_T);
    #ifdef debug
    Serial.println("Deactivating RELAY..");
    #endif
    digitalWrite(RELAY_PIN, LOW);
  }
  #endif
  }
  else{
    #ifdef debug
    Serial.println("Failed to connect to BOSCH GLM");
    #endif
    vTaskResume(MainTask_handle);
  }
  if(relay_flag){
  relay_flag = 0;
  vTaskDelay((BT_SLEEP_TIME_T - 60000)/portTICK_PERIOD_MS); // 3 minutes
  }
  else{
  vTaskDelay(BT_SLEEP_TIME_T/portTICK_PERIOD_MS); // 3 minutes
  }
}
}
#endif

// void appendFile(fs::FS &fs, const char *path, const char *message)
// {
//     Serial.printf("Appending to file: %s\n", path);

//     File file = fs.open(path, FILE_APPEND);
//     if (!file)
//     {
//         Serial.println("Failed to open file for appending");
//         return;
//     }
//     if (file.print(message))
//     {
//         Serial.println("APOK");
//     }
//     else
//     {
//         Serial.println("APX");
//     }
// }

// void readFile(fs::FS &fs, const char *path)
// {
//     Serial.printf("Reading file: %s\n", path);

//     File file = fs.open(path);
//     if (!file || file.isDirectory())
//     {
//         Serial.println("Failed to open file for reading");
//         return;
//     }

//     Serial.print("Read from file: ");
//     while (file.available())
//     {
//         Serial.write(file.read());
//         delayMicroseconds(100);
//     }
// }

// void writeFile(fs::FS &fs, const char *path, const char *message)
// {
//     Serial.printf("Writing file: %s\n", path);

//     File file = fs.open(path, FILE_WRITE);
//     if (!file)
//     {
//         Serial.println("Failed to open file for writing");
//         return;
//     }
//     if (file.print(message))
//     {
//         Serial.println("File written");
//     }
//     else
//     {
//         Serial.println("Write failed");
//     }
// }

// void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
// {
//     Serial.printf("Listing directory: %s\n", dirname);

//     File root = fs.open(dirname);
//     if (!root)
//     {
//         Serial.println("Failed to open directory");
//         return;
//     }
//     if (!root.isDirectory())
//     {
//         Serial.println("Not a directory");
//         return;
//     }

//     File file = root.openNextFile();
//     while (file)
//     {
//         if (file.isDirectory())
//         {
//             Serial.print("  DIR : ");
//             Serial.println(file.name());
//             if (levels)
//             {
//                 listDir(fs, file.name(), levels - 1);
//             }
//         }
//         else
//         {
//             Serial.print("  FILE: ");
//             Serial.print(file.name());
//             Serial.print("  SIZE: ");
//             Serial.println(file.size());
//         }
//         file = root.openNextFile();
//     }
// }

// void deleteFile(fs::FS &fs, const char *path)
// {
//     Serial.printf("Deleting file: %s\n", path);
//     if (fs.remove(path))
//     {
//         Serial.println("File deleted");
//     }
//     else
//     {
//         Serial.println("Delete failed");
//     }
// }

// bool updateFromFS()
// {
//     File updateBin = SPIFFS.open("/update.bin");
//     if (updateBin)
//     {
//         if (updateBin.isDirectory())
//         {
//             if(DEBUG_FLAG){
//             Serial.println("Directory error");
//             }
//             updateBin.close();
//             return 0;
//         }

//         size_t updateSize = updateBin.size();

//         if (updateSize > 0)
//         {
//             if(DEBUG_FLAG){
//             Serial.println("Starting update");
//             }
//             bool update_status;
//             uint8_t attempt_num = 1;
//             update_status = performUpdate(updateBin, updateSize);
//             if (update_status) return 1;
//             else return 0;
//             }
    
//         else
//         {
//             if(DEBUG_FLAG){
//             Serial.println("Error, archivo vacío");
//             }
//             return 0;
//         }

//         updateBin.close();

//         // whe finished remove the binary from sd card to indicate end of the process
//         deleteFile(SPIFFS, "/update.bin");
//         return 1;
//     }
//     else{
//         if(DEBUG_FLAG){
//         Serial.println("no such binary");
//         }
//         return 0;
//     }
// }

// bool performUpdate(Stream &updateSource, size_t updateSize)
// {
//     if (Update.begin(updateSize))
//     {
//         size_t written = Update.writeStream(updateSource);
//         if (written == updateSize)
//         {
//             if(DEBUG_FLAG){
//             Serial.println("Writes : " + String(written) + " successfully");
//             }
//         }
//         else
//         {
//             if(DEBUG_FLAG){
//             Serial.println("Written only : " + String(written) + "/" + String(updateSize) + ". Retry?");
//             }
//             Update.abort();
//             return 0;
//         }
//         if (Update.end())
//         {
//             if(DEBUG_FLAG){
//             Serial.println("OTA finished!");
//             }
//             if (Update.isFinished())
//             {
//                 digitalWrite(2, LOW);
//                 if(DEBUG_FLAG){
//                 Serial.println("Deleting update file");
//                 }
//                 deleteFile(SPIFFS, "/update.bin");
//                 if(DEBUG_FLAG){
//                 Serial.println("Restart ESP device!");
//                 }
//                 ESP.restart();
//             }
//             else
//             {
//                 if(DEBUG_FLAG){
//                 Serial.println("OTA not finished");
//                 }
//                 return 0;
//             }
//         }
//         else
//         {
//             if(DEBUG_FLAG){
//             Serial.println("Error occured #: " + String(Update.getError()));
//             }
//             Update.abort();
//             return 0;
//         }
//     }
//     else
//     {
//         if(DEBUG_FLAG){
//         Serial.println("Cannot begin update");
//         }
//         return 0;
//     }
// }





void printPercent(uint32_t readLength, uint32_t contentLength)
{
    // If we know the total length
    if (contentLength != -1)
    {
        // Serial.print("\r ");
        Serial.print((100.0 * readLength) / contentLength);
        Serial.println('%');
    }
    else
    {
        Serial.println(readLength);
    }
}

bool connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.status() != WL_CONNECTED) {
    #ifdef debug
    Serial.print("Connecting to Wifi AP.");
    #endif
    for ( int i = 0; i < 5; i++) {
      if (WiFi.status() == WL_CONNECTED){
        break;
      }
      #ifdef debug
      Serial.print(".");
      #endif
      delay(1000);
    }
  }
  if(WiFi.status() == WL_CONNECTED) {
    #ifdef debug
    Serial.println("\nconnected...yesss! :)");
    #endif
    return true;
  }
  else {
    #ifdef debug
    Serial.println("\n*** Time out !!! Something wrong happened. It did not connected...***");
    #endif
    return false;
  }
}



bool CheckWiFiOta(){
  uint32_t contentLength = knownFileSize;
  HttpClient http(Wificlient, OTA_URL, OTA_SERVER_PORT);
  File file = SPIFFS.open("/update.bin", FILE_APPEND);
  http.get(ota_resource);
  while(http.headerAvailable())
      {
       String headerName = http.readHeaderName();
       if (headerName == "X-Version"){
          String headerValue = http.readHeaderValue();
          firmware_version = headerValue.toInt();
          // Serial.println(headerName);
          // Serial.println(headerValue);
       }
       if (headerName == "Content-Length"){
        String headerValue = http.readHeaderValue();
        contentLength = headerValue.toInt();
        // Serial.println(headerName);
        // Serial.println(headerValue);
       }
      }
    if(DEBUG_FLAG){
    Serial.println("Available version:");
    Serial.println(firmware_version);
    Serial.println("Current firmware version:");
    Serial.println(FIRMWARE_VER);
    }
    uint32_t readLength = 0;
    long length = http.contentLength();
    Serial.println("Length is : ");
    Serial.print(length);
    Serial.print(" bytes");
    if(FIRMWARE_VER == firmware_version){
    Serial.println("hang 1");
    if(!InternalStorage.open(length)){
      http.stop();
      Serial.println("There is not enough space to continue with the update");
      return 0;
    }
    Serial.println("hang 3");
    // while(1);
    byte b;
    while(length > 0){
      if(!http.readBytes(&b, 1)){
        Serial.println("hang 4");
        break;
      }
      if (readLength % (contentLength / 25) == 0)
                {
                    if(DEBUG_FLAG){
                    printPercent(readLength, contentLength);
                    Serial.println(http.connected());
                    }
                }
      // Serial.println(length);
      // file.print(char(b));
      InternalStorage.write(b);
      length--;
      readLength++;
    }
    Serial.println("hang 6");
    InternalStorage.close();
    http.stop();
    if(length > 0){
      Serial.print("Timeout downloading update file at:");
      Serial.print(length);
      Serial.println("bytes. Can't continue with update.");
      return 0;
    }
    Serial.println("OTA finished.. applying update and reset");
    Serial.flush();
    InternalStorage.apply();
    }
    else{
      Serial.println("OTA version already matching, moving on...");
      return 1;
    }
}