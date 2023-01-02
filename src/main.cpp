#include <Arduino.h>
#include <esp_now.h>
#include <Adafruit_INA260.h>
#include <math.h>
#include "esp_task_wdt.h"

#ifdef ESP32
  #include <WiFi.h>
#endif


#define INA260_I2CADDR_3 0x41 // INA260 I2C Adress 0x41 A1=0/A0=1
#define INA260_I2CADDR_2 0x40 // INA260 default i2c address 0x40 A1=0/A0=0
#define INA260_I2CADDR_1 0x44 // INA260 I2c Adress 0x44 A1=1/A0=0






#define debug

#define DynamiqueKarsher // Define had la ligne fin ikon l karsher Dynamique, sinon  desactiviha 

#define BusVoltage




#define Sensor3V



// #define test
// #define TestDinamyqueKarsher

// #define Laveusecolonne
// #define BalayeuseMeca // pour detecter le niveau d'eau et le karsher
#define CiterneTanger
// #define LaveuseBacTanger
// #define BOM
// #define COPAG






uint8_t receiverMAC[] = {0x4c,0x11,0xae,0x9d,0x6e,0xc0}; // TracCar MAC Adress  4c:11:ae:9d:6e:c0




bool SendOK=false;



Adafruit_INA260 ina260_1 = Adafruit_INA260();
Adafruit_INA260 ina260_2 = Adafruit_INA260();
Adafruit_INA260 ina260_3 = Adafruit_INA260();

#define cSize 24 // Trame Powerdetect size

#define Led_esp 2  
boolean ledState = false;

#define RETRY_INTERVAL 5000
#define SEND_INTERVAL 1000 
#define N 10 // nombre de tags // il faut initialiser les valeurs du tableau winnerRSSI[N] par -255  à chaque fois cette valeur est changée 

#define TensionSeuil 10

unsigned long previousMillis;
unsigned long currentMillis;

unsigned long sentStartTime;
unsigned long lastSentTime;

typedef struct message {
  char PD1 = '0';
  char PD2 = '0';
  char PD3 = '0';
  int TotalDistance =-1;
  int FuelTank =-1;
  int TotalHours =-1;
  int TotalFuelused =-1;
  int CoolantTemp =-1; // 
  int RPM =-1;
  int BrakePP=-1;
  int AccPP=-1;
  // char a[10];

  char Nsensor[cSize+1]= {'0','0','0','0','0','0','0','0',
                          '0','0','0','0','0','0','0','0',
                          '0','0','0','0','0','0','0','0'};
}message;
message bus; // créer une structure message nommé bus


int voltage=0;
int current=0;
int power=0;

char refPD1='0';
char refPD2='0'; 
char refPD3='0'; 
int refWaterLV=0;
bool send=false;

  uint16_t count=0;
  int MkarsherPrec=0;









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
  delay(5000);

  pinMode(Led_esp,OUTPUT);


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
    if (!ina260_2.begin(INA260_I2CADDR_2)){
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
    if (!ina260_3.begin(INA260_I2CADDR_3)){
    #ifdef debug
    Serial.println("Couldn't find INA260 chip 3");
    #endif
    esp_restart();
  }
  else{
    #ifdef debug
    Serial.println("Found INA260 chip 3");
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

  esp_task_wdt_init(30, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  #ifdef Laveusecolonne

  
  uint8_t compteur=0;
  int karsher=0;
  int Mkarsher=0;
  int LavageBac=0;
  int waterlv=0;
  int Mwaterlv=0;


  for(int i=0;i<20;i++){
    karsher=ina260_1.readBusVoltage();
    Mkarsher=Mkarsher+karsher;
    // Serial.println(karsher);
    compteur++;
    delay(10);
  }

  Mkarsher=Mkarsher/compteur;
  Serial.print("Mkarsher=");
  Serial.println(Mkarsher);

  #ifdef  DynamiqueKarsher
  if(Mkarsher<700){
    Serial.println("Karsher OFF");
    bus.PD1='0';
    MkarsherPrec=Mkarsher;
  }else{
    int16_t deltaMean=Mkarsher-MkarsherPrec;
    Serial.println(deltaMean);
    if(deltaMean<-100){
      Serial.println("Karsher ON");
      bus.PD1='1';
    }else{
      MkarsherPrec=Mkarsher;
      Serial.println("Karsher OFF");
      bus.PD1='0';
    }
  }
  #else
    if(Mkarsher>700){
    Serial.println("Karsher ON");
    bus.PD1='1';
    }
    if(Mkarsher<=700){
      Serial.println("Karsher OFF");
      bus.PD1='0';
    }
  #endif





  compteur=0;
  for(int i=0;i<10;i++){
    LavageBac=ina260_2.readBusVoltage();
    LavageBac=LavageBac/1000;
    Serial.print(LavageBac);Serial.print("*");
    if(LavageBac>TensionSeuil){
      compteur++;
    }
  }
  if(compteur>5){
    Serial.println("Lavage de BAC ON");
    bus.PD2='1';
  }else{
    Serial.println("Lavage de BAC OFF");
    bus.PD2='0';
  }

  int comp=0;
  for(int i=0;i<80;i++){
    waterlv=ina260_3.readBusVoltage();
    // Serial.println(waterlv);
    if(waterlv>490 && waterlv<3000){
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

  if(refPD1!=bus.PD1){
    refPD1=bus.PD1;
    send=true;
  }
  if(refPD2!=bus.PD2){
    refPD2=bus.PD2;
    send=true;
  }
  int deltaLv;
  deltaLv=Mwaterlv-refWaterLV;
  if(abs(deltaLv)>50){
    bus.CoolantTemp=Mwaterlv;
    refWaterLV=Mwaterlv;
    send=true;
  }

  if(send){
    send=false;
    for(int i=0;i<3;i++){
      #ifdef debug
      Serial.println("Done");
      Serial.print(bus.PD1);
      Serial.print(bus.PD2);
      Serial.print("*");
      Serial.print(bus.CoolantTemp);
      #endif
      sendData();
      delay(2500);
      if(SendOK){
        blinkLed(500,25);
        break;
      }
      else{
        delay(500);
      }
    }
  }
  delay(250);

  #endif



  #ifdef LaveuseBacTanger

  
  uint8_t compteur=0;
  int karsher=0;
  int Mkarsher=0;
  int LavageBac=0;
  int waterlv=0;
  int Mwaterlv=0;




  for(int i=0;i<20;i++){
    karsher=ina260_1.readBusVoltage();
    Mkarsher=Mkarsher+karsher;
    Serial.println(karsher);
    compteur++;
    delay(10);
  }

  Mkarsher=Mkarsher/compteur;
  Serial.print("Mkarsher=");
  Serial.println(Mkarsher);

  if(Mkarsher<700){
    Serial.println("Karsher OFF");
    bus.PD1='0';
    MkarsherPrec=Mkarsher;
  }else{
    int16_t deltaMean=Mkarsher-MkarsherPrec;
    Serial.println(deltaMean);
    if(deltaMean<-100){
      Serial.println("Karsher ON");
      bus.PD1='1';
    }else{
      MkarsherPrec=Mkarsher;
      Serial.println("Karsher OFF");
      bus.PD1='0';

    }
  }



  compteur=0;
  for(int i=0;i<10;i++){
    LavageBac=ina260_2.readBusVoltage();
    LavageBac=LavageBac/1000;
    Serial.print(LavageBac);Serial.print("*");
    if(LavageBac>TensionSeuil){
      compteur++;
    }
  }
  if(compteur>5){
    Serial.println("Lavage de BAC ON");
    bus.PD2='1';
  }else{
    Serial.println("Lavage de BAC OFF");
    bus.PD2='0';
  }


  int comp=0;
  #ifdef Sensor3V
  for(int i=0;i<80;i++){
    waterlv=ina260_3.readBusVoltage();
    // Serial.println(waterlv);
    if(waterlv>490 && waterlv<2600){
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
  #endif
  Serial.print("Moyenne en mm=");Serial.println(Mwaterlv);

  if(refPD1!=bus.PD1){
    refPD1=bus.PD1;
    send=true;
  }
  if(refPD2!=bus.PD2){
    refPD2=bus.PD2;
    send=true;
  }
  int deltaLv;
  deltaLv=Mwaterlv-refWaterLV;
  if(abs(deltaLv)>50){
    bus.CoolantTemp=Mwaterlv;
    refWaterLV=Mwaterlv;
    send=true;
  }

  if(send){
    send=false;
    for(int i=0;i<3;i++){
      #ifdef debug
      Serial.println("Done");
      Serial.print(bus.PD1);
      Serial.print(bus.PD2);
      Serial.print("*");
      Serial.print(bus.CoolantTemp);
      #endif
      sendData();
      delay(2500);
      if(SendOK){
        blinkLed(500,25);
        break;
      }
      else{
        delay(500);
      }
    }
  }
  delay(1000);

  #endif






  #ifdef CiterneTanger

  int karsher=0;
  int Mkarsher=0;
  int arosseur=0;
  int Marosseur=0;
  int waterlv=0;
  int Mwaterlv=0;
  int compteur=0;

  for(int i=0;i<20;i++){
    karsher=ina260_1.readBusVoltage();
    Mkarsher=Mkarsher+karsher;
    // Serial.println(karsher);
    compteur++;
    delay(10);
  }

  Mkarsher=Mkarsher/compteur;
  Serial.print("Mkarsher=");
  Serial.println(Mkarsher);

  #ifdef  DynamiqueKarsher
  if(Mkarsher<700){
    Serial.println("Karsher OFF");
    bus.PD1='0';
    MkarsherPrec=Mkarsher;
  }else{
    int16_t deltaMean=Mkarsher-MkarsherPrec;
    Serial.println(deltaMean);
    if(deltaMean<-100){
      Serial.println("Karsher ON");
      bus.PD1='1';
    }else{
      MkarsherPrec=Mkarsher;
      Serial.println("Karsher OFF");
      bus.PD1='0';
    }
  }
  #else
    if(Mkarsher>700){
    Serial.println("Karsher ON");
    bus.PD1='1';
    }
    if(Mkarsher<=700){
      Serial.println("Karsher OFF");
      bus.PD1='0';
    }
  #endif

  int comA=0;
  for(int i=0;i<20;i++){
    arosseur=ina260_2.readBusVoltage();
      Serial.println(arosseur);
      Marosseur=Marosseur+arosseur;
      comA++;
      delay(20);
    }
  Marosseur=Marosseur/comA;
  Serial.println(Marosseur);

  if(Marosseur>570){
    Serial.println("Arosseur devant ON");
    bus.PD2='1';
  }else{
    Serial.println("Arosseur devant OFF");
    bus.PD2='0';
  }

  int comp=0;

  for(int i=0;i<80;i++){
    waterlv=ina260_3.readBusVoltage();
    // Serial.println(waterlv);
    if(waterlv>490 && waterlv<2600){
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

  if(refPD1!=bus.PD1){
    refPD1=bus.PD1;
    send=true;
  }
  if(refPD2!=bus.PD2){
    refPD2=bus.PD2;
    send=true;
  }
  int deltaLv;
  deltaLv=Mwaterlv-refWaterLV;
  if(abs(deltaLv)>100){
    bus.CoolantTemp=Mwaterlv;
    refWaterLV=Mwaterlv;
    send=true;
  }

  if(send){
    send=false;
    for(int i=0;i<3;i++){
      #ifdef debug
      Serial.println("Done");
      Serial.print(bus.PD1);
      Serial.print(bus.PD2);
      Serial.print("*");
      Serial.print(bus.CoolantTemp);
      #endif
      sendData();
      delay(2500);
      if(SendOK){
        blinkLed(500,25);
        break;
      }
      else{
        delay(500);
      }
    }
  }

  delay(500);


  #endif

  #ifdef BOM

  uint8_t compteur=0;
  for(int i=0;i<10;i++){
    voltage=ina260_1.readBusVoltage();
    voltage=voltage/1000;
    if(voltage>TensionSeuil){
      compteur++;
    }
  }
  if(compteur>6){
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

  compteur=0;
  for(int i=0;i<10;i++){
    voltage=ina260_2.readBusVoltage();
    voltage=voltage/1000;
    if(voltage>TensionSeuil){
      compteur++;
    }
  }
  if(compteur>5){
    bus.PD2='1';
    #ifdef debug
    Serial.println("Levée du Bac ON");
    #endif
  }else{
    bus.PD2='0';
    #ifdef debug
    Serial.println("Levée du Bac OFF");
    #endif
  }

  compteur=0;
  for(int i=0;i<10;i++){
    voltage=ina260_3.readBusVoltage();
    voltage=voltage/1000;
    if(voltage>TensionSeuil){
      compteur++;
    }
  }
  if(compteur>5){
    bus.PD3='1';
    Serial.println("Cycle LC ON");

  }else{
    bus.PD3='0';
    Serial.println("Cycle LC OFF");
  }


  if(refPD1!=bus.PD1){
    refPD1=bus.PD1;
    send=true;
  }
  if(refPD2!=bus.PD2){
    refPD2=bus.PD2;
    send=true;
  }
  if(refPD3!=bus.PD3){
    refPD3=bus.PD3;
    send=true;
  }
  if(send){
    send=false;
    for(int i=0;i<3;i++){
      #ifdef debug
      Serial.println("Done");
      Serial.print(bus.PD1);
      Serial.print("*");
      Serial.print(bus.PD2);
      Serial.print("*");
      Serial.println(bus.PD3);
      #endif
      sendData();
      delay(2500);
      if(SendOK){
        blinkLed(500,25);
        break;
      }
      else{
        delay(500);
      }
    }
  }
  delay(1000);


  #endif

  #ifdef BalayeuseMeca

  
  uint8_t compteur=0;
  int karsher=0;
  int Mkarsher=0;
  int LavageBac=0;
  int waterlv=0;
  int Mwaterlv=0;


  #ifdef Sensor3V
  int compK=0;
  for(int i=0;i<20;i++){
    karsher=ina260_1.readBusVoltage();
      Serial.println(karsher);
      Mkarsher=Mkarsher+karsher;
      compK++;
      delay(50);
    }
  Mkarsher=Mkarsher/compK;
  Serial.println(Mkarsher);
  if(Mkarsher>800 && Mkarsher<1000){
    Serial.println("Karsher ON");
    bus.PD1='1';
  }else{
    Serial.println("Karsher OFF");
    bus.PD1='0';
  }
  #endif

  int comp=0;

  #ifdef Sensor3V
  for(int i=0;i<100;i++){
    waterlv=ina260_3.readBusVoltage();
    Serial.println(waterlv);
    if(waterlv>490 && waterlv<2600){
      Mwaterlv=Mwaterlv+waterlv;
      comp++;
    }
    delay(20);
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
  #endif
  Serial.print("Moyenne en mm=");Serial.println(Mwaterlv);

  if(refPD1!=bus.PD1){
    refPD1=bus.PD1;
    send=true;
  }

  int deltaLv;
  deltaLv=Mwaterlv-refWaterLV;
  if(abs(deltaLv)>50){
    bus.CoolantTemp=Mwaterlv;
    refWaterLV=Mwaterlv;
    send=true;
  }

  if(send){
    send=false;
    for(int i=0;i<3;i++){
      #ifdef debug
      Serial.println("Done");
      Serial.print(bus.PD1);
      Serial.print("*");
      Serial.print(bus.CoolantTemp);
      #endif
      sendData();
      delay(2500);
      if(SendOK){
        blinkLed(500,25);
        break;
      }
      else{
        delay(500);
      }
    }
  }
  delay(2000);
  #endif

  #ifdef test
  int voltage1=ina260_1.readBusVoltage();
  Serial.print("Port1=");Serial.println(voltage1);
  int voltage2=ina260_2.readBusVoltage();
  Serial.print("Port2=");Serial.println(voltage2);
  int voltage3=ina260_3.readBusVoltage();
  Serial.print("Port3=");Serial.println(voltage3);
  Serial.println("");
  delay(50);
  #endif


  #ifdef TestDinamyqueKarsher

  int karsher=0;
  int Mkarsher=0;
  int compteur=0;

  for(int i=0;i<20;i++){
    karsher=ina260_1.readBusVoltage();
    Mkarsher=Mkarsher+karsher;
    Serial.println(karsher);
    compteur++;
    delay(10);
  }

  Mkarsher=Mkarsher/compteur;
  Serial.print("Mkarsher=");
  Serial.println(Mkarsher);

  if(Mkarsher<700){
    Serial.println("Karsher OFF");
    MkarsherPrec=Mkarsher;
  }else{
    int16_t deltaMean=Mkarsher-MkarsherPrec;
    Serial.println(deltaMean);
    if(deltaMean<-100){
      Serial.println("Karsher ON");
    }else{
      MkarsherPrec=Mkarsher;
      Serial.println("Karsher OFF");
    }
  }

  delay(2000);

  #endif


  esp_task_wdt_reset();
}



  #ifdef COPAG

  uint8_t compteur=0;
  int karsher=0;
  int Mkarsher=0;
  int PorteArriere=0;
  int Trappe=0;
  int waterlv=0;
  int Mwaterlv=0;

  
  compteur=0;
  for(int i=0;i<10;i++){
    PorteArriere=ina260_1.readBusVoltage();
    PorteArriere=PorteArriere/1000;
    // Serial.print(PorteArriere);Serial.print("*");
    if(PorteArriere>TensionSeuil){
      compteur++;
    }
  }
  if(compteur>5){
    Serial.println("Ouverture du porte arriere ON");
    bus.PD1='1';
  }else{
    Serial.println("Ouverture du porte arriere OFF");
    bus.PD1='0';
  }


  compteur=0;
  for(int i=0;i<10;i++){
    Trappe=ina260_2.readBusVoltage();
    Trappe=Trappe/1000;
    // Serial.print(Trappe);Serial.print("*");
    if(Trappe>TensionSeuil){
      compteur++;
    }
  }
  if(compteur>5){
    Serial.println("Trappe ouverte");
    bus.PD2='1';
  }else{
    Serial.println("Trappe Fermée");
    bus.PD2='0';
  }


  int comp=0;

  #ifdef Sensor3V
  for(int i=0;i<100;i++){
    waterlv=ina260_3.readBusVoltage();
    Serial.println(waterlv);
    if(waterlv>490 && waterlv<2600){
      Mwaterlv=Mwaterlv+waterlv;
      comp++;
    }
    delay(20);
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
  #endif
  Serial.print("Moyenne en mm=");Serial.println(Mwaterlv);

  if(refPD1!=bus.PD1){
    refPD1=bus.PD1;
    send=true;
  }
  if(refPD2!=bus.PD2){
    refPD2=bus.PD2;
    send=true;
  }
  int deltaLv;
  deltaLv=Mwaterlv-refWaterLV;
  if(abs(deltaLv)>10){
    bus.CoolantTemp=Mwaterlv;
    refWaterLV=Mwaterlv;
    send=true;
  }

  if(send){
    send=false;
    for(int i=0;i<3;i++){
      #ifdef debug
      Serial.println("Done");
      Serial.print(bus.PD1);
      Serial.print(bus.PD2);
      Serial.print("*");
      Serial.print(bus.CoolantTemp);
      #endif
      sendData();
      delay(2500);
      if(SendOK){
        blinkLed(500,25);
        break;
      }
      else{
        delay(500);
      }
    }
  }
  delay(1000);

  #endif