#include <Arduino.h>
#include <esp_now.h>
#include <Adafruit_INA260.h>
#include <math.h>

#ifdef ESP32
  #include <WiFi.h>
#endif


#define INA260_I2CADDR_3 0x41 // INA260 I2c Adress 0x41 A1=0/A0=1
#define INA260_I2CADDR_2 0x40 // INA260 default i2c address 0x40 A1=0/A0=0
#define INA260_I2CADDR_1 0x44 // INA260 I2c Adress 0x44 A1=1/A0=0




#define debug
#define BusVoltage


// #define test

#define Laveusecolonne
// #define BalayeuseMeca
// #define CiterneTanger
// #define LaveuseBacTanger
// #define BOM






uint8_t receiverMAC[] = {0x4c,0x11,0xae,0x9b,0xdb,0x44}; // TracCar MAC Adress 4c:11:ae:9b:db:44
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
  int CoolantTemp =-1;
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


  #ifdef Laveusecolonne

  
  uint8_t compteur=0;
  int VidageSale=0;
  int LavageBac=0;
  int waterlv=0;
  int Mwaterlv=0;

  for(int i=0;i<7;i++){
    VidageSale=ina260_1.readBusVoltage();
    VidageSale=VidageSale/1000;
    Serial.println(VidageSale);
    if(VidageSale>TensionSeuil){
      compteur++;
    }
  }
  if(compteur>4){
    Serial.println("Vidage eau sale On");
    bus.PD1='1';
  }else{
    Serial.println("Vidage eau sale OFF");
    bus.PD1='0';
  }
  compteur=0;
  // for(int i=0;i<8;i++){
  //   LavageBac=ina260_2.readBusVoltage();
  //   LavageBac=LavageBac/1000;
  //   Serial.print(LavageBac);Serial.print("*");
  //   if(LavageBac>TensionSeuil){
  //     compteur++;
  //   }
  // }
  // if(compteur>4){
  //   Serial.println("Lavage de BAC ON");
  //   bus.PD2='1';
  // }else{
  //   Serial.println("Lavage de BAC OFF");
  //   bus.PD2='0';
  // }
  // LavageBac=ina260_2.readBusVoltage();
  // unsigned long prevms=millis();
  // while((LavageBac>TensionSeuil) && ((millis()-prevms)<15000)){
  //   LavageBac=ina260_2.readBusVoltage();
  // }

  waterlv=ina260_3.readBusVoltage();
  Mwaterlv=waterlv;
  int comp=1;
  for(int i=0;i<14;i++){
    waterlv=ina260_3.readBusVoltage();
    // Serial.println(waterlv);
    if(waterlv>0 && waterlv<5300){
      Mwaterlv=Mwaterlv+waterlv;
      comp++;
    }
  }
  Mwaterlv=Mwaterlv/comp;
  // Serial.print("Moyenne=");Serial.println(Mwaterlv);
  Mwaterlv=Mwaterlv*1500;
  Mwaterlv=Mwaterlv/5000;
  // Mwaterlv=Mwaterlv+600;
  Serial.print("Moyenne en mm=");Serial.println(Mwaterlv);

  bus.CoolantTemp=Mwaterlv;
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
  if(abs(deltaLv)>200){
    refWaterLV=Mwaterlv;
    send=true;
  }

  if(send){
    send=false;
    for(int i=0;i<3;i++){
      #ifdef debug
      Serial.println("Done");
      Serial.print(bus.PD1);
      // Serial.print(bus.PD2);
      Serial.print("*");
      Serial.print(bus.CoolantTemp);
      #endif
      sendData();
      // delay(7000);
      if(SendOK){
        blinkLed(500,25);
        break;
      }
      // else{
      //   delay(5000);
      // }
    }
  }
  delay(300);

  #endif



  #ifdef LaveuseBacTanger

  
  uint8_t compteur=0;
  int karsher=0;
  int LavageBac=0;
  int waterlv=0;
  int Mwaterlv=0;

  for(int i=0;i<8;i++){
    karsher=ina260_1.readBusVoltage();
    if(karsher>1700 && karsher<1900 ){
      compteur++;
    }
  }
  if(compteur>4){
    Serial.println("Karsher ON");
    bus.PD1='1';
  }else{
    Serial.println("Karsher OFF");
    bus.PD1='0';
  }
  compteur=0;
  for(int i=0;i<8;i++){
    LavageBac=ina260_2.readBusVoltage();
    LavageBac=LavageBac/1000;
    Serial.print(LavageBac);Serial.print("*");
    if(LavageBac>TensionSeuil){
      compteur++;
    }
  }
  if(compteur>4){
    Serial.println("Lavage de BAC ON");
    bus.PD2='1';
  }else{
    Serial.println("Lavage de BAC OFF");
    bus.PD2='0';
  }
  LavageBac=ina260_2.readBusVoltage();
  unsigned long prevms=millis();
  while((LavageBac>TensionSeuil) && ((millis()-prevms)<15000)){
    LavageBac=ina260_2.readBusVoltage();
  }

  waterlv=ina260_3.readBusVoltage();
  Mwaterlv=waterlv;
  int comp=1;
  for(int i=0;i<14;i++){
    waterlv=ina260_3.readBusVoltage();
    // Serial.println(waterlv);
    if(waterlv>0 && waterlv<5300){
      Mwaterlv=Mwaterlv+waterlv;
      comp++;
    }
  }
  Mwaterlv=Mwaterlv/comp;
  // Serial.print("Moyenne=");Serial.println(Mwaterlv);
  Mwaterlv=Mwaterlv*1500;
  Mwaterlv=Mwaterlv/5000;
  Mwaterlv=Mwaterlv+600;
  Serial.print("Moyenne en mm=");Serial.println(Mwaterlv);

  bus.CoolantTemp=Mwaterlv;
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
  if(abs(deltaLv)>150){
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
      // delay(7000);
      if(SendOK){
        blinkLed(500,25);
        break;
      }
      // else{
      //   delay(5000);
      // }
    }
  }
  delay(100);

  #endif







  #ifdef CiterneTanger

  uint8_t compteur=0;
  int cardan=0;
  int karsher=0;
  int waterlv=0;
  int Mwaterlv=0;


  for(int i=0;i<5;i++){
    cardan=ina260_1.readBusVoltage();
    cardan=cardan/1000;
    Serial.println(cardan);
    if(cardan>TensionSeuil){
      compteur++;
    }
  }
  if(compteur>2){
    bus.PD1='1';
  }else{
    bus.PD1='0';
  }
  compteur=0;
  if(bus.PD1=='1'){
    Serial.println("Cardan ON");
    for(int i=0;i<7;i++){
      karsher=ina260_2.readBusVoltage();
      Serial.println(karsher);
      if(karsher>520){
        compteur++;
      }
    }
    if(compteur>2){
      bus.PD2='1';
    }else{
      bus.PD2='0';
    }
  }else{
    Serial.println("Cardan OFF");
    bus.PD2='0';
  }
  if(bus.PD2=='1'){
    Serial.println("Karsher ON");
  }else{
    Serial.println("Karsher OFF");
  }
  waterlv=ina260_3.readBusVoltage();
  Mwaterlv=waterlv;
  int comp=1;
  for(int i=0;i<14;i++){
    waterlv=ina260_3.readBusVoltage();
    Serial.println(waterlv);
    if(waterlv>0 && waterlv<5300){
      Mwaterlv=Mwaterlv+waterlv;
      comp++;
    }
  }
  Mwaterlv=Mwaterlv/comp;
  Serial.print("Moyenne=");Serial.println(Mwaterlv);
  Mwaterlv=Mwaterlv*1500;
  Serial.println(Mwaterlv);
  Mwaterlv=Mwaterlv/5000;
  Serial.print("Moyenne en mm=");Serial.println(Mwaterlv);


  bus.CoolantTemp=Mwaterlv;

  for(int i=0;i<3;i++){
    #ifdef debug
    Serial.println("Done");
    #endif
    sendData();
    delay(1500);
    if(SendOK){
      blinkLed(500,25);
      delay(10000);
      break;
    }else{
      delay(5000);
    }
  }
  

  
















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
      sendData();
      // delay(7000);
      if(SendOK){
        blinkLed(500,25);
        break;
      }
    }
  }
  delay(3000);
  #endif

  // #ifdef Citerne
  // previousMillis=millis();
  // int voltage1=ina260_1.readBusVoltage();
  // int voltage2=ina260_2.readBusVoltage();
  // int voltage3=ina260_3.readBusVoltage();
  // #ifdef debug
  // Serial.print("PD1=");Serial.println(voltage1);
  // Serial.print("PD2=");Serial.println(voltage2);
  // Serial.print("PD3=");Serial.println(voltage3);
  // #endif
  // bus.TotalFuelused=voltage1;
  // bus.CoolantTemp=voltage3;
  // sendData();
  // bool a=true;
  // while(millis()-previousMillis<180000){
  //   voltage1=ina260_1.readBusVoltage();
  //   voltage2=ina260_2.readBusVoltage();
  //   voltage3=ina260_3.readBusVoltage();
  //   if(voltage2>3180){
  //     a=true;
  //     #ifdef debug
  //     Serial.print("PD1=");Serial.println(voltage1);
  //     Serial.print("PD2=");Serial.println(voltage2);
  //     Serial.print("PD3=");Serial.println(voltage3);
  //     #endif
  //     bus.PD2='0';
  //     bus.TotalFuelused=voltage1;
  //     bus.CoolantTemp=voltage3;
  //     sendData();
  //     delay(2000);
  //   }
  //   if(voltage2>2500 && voltage2 < 2680){
  //     a=true;
  //     #ifdef debug
  //     Serial.print("PD1=");Serial.println(voltage1);
  //     Serial.print("PD2=");Serial.println(voltage2);
  //     Serial.print("PD3=");Serial.println(voltage3);
  //     #endif
  //     bus.PD2='1';
  //     bus.TotalFuelused=voltage1;
  //     bus.CoolantTemp=voltage3;
  //     sendData();
  //     delay(2000);
  //   }
  //   if(voltage2<1500 && a==true){
  //     a=false;
  //     #ifdef debug
  //     Serial.print("PD2=");Serial.println(voltage2);
  //     Serial.print("PD3=");Serial.println(voltage3);
  //     #endif
  //     bus.PD2='0';
  //     bus.TotalFuelused=voltage1;
  //     bus.CoolantTemp=voltage3;
  //     sendData();
  //     delay(2000);
  //   }
  // }
  // #endif
  #ifdef BalayeuseMeca

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
    Serial.print("Port 1 ON");
    #endif
  }else{
    #ifdef debug
    Serial.print("Port 1 OFF");
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
  if(compteur>6){
    bus.PD2='1';
    #ifdef debug
    Serial.print("Port 2 ON");
    #endif
  }else{
    bus.PD2='0';
    #ifdef debug
    Serial.print("Port 2 OFF");
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
  if(compteur>6){
    bus.PD3='1';
    Serial.print("Port 3 ON");

  }else{
    bus.PD3='0';
    Serial.print("Port 3 OFF");
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
      sendData();
      // delay(7000);
      if(SendOK){
        blinkLed(500,25);
        break;
      }
    }
  }
  delay(2000);
  #endif

  #ifdef test
  int voltage1=ina260_1.readBusVoltage();
  Serial.print("Port1=");Serial.println(voltage1);
  // int voltage2=ina260_2.readBusVoltage();
  // Serial.print("Port2=");Serial.println(voltage2);
  // int voltage3=ina260_3.readBusVoltage();
  // Serial.print("Port3=");Serial.println(voltage3);
  // Serial.println("");
  // delay(1000);
  #endif
}