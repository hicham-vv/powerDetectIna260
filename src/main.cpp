#include <Arduino.h>
#include <esp_now.h>
#include <Adafruit_INA260.h>
#include <math.h>
#include "SparkFunSX1509.h"
#include "esp_task_wdt.h"


  /******** BLock SX01 ********/

  #define  BD  0  // Brosse Droite
  #define  AD  1  // Aspirateur Droite
  #define  BG  2  // Brosse Gauche
  #define  AG  3  // Aspirateur Gauche
  #define  BC  4  // Brosse Centrale
  #define  AC  5  // Aspirateur Central

  #define  LC  7  // Levée du caisson
  #define  AMA 8  // Aspirateur Manuel arrière
  #define  AP  9  // Activation de la pompe
  #define  ARD 10  // Aroseur devant
  #define  K1  11  // Karsher 1
  #define  K2  12  // Karsher 2
  #define  LAB 13 // Lavage Bac
  #define  VES 14 // Vidage d'eau sale
  #define  OPA 15  // Ouverture de la Porte arrière

  /******** BLock SX02 ********/

  #define  CLC 0 // Cycle LC --> 16
  #define  COM 1 // Compactation --> 17
  #define  PT  2 // Pousée Tablier --> 18


#ifdef ESP32
  #include <WiFi.h>
#endif




#define debug
#define BusVoltage


// #define test
// #define Test2

// #define Laveusecolonne
#define BalayeuseMeca
// #define CiterneTanger
// #define LaveuseBacTanger
// #define BOM
// #define BenneSat






uint8_t receiverMAC[] = {0x3c,0x71,0xbf,0x86,0x56,0xe0}; // TracCar MAC Adress 3c:71:bf:86:56:e0 
bool SendOK=false; // 98:f4:ab:6b:d6:68



SX1509 io_1; // Create an SX1509_1 object to be used throughout
SX1509 io_2; // Create an SX1509_2 object to be used throughout

const byte SX1509_ADDRESS_1 = 0x3E;  // SX1509_1 I2C address
const byte SX1509_ADDRESS_2 = 0x3F;  // SX1509_2 I2C address





#define Led_esp 2
#define Reg_Enable 15
boolean ledState = false;

#define RETRY_INTERVAL 5000
#define SEND_INTERVAL 1000 
#define N 10 // nombre de tags // il faut initialiser les valeurs du tableau winnerRSSI[N] par -255  à chaque fois cette valeur est changée 

#define TensionSeuil 10

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
  // char a[10];

  char Nsensor[cSize+1]= {'0','0','0','0','0','0','0','0',
                          '0','0','0','0','0','0','0','0',
                          '0','0','0','0','0','0','0','0'};
}message;
message bus; // créer une structure message nommé bus


bool voltage=false;


char refPD[cSize+1]= {'0','0','0','0','0','0','0','0',
                      '0','0','0','0','0','0','0','0',
                      '0','0','0','0','0','0','0','0'};
int refWaterLV=0;
bool send=false;







bool CheckVoltage(uint8_t pos, uint8_t sx);
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

void setup() {
  delay(500);
  esp_task_wdt_init(25, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
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

  esp_task_wdt_reset();
  delay(500);
  if (!io_2.begin(SX1509_ADDRESS_2)){
    #ifdef debug
    Serial.println("SX1509_2 Failed to communicate.");
    #endif
    blinkLed(2000,500);
    esp_restart();
  }
  else{
    #ifdef debug
    Serial.println("SX1509_2 begin");
    #endif
  }

  // Block SX1 // 
  io_1.pinMode(BD, INPUT);
  io_1.pinMode(AD, INPUT);
  io_1.pinMode(BG, INPUT);
  io_1.pinMode(AG, INPUT);
  io_1.pinMode(BC, INPUT);
  io_1.pinMode(AC, INPUT);

  io_1.pinMode(LC, INPUT);
  io_1.pinMode(AMA, INPUT);
  io_1.pinMode(AP, INPUT);

  

  io_1.pinMode(LAB, INPUT);
  io_1.pinMode(VES, INPUT);
  io_1.pinMode(OPA, INPUT);



  // Block SX2 //
  io_2.pinMode(CLC, INPUT);
  io_2.pinMode(COM, INPUT);
  io_2.pinMode(PT, INPUT);
  // io_2.pinMode(3, INPUT);
  // io_2.pinMode(4, INPUT);
  // io_2.pinMode(5, INPUT);
  // io_2.pinMode(6, INPUT);
  



  esp_task_wdt_reset();

  









  WiFi.mode(WIFI_STA); // set the wifi mode as Station
  if (esp_now_init() != ESP_OK) {
    #ifdef debug
    Serial.println("ESP_Now init failed...");
    #endif
    delay(RETRY_INTERVAL);
    esp_restart();
  }else{
    #ifdef debug
    Serial.println("ESP_Now init OK...");
    #endif

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
  esp_task_wdt_reset();
  }

void loop() {


  #ifdef BenneSat
  uint8_t compteur=0;
  for(int i=0;i<5;i++){
    voltage=io_1.digitalRead(LC);
    if(!voltage){
      compteur++;
    }
  }
  if(compteur>2){
    bus.Nsensor[LC]='1';
    #ifdef debug
    Serial.println("Levée du caisson ON");
    #endif
  }else{
    #ifdef debug
    Serial.println("Levée du caisson OFF");
    #endif
    bus.Nsensor[LC]='0';
  }

  compteur=0;
  for(int i=0;i<5;i++){
    voltage=io_2.digitalRead(CLC);
    if(!voltage){
      compteur++;
    }
  }
  if(compteur>2){
    bus.Nsensor[CLC+16]='1';
    #ifdef debug
    Serial.println("Cycle LC ON");
    #endif
  }else{
    bus.Nsensor[CLC+16]='0';
    #ifdef debug
    Serial.println("Cycle LC OFF");
    #endif
  }
  

  bool compare=compareArrays(refPD,bus.Nsensor,cSize);
  if(!compare){
    copyArrays(refPD,bus.Nsensor,cSize);
    send=true;
  }
  if(send){
    send=false;
    #ifdef debug
    Serial.println("\nSending Data\n");
    #endif
    for(int i=0;i<3;i++){
      sendData();
      delay(2500);
      if(SendOK){
        blinkLed(500,25);
        delay(2000);
        break;
      }
    }
  }
  #ifdef debug
  Serial.println("*******************************");
  #endif
  delay(1500);
  #endif


  #ifdef BOM
  uint8_t compteur=0;
  for(int i=0;i<5;i++){
    voltage=io_1.digitalRead(OPA);
    if(!voltage){
      compteur++;
    }
  }
  if(compteur>2){
    bus.Nsensor[OPA]='1';
    #ifdef debug
    Serial.println("Ouverture de la porte arrière ON");
    #endif
  }else{
    #ifdef debug
    Serial.println("Ouverture de la porte arrière OFF");
    #endif
    bus.Nsensor[OPA]='0';
  }

  compteur=0;
  for(int i=0;i<5;i++){
    voltage=io_2.digitalRead(CLC);
    if(!voltage){
      compteur++;
    }
  }
  if(compteur>2){
    bus.Nsensor[CLC+16]='1';
    #ifdef debug
    Serial.println("Cycle LC ON");
    #endif
  }else{
    bus.Nsensor[CLC+16]='0';
    #ifdef debug
    Serial.println("Cycle LC OFF");
    #endif
  }


  compteur=0;
  for(int i=0;i<5;i++){
    voltage=io_2.digitalRead(COM);
    if(!voltage){
      compteur++;
    }
  }
  if(compteur>2){
    bus.Nsensor[COM+16]='1';
    #ifdef debug
    Serial.println("Compactation ON");
    #endif

  }else{
    bus.Nsensor[COM+16]='0';
    #ifdef debug
    Serial.println("Compactation OFF\n");
    #endif
  }

  

  bool compare=compareArrays(refPD,bus.Nsensor,cSize);
  if(!compare){
    copyArrays(refPD,bus.Nsensor,cSize);
    send=true;
  }
  if(send){
    send=false;
    #ifdef debug
    Serial.println("\nSending Data\n");
    #endif
    for(int i=0;i<3;i++){
      sendData();
      delay(2500);
      if(SendOK){
        blinkLed(500,25);
        delay(2000);
        break;
      }
    }
  }
  #ifdef debug
  Serial.println("*******************************");
  #endif
  delay(1500);
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

  bool check=false;

  check=CheckVoltage(BD,1);
  if(check) Serial.println("Brosse Droite ON"); else Serial.println("Brosse Droite OFF");

  check=CheckVoltage(AD,1);
  if(check) Serial.println("Aspirateur Droit ON"); else Serial.println("Aspirateur Droit OFF");
  
    check=CheckVoltage(BG,1);
  if(check) Serial.println("Brosse Gauche ON"); else Serial.println("Brosse Gauche OFF");

    check=CheckVoltage(AG,1);
  if(check) Serial.println("Aspirateur Gauche ON"); else Serial.println("Aspirateur Gauche OFF");

  check=CheckVoltage(BC,1);
  if(check) Serial.println("Brosse Centrale ON"); else Serial.println("Brosse Centrale OFF");

  check=CheckVoltage(AC,1);
  if(check) Serial.println("Aspirateur Centrale ON"); else Serial.println("Aspirateur Centrale OFF");

  check=CheckVoltage(LC,1);
  if(check) Serial.println("levée caisson ON"); else Serial.println("levée caisson OFF");

  check=CheckVoltage(AMA,1);
  if(check) Serial.println("Aspirateur Manuel Arriere ON"); else Serial.println("Aspirateur Manuel Arriere OFF");

  check=CheckVoltage(AP,1);
  if(check) Serial.println("Activation pompe ON"); else Serial.println("Activation pompe OFF");

  check=CheckVoltage(OPA,1);
  if(check) Serial.println("Ouverture de la Porte arrière ON"); else Serial.println("Ouverture de la Porte arrière OFF");

  #ifdef debug
  Serial.println("***************Data*****************");
  for (int i=0; i<cSize; ++i){
    if(i==16) Serial.print("   ");
    Serial.print(bus.Nsensor[i]);
  }
  Serial.println("\n************************************");
  #endif



  bool compare=compareArrays(refPD,bus.Nsensor,cSize);
  if(!compare){
    copyArrays(refPD,bus.Nsensor,cSize);
    send=true;
  }
  if(send){
    send=false;
    #ifdef debug
    Serial.println("\nSending Data\n");
    #endif
    for(int i=0;i<3;i++){
      sendData();
      delay(2500);
      if(SendOK){
        blinkLed(500,25);
        delay(2000);
        break;
      }
    }
  }
  #ifdef debug
  Serial.println("*******************************");
  #endif
  delay(1500);
  


  #endif

#ifdef test

  voltage=io_1.digitalRead(BD);
  if(!voltage) bus.Nsensor[BD]='1'; else bus.Nsensor[BD]='0';
  
  voltage=io_1.digitalRead(AD);
  if(!voltage) bus.Nsensor[AD]='1'; else bus.Nsensor[AD]='0';

  voltage=io_1.digitalRead(BG);
  if(!voltage) bus.Nsensor[BG]='1'; else bus.Nsensor[BG]='0';

  voltage=io_1.digitalRead(AG);
  if(!voltage) bus.Nsensor[AG]='1'; else bus.Nsensor[AG]='0';

  voltage=io_1.digitalRead(BC);
  if(!voltage) bus.Nsensor[BC]='1'; else bus.Nsensor[BC]='0';

  voltage=io_1.digitalRead(AC);
  if(!voltage) bus.Nsensor[AC]='1'; else bus.Nsensor[AC]='0';

  voltage=io_1.digitalRead(LC);
  if(!voltage) bus.Nsensor[LC]='1'; else bus.Nsensor[LC]='0';

  voltage=io_1.digitalRead(AMA);
  if(!voltage) bus.Nsensor[AMA]='1'; else bus.Nsensor[AMA]='0';

  voltage=io_1.digitalRead(AP);
  if(!voltage) bus.Nsensor[AP]='1'; else bus.Nsensor[AP]='0';

  voltage=io_1.digitalRead(ARD);
  if(!voltage) bus.Nsensor[ARD]='1'; else bus.Nsensor[ARD]='0';

  voltage=io_1.digitalRead(K1);
  if(!voltage) bus.Nsensor[K1]='1'; else bus.Nsensor[K1]='0';

  voltage=io_1.digitalRead(K2);
  if(!voltage) bus.Nsensor[K2]='1'; else bus.Nsensor[K2]='0';

  voltage=io_1.digitalRead(LAB);
  if(!voltage) bus.Nsensor[LAB]='1'; else bus.Nsensor[LAB]='0';

  voltage=io_1.digitalRead(VES);
  if(!voltage) bus.Nsensor[VES]='1'; else bus.Nsensor[VES]='0';

  voltage=io_1.digitalRead(OPA);
  if(!voltage) bus.Nsensor[OPA]='1'; else bus.Nsensor[OPA]='0';


  /**************************SX02************************************/

  voltage=io_2.digitalRead(CLC);
  if(!voltage) bus.Nsensor[CLC+16]='1'; else bus.Nsensor[CLC+16]='0';

  voltage=io_2.digitalRead(COM);
  if(!voltage) bus.Nsensor[COM+16]='1'; else bus.Nsensor[COM+16]='0';

  voltage=io_2.digitalRead(PT);
  if(!voltage) bus.Nsensor[PT+16]='1'; else bus.Nsensor[PT+16]='0';

  

  


  #ifdef debug
  for (int i=0; i<cSize; ++i){
    if(i==16) Serial.print("   ");
    Serial.print(bus.Nsensor[i]);
  }
  Serial.println("\n************************************");
  #endif

  delay(2500);
#endif



#ifdef test1

  if(true){
    send=false;
    #ifdef debug
    Serial.println("\nSending Data\n");
    #endif
    for(int i=0;i<3;i++){
      sendData();
      delay(2500);
      if(SendOK){
        blinkLed(500,25);
        delay(2000);
        break;
      }
    }
  }
  delay(5000);

  Serial.println("****************************");
#endif

  esp_task_wdt_reset();

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

bool CheckVoltage(uint8_t pos, uint8_t sx){
  
  uint8_t compteur=0;
  // check if SX 1 or SX2
  if(sx==1){
    for(int i=0;i<7;i++){
      voltage=io_1.digitalRead(pos);
      if(!voltage){
        compteur++;
      }
    }

    if(compteur>3){
      bus.Nsensor[pos]='1';
      return true;
    }else{
      bus.Nsensor[pos]='0';
      return false;
    }
  }

  if(sx==2){
    for(int i=0;i<7;i++){
      voltage=io_2.digitalRead(pos);
      if(!voltage){
        compteur++;
      }
    }
    if(compteur>3){
      bus.Nsensor[pos+16]='1';
      return true;
    }else{
      bus.Nsensor[pos+16]='0';
      return false;
    }
  }
  return false;
}