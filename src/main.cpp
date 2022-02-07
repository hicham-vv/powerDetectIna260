#include <Arduino.h>
#include <esp_now.h>
#include <Adafruit_INA260.h>

#ifdef ESP32
  #include <WiFi.h>
#endif


#define INA260_I2CADDR_3 0x41 // INA260 I2c Adress 0x41 A1=0/A0=1
#define INA260_I2CADDR_2 0x40 // INA260 default i2c address 0x40 A1=0/A0=0
#define INA260_I2CADDR_1 0x44 // INA260 I2c Adress 0x44 A1=1/A0=0




#define debug
#define BusVoltage




// #define CiterneMAN  // Pour les Citernes MAN de Tanger

#define MicroBalayeuse


uint8_t receiverMAC[] = {0x4c, 0x11, 0xae, 0x9d, 0x68, 0xec}; // TracCar MAC Adress




// 4c:11:ae:9d:6d:ec



Adafruit_INA260 ina260_1 = Adafruit_INA260();
Adafruit_INA260 ina260_2 = Adafruit_INA260();
Adafruit_INA260 ina260_3 = Adafruit_INA260();



#define led_esp 2

#define RETRY_INTERVAL 5000
#define SEND_INTERVAL 1000 
#define N 10 // nombre de tags // il faut initialiser les valeurs du tableau winnerRSSI[N] par -255  à chaque fois cette valeur est changée 

#define TensionSeuil 6

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
}message;
message bus; // créer une structure message nommé bus


int voltage=0;
int current=0;
int power=0;


void sendData() {
  sentStartTime = micros();
  esp_now_send(receiverMAC,(uint8_t *) &bus, sizeof(bus)); // NULL means send to all peers
}


void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  if(status == ESP_NOW_SEND_SUCCESS){
    Serial.println("points sents");
  }else{
    Serial.println("points not  sents");

  }
}

void setup() {
  #ifdef debug
  Serial.begin(115200);
  while(!Serial);
  #endif

  pinMode(led_esp,OUTPUT);

  #ifndef MicroBalayeuse

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

  #endif
  

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

  #ifndef MicroBalayeuse

  current=ina260_1.readCurrent();
  Serial.print("Current 1 =");Serial.println(current);
  voltage=ina260_1.readBusVoltage();
  Serial.print("voltage 1 =");Serial.println(voltage/1000);
  power=ina260_1.readPower();
  Serial.print("power 1 =");Serial.println(power);

  voltage=voltage/1000;

  #ifdef BusVoltage
  if(voltage>=TensionSeuil){
    bus.PD1='1';
  }else{
    bus.PD1='0';
  }
  #endif
  #ifdef BusCurrent
  bus.PD1=current;
  #endif
  #ifdef BusPower
  bus.PD1=power;
  #endif

  current=ina260_2.readCurrent();
  Serial.print("Current 2 =");Serial.println(current);
  voltage=ina260_2.readBusVoltage();
  Serial.print("voltage 2 =");Serial.println(voltage/1000);
  power=ina260_2.readPower();
  Serial.print("power 2 =");Serial.println(power);

  voltage=voltage/1000;

  #ifdef BusVoltage
  if(voltage>=TensionSeuil){
    bus.PD2='1';
  }else{
    bus.PD2='0';
  }
  #endif
  #ifdef BusCurrent
  bus.PD2=current;
  #endif
  #ifdef BusPower
  bus.PD2=power;
  #endif


  current=ina260_3.readCurrent();
  Serial.print("Current 3 =");Serial.println(current);
  voltage=ina260_3.readBusVoltage();
  Serial.print("voltage 3 =");Serial.println(voltage/1000);
  power=ina260_3.readPower();
  Serial.print("power 3 =");Serial.println(power);

  voltage=voltage/1000;

  #ifdef BusVoltage
  if(voltage>=TensionSeuil){
    bus.PD3='1';
  }else{
    bus.PD3='0';
  }
  #endif
  #ifdef BusCurrent
  bus.PD3=current;
  #endif
  #ifdef BusPower
  bus.PD3=power;
  #endif


  #ifdef CiterneMAN
  if(bus.PD3=='1'){
    bus.PD2='0';
  }else{
    bus.PD2='1';
  }
  #endif
  #endif
  #ifdef MicroBalayeuse

  bus.PD1='0';
  bus.PD2='1';
  bus.PD3='1';

  #endif


  sendData();
  delay(5000);
  #ifdef MicroBalayeuse
  delay(15000);
  #endif
}