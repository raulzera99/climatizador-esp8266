#include <Arduino.h>
#include "DHT.h"
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

#define DHTTYPE DHT11                 // DHT version
#define DHTPIN D1                     // SU - digital input
#define BOIA D8                       // SR - digital input
#define BTN_ON_OFF D2                 // BO - digital input
#define BOMBA_DAGUA D0                // BA - digital output
#define VALVULA_RESERVATORIO 3        // VR - digital output
#define VENTILADOR D4                 // VD - digital output
#define BTN_MODO D7                   // BM - digital input
#define PIN_B D6                      // PinB - RotaryEncoder
#define PIN_A D5                      // PinA - RotaryEncoder

DHT dht(DHTPIN, DHTTYPE);

// MQTT Credentials
//const char* ssid = "REP - 2.4Gh";
//const char* password = "3B9E4F45";
const char* ssid = "IoT";
const char* password = "ifspifsp";
const char* mqttServer = "broker.hivemq.com";
const char* mqttUsername = "";
const char* mqttPassword = "";
const char* clientID = "";
const char* topic = "data";
String msgStr = ""; //MQTT message buffer

//Setting up WiFi and MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);

//parameters for using non-blocking delay
unsigned long previousMillis = 0;
const long interval = 3000;

volatile int velocidade_ventilador = 100; //VV
volatile int setpoint_umidade = 60;      //SPU

int counter = 0;
int pinAlastValue;
volatile int pinAvalue;
volatile bool onOff = false;
float humidity;

enum State {
  A = 1,    //VENTILADOR(desligado), BOMBA D'ÁGUA(desligada) e VALVULA RESERVATORIO(fechada)
  B = 2,    //VENTILADOR(ligado), BOMBA D'ÁGUA(desligada) e VALVULA RESERVATORIO(fechada)
  C = 3,    //VENTILADOR(ligado), BOMBA D'ÁGUA(desligada) e VALVULA RESERVATORIO(aberta)
  D = 4     //VENTILADOR(ligado), BOMBA D'ÁGUA(ligada) e VALVULA RESERVATORIO(fechada)
};

State estado;

enum Mode{
  operacao = 1,
  ajuste_umidade = 2,
  ajuste_velocidade = 3
};

volatile Mode modo = operacao;

//ISR - Interrupt Service Routine
void IRAM_ATTR handleMode(){
  if(modo == operacao){
    modo = ajuste_umidade;
  }
  else if (modo == ajuste_umidade){
    modo = ajuste_velocidade;
  }
  else if (modo == ajuste_velocidade){
    modo = operacao;
  }
}

void handleState(State estado){
  int dutyCicle = (1023*velocidade_ventilador)/100;
  switch (estado){
    case 1: //VENTILADOR(desligado), BOMBA D'ÁGUA(desligada) e VALVULA RESERVATORIO(fechada)
      analogWrite(VENTILADOR, 0);
      //digitalWrite(VENTILADOR, LOW);
      digitalWrite(BOMBA_DAGUA, LOW);
      digitalWrite(VALVULA_RESERVATORIO, LOW);                                              
      break;

    case 2://VENTILADOR(ligado), BOMBA D'ÁGUA(desligada) e VALVULA RESERVATORIO(fechada)
      analogWrite(VENTILADOR, dutyCicle);
      //digitalWrite(VENTILADOR, HIGH);
      digitalWrite(BOMBA_DAGUA, LOW);
      digitalWrite(VALVULA_RESERVATORIO, LOW);
      break;

    case 3://VENTILADOR(ligado), BOMBA D'ÁGUA(desligada) e VALVULA RESERVATORIO(aberta)
      analogWrite(VENTILADOR, dutyCicle);
      //digitalWrite(VENTILADOR, HIGH);
      digitalWrite(BOMBA_DAGUA, LOW);
      digitalWrite(VALVULA_RESERVATORIO, HIGH);
      break;

    case 4://VENTILADOR(ligado), BOMBA D'ÁGUA(ligada) e VALVULA RESERVATORIO(fechada)
      analogWrite(VENTILADOR, dutyCicle);
      //digitalWrite(VENTILADOR, HIGH);
      digitalWrite(BOMBA_DAGUA, HIGH);
      digitalWrite(VALVULA_RESERVATORIO, LOW);
      break;
  }
} 

float readHumidity(boolean showSerialMonitor){
  // A leitura da temperatura e umidade pode levar 250ms!
  // O atraso do sensor pode chegar a 2 segundos.
  float h = dht.readHumidity();

  // testa se retorno é valido, caso contrário algo está errado.
  if(showSerialMonitor){
    if (isnan(h)) 
    {
      Serial.println("Failed to read from DHT");
    } 
    else
    {
      Serial.print("Umidade: ");
      Serial.print(h);
      Serial.println(" %");
    }
  }
  //delay(2000);
  return h;
}

//ISR - Interrupt Service Routine
void IRAM_ATTR setBtnOnOff(){
  onOff = !onOff;

  msgStr = onOff;
  byte arrSize = msgStr.length() + 1;
  char msg[arrSize];
  msgStr.toCharArray(msg, arrSize);
  client.publish("OnOff", msg);
  //Serial.println(msgStr);
  msgStr = "";
}

//ISR - Interrupt Service Routine
void IRAM_ATTR readRotaryEncoder(){

  pinAvalue = digitalRead(PIN_A);

  if((pinAlastValue == LOW) && (pinAvalue == HIGH)){

    if(modo == ajuste_umidade){
      if(digitalRead(PIN_B) == HIGH){
        setpoint_umidade +=1;
        
        if(setpoint_umidade > 70){
          setpoint_umidade = 70;
        }
      }
      else{
        setpoint_umidade -=1;
        
        if(setpoint_umidade < 40){
          setpoint_umidade = 40;
        }
      }

      msgStr = String(setpoint_umidade);
      byte arrSize = msgStr.length() + 1;
      char msg[arrSize];
      msgStr.toCharArray(msg, arrSize);
      client.publish("SPU", msg);
      //Serial.println(msgStr);
      msgStr = "";
    }
    else if (modo == ajuste_velocidade){

      if(digitalRead(PIN_B) == HIGH){
        velocidade_ventilador +=1;
        
        if(velocidade_ventilador > 100){
          velocidade_ventilador = 100;
        }
        if(velocidade_ventilador < 30){
          velocidade_ventilador = 30;
        }
      }
      else{
        velocidade_ventilador -=1;
        
        if(velocidade_ventilador < 30){
          velocidade_ventilador = 0;
        }
      }

      //Serial.println((velocidade_ventilador*1023)/100);
      if(estado != A){
        analogWrite(VENTILADOR, (velocidade_ventilador*1023)/100);
      }

      msgStr = String(velocidade_ventilador);
      byte arrSize = msgStr.length() + 1;
      char msg[arrSize];
      msgStr.toCharArray(msg, arrSize);
      client.publish("VV", msg);
      //Serial.println(msgStr);
      msgStr = "";
    }
  }
}

//debug
void showModoActive(boolean showSerialMonitor){
  if(showSerialMonitor){
    if(modo == operacao){
      Serial.println("MODO = [operação]");
    }
    else if (modo == ajuste_umidade)
    {
      Serial.println("MODO = [ajuste_umidade]");
      Serial.println();
      Serial.print("SetPoint Umidade: ");
      Serial.println(setpoint_umidade);
    }
    else if (modo == ajuste_velocidade)
    {
      Serial.println("MODO = [ajuste_velocidade]");
      Serial.println();
      Serial.print("Velocidade Ventilador: ");
      Serial.println(velocidade_ventilador);
    }
  }
}

void showEstadoActive(boolean showSerialMonitor){
  if(showSerialMonitor){
    if(estado == A){
      Serial.println("ESTADO -> AAAAA");
      Serial.println("VENTILADOR = [desligado]");
      Serial.println("BOMBA D'ÁGUA = [desligada]");
      Serial.println("VÁLVULA DO RESERVATÓRIO = [fechada]");
    }
    else if(estado == B){
      Serial.println("ESTADO -> BBBBB");
      Serial.println("VENTILADOR = [ligado]");
      Serial.println("BOMBA D'ÁGUA = [desligada]");
      Serial.println("VÁLVULA DO RESERVATÓRIO = [fechada]");
    }
    else if(estado == C){
      Serial.println("ESTADO -> CCCCC");
      Serial.println("VENTILADOR = [ligado]");
      Serial.println("BOMBA D'ÁGUA = [desligada]");
      Serial.println("VÁLVULA DO RESERVATÓRIO = [aberta]");
    }
    else if(estado == D){
      Serial.println("ESTADO -> DDDDD");
      Serial.println("VENTILADOR = [ligado]");
      Serial.println("BOMBA D'ÁGUA = [ligada]");
      Serial.println("VÁLVULA DO RESERVATÓRIO = [fechada]");
    }
  }
}

void showBtnOnOffActive(boolean showSerialMonitor){
  if(showSerialMonitor){
    if(onOff){
      Serial.println("CLIMATIZADOR = [ligado]");
    }
    else{
      Serial.println("CLIMATIZADOR = [desligado]");
    }
  }
}

void showBoiaActive(boolean showSerialMonitor){
  if(showSerialMonitor){
    if(digitalRead(BOIA)){
      Serial.println("SENSOR RESERVATÓRIO (BÓIA) = [com água]");
    }
    else{
      Serial.println("SENSOR RESERVATÓRIO (BÓIA) = [sem água]");
    }
  }
}

//WiFi Settings
void setup_wifi(){
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  //Connection to WiFi
  WiFi.begin(ssid, password);
  //Set static ip
  //WiFi.config(ip, gateway, subnet);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}


void reconnect(){
  while(!client.connected()){
    if(client.connect(clientID, mqttUsername, mqttPassword)){
      Serial.println("MQTT Connected");
      client.subscribe("appInventor/VV");
      client.subscribe("appInventor/SPU");
      client.subscribe("appInventor/OnOff");
      client.subscribe("appInventor/modo");
      Serial.println("Topics Subscribed");
    }
    else{
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds");
      delay(5000);
    }
  }
}

//Subscribe callback function
void callback(char*topic, byte* payload, int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  String msg;

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    msg+= (char)payload[i];
  }

  Serial.println();
  Serial.print("Message size :");
  Serial.println(length);
  Serial.println();
  Serial.println("-----------------------");
  Serial.println(msg);
  
  if(String(topic).equals("appInventor/VV")){
    if(msg == "+"){
      velocidade_ventilador++;
      if(velocidade_ventilador >= 100){
        velocidade_ventilador = 100;
      }
    }
    else if ( msg == "-"){
      velocidade_ventilador--;
      if(velocidade_ventilador == 30 || velocidade_ventilador < 0){
        velocidade_ventilador = 0;
      }
    }
    
    msgStr = String(velocidade_ventilador);
    byte arrSize = msgStr.length() + 1;
    char msg[arrSize];
    msgStr.toCharArray(msg, arrSize);
    client.publish("VV", msg);
    msgStr = "";
    Serial.println("Velocidade do ventilador atualizada: " + String(velocidade_ventilador) + " %");
  }
  else if(String(topic).equals("appInventor/SPU")){
    if(msg == "+"){
      setpoint_umidade++;
      if(setpoint_umidade >=70){
        setpoint_umidade=70;
      }
    }
    else if ( msg == "-"){
      setpoint_umidade--;
      if(setpoint_umidade < 40){
        setpoint_umidade = 40;
      }
    }
    msgStr = String(setpoint_umidade);
    byte arrSize = msgStr.length() + 1;
    char msg[arrSize];
    msgStr.toCharArray(msg, arrSize);
    client.publish("SPU", msg);
    msgStr = "";
    Serial.println("Setpoint de umidade atualizado: "+String(setpoint_umidade)+" %");
  }
  else if (String(topic).equals("appInventor/modo")){
    handleMode();
  }
  else if (String(topic).equals("appInventor/OnOff")){
    setBtnOnOff();
  }
}



// put your setup code here, to run once:
void setup() {
  Serial.begin(115200);

  //DHT11 Settings
  dht.begin();

  //System Settings
  pinMode(BTN_ON_OFF, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_ON_OFF), setBtnOnOff, FALLING);
  
  pinMode(BTN_MODO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_MODO), handleMode, FALLING);

  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_B), readRotaryEncoder, CHANGE);

  pinMode(BOIA, INPUT);

  pinMode(VENTILADOR, OUTPUT);
  analogWriteRange(1023);

  pinMode(VALVULA_RESERVATORIO, OUTPUT);
  pinMode(BOMBA_DAGUA, OUTPUT);

  //WiFi Settings
  setup_wifi();
  client.setServer(mqttServer, 1883); // Setting MQTT Server
  client.setCallback(callback); // Function which will be called when message is received

  estado = A;//VENTILADOR(desligado), BOMBA D'ÁGUA(desligada) e VALVULA RESERVATORIO(fechada)

  pinAvalue = LOW;
  pinAlastValue = pinAvalue;

}

boolean showSerialMonitor = true;

// put your main code here, to run repeatedly:
void loop() {
  if (!client.connected()) { //if client is not connected
    reconnect(); //try to reconnect
  }
  client.loop();
 
  unsigned long currentMillis = millis(); //read current time
  
  //if current time - last time > 5 sec
  if (currentMillis - previousMillis >= interval) { 
    previousMillis = currentMillis;

    //System output settings
    handleState(estado);

    //SerialMonitor settings
    showBtnOnOffActive(showSerialMonitor);
    showEstadoActive(showSerialMonitor);
    showBoiaActive(showSerialMonitor);
    showModoActive(showSerialMonitor);

    humidity = readHumidity(showSerialMonitor);

    if(onOff){
      if(estado == A){//VENTILADOR(desligado), BOMBA D'ÁGUA(desligada) e VALVULA RESERVATORIO(fechada)
        if(humidity >= (setpoint_umidade)){
          estado = B;
        }
        else if (digitalRead(BOIA) == LOW){
          estado = C;
        }
        else{
          estado = D;
        }
      }
      else if (estado == B){//VENTILADOR(ligado), BOMBA D'ÁGUA(desligada) e VALVULA RESERVATORIO(fechada)
        if(humidity < (setpoint_umidade - 2)){
          estado = C;
        }
      }
      else if(estado == C){//VENTILADOR(ligado), BOMBA D'ÁGUA(desligada) e VALVULA RESERVATORIO(aberta)
        if(digitalRead(BOIA) == HIGH){
          estado = D;
        }
      }
      else if (estado == D){//VENTILADOR(ligado), BOMBA D'ÁGUA(ligada) e VALVULA RESERVATORIO(fechada)
        if(humidity > (setpoint_umidade + 2)){
          estado = B;
        }
        else if(digitalRead(BOIA) == LOW){
          estado = C;
        }
      }
      
    }
    else{
      estado = A;
    }
    
    //Joining messages in message buffer
    msgStr = String(int(humidity)) +" %,"
    +String(estado) + ","
    +String(digitalRead(BOIA)) + ","
    +String(modo);
    byte arrSize = msgStr.length() + 1;
    char msg[arrSize];
    msgStr.toCharArray(msg, arrSize);
    //Publishing a message on topic
    client.publish("data", msg);
    if(showSerialMonitor){
      Serial.print("PUBLISH DATA:");
      Serial.println(msgStr);
    }
    msgStr = "";

    if(showSerialMonitor){
      Serial.println("========================================");
    }
  }

}