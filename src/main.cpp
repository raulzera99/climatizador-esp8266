#include <Arduino.h>
#include "DHT.h"

#define DHTTYPE DHT11                     // DHT version
#define DHTPIN D1  //D1                       // SU - digital input
#define BOIA D8   //D8               // SR - digital input
#define BTN_ON_OFF D0//D2             // BO - digital input
#define VENTILADOR LED_BUILTIN            // VD - digital output
#define VALVULA_RESERVATORIO D9       // VR - digital output
#define BOMBA_DAGUA D10                   // BA - digital output
#define BTN_MODO D7 //D7              // BM - digital input
//#define BTN_INC                           // BINC - digital input
//#define BTN_DEC                           // BDEC - digital input
#define PIN_B D6 //D6                 // PinB - RotaryEncoder
#define PIN_A D5 //D5                 // PinA - RotaryEncoder
#define RED D4
#define GREEN D3
#define BLUE D2

DHT dht(DHTPIN, DHTTYPE);

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
  analogWrite(RED, 0);
  analogWrite(GREEN, 0);
  switch (estado){
    case 1: //VENTILADOR(desligado), BOMBA D'ÁGUA(desligada) e VALVULA RESERVATORIO(fechada)
      digitalWrite(VENTILADOR, LOW);
      digitalWrite(BOMBA_DAGUA, LOW);
      digitalWrite(VALVULA_RESERVATORIO, LOW);
      analogWrite(RED, 255);
      break;

    case 2://VENTILADOR(ligado), BOMBA D'ÁGUA(desligada) e VALVULA RESERVATORIO(fechada)
      digitalWrite(VENTILADOR, HIGH);
      digitalWrite(BOMBA_DAGUA, LOW);
      digitalWrite(VALVULA_RESERVATORIO, LOW);
      analogWrite(RED, 255);
      break;

    case 3://VENTILADOR(ligado), BOMBA D'ÁGUA(desligada) e VALVULA RESERVATORIO(aberta)
      digitalWrite(VENTILADOR, HIGH);
      digitalWrite(BOMBA_DAGUA, LOW);
      digitalWrite(VALVULA_RESERVATORIO, HIGH);
      analogWrite(GREEN, 255);
      break;

    case 4://VENTILADOR(ligado), BOMBA D'ÁGUA(ligada) e VALVULA RESERVATORIO(fechada)
      digitalWrite(VENTILADOR, HIGH);
      digitalWrite(BOMBA_DAGUA, HIGH);
      digitalWrite(VALVULA_RESERVATORIO, LOW);
      analogWrite(RED, 255);
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
  delay(2000);
  return h;
}

//ISR - Interrupt Service Routine
void IRAM_ATTR setBtnOnOff(){
  onOff = !onOff;
}

//ISR - Interrupt Service Routine
void IRAM_ATTR readRotaryEncoder(){
  pinAvalue = digitalRead(PIN_A);

  if((pinAlastValue == LOW) && (pinAvalue == HIGH)){
    if(digitalRead(PIN_B) == HIGH){
      Serial.println("horário");
    }
    else{
      Serial.println("anti horário");
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
    }
    else if (modo == ajuste_velocidade)
    {
      Serial.println("MODO = [ajuste_velocidade]");
    }
  }
}

void showEstadoActive(boolean showSerialMonitor){
  if(showSerialMonitor){
    if(estado == A){
      Serial.println("VENTILADOR = [desligado]");
      Serial.println("BOMBA D'ÁGUA = [desligada]");
      Serial.println("VÁLVULA DO RESERVATÓRIO = [fechada]");
    }
    else if(estado == B){
      Serial.println("VENTILADOR = [ligado]");
      Serial.println("BOMBA D'ÁGUA = [desligada]");
      Serial.println("VÁLVULA DO RESERVATÓRIO = [fechada]");
    }
    else if(estado == C){
      Serial.println("VENTILADOR = [ligado]");
      Serial.println("BOMBA D'ÁGUA = [desligada]");
      Serial.println("VÁLVULA DO RESERVATÓRIO = [aberta]");
    }
    else if(estado == D){
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

// put your setup code here, to run once:
void setup() {
  Serial.begin(9600);
  dht.begin();

  pinMode(BTN_ON_OFF, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_ON_OFF), setBtnOnOff, FALLING);
  
  pinMode(BTN_MODO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_MODO), handleMode, FALLING);

  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_B), readRotaryEncoder, CHANGE);

  pinMode(BOIA, INPUT);
  pinMode(VENTILADOR, OUTPUT);
  pinMode(VALVULA_RESERVATORIO, OUTPUT);
  pinMode(BOMBA_DAGUA, OUTPUT);
  //pinMode(BTN_INC, INPUT);
  //pinMode(BTN_DEC, INPUT);

  estado = A;//VENTILADOR(desligado), BOMBA D'ÁGUA(desligada) e VALVULA RESERVATORIO(fechada)

  pinAvalue = LOW;
  pinAlastValue = pinAvalue;

}

boolean showSerialMonitor = true;

// put your main code here, to run repeatedly:
void loop() {
                                     handleState(estado);

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
  
  showEstadoActive(showSerialMonitor);
  showBtnOnOffActive(showSerialMonitor);
  showModoActive(showSerialMonitor);
  
  
 if(showSerialMonitor){
    Serial.println("------------------------------------------");
  } 
}