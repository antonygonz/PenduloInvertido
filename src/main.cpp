#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <WiFi.h>
#include <Wire.h>
#include <EEPROM.h>
#include <FastAccelStepper.h>
#include <Wire.h>
#include "AS5600.h"
#define AS5600_ADDRESS 0x36
// As in StepperDemo for Motor 1 on ESP32
#define enablePinStepper 27
#define dirPinStepper 26
#define stepPinStepper 25
#define ONOFF 23
// Variables de la comunicacion
String inputString = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
int contadorFiltro=0;

//variables del PID
float Setpoint = 35;       // Setpoint
float Kp = 4.6726; float Tao_I = 67.70;// Constantes de PID
float Kd=0;

float PID_error = 0;
float previous_error = 0;
float PID_value = 0;
float Error_INT = 0;
//Variables del tiempo
unsigned long Tiempo_Previo_loop1=0;
unsigned long Tiempo_Actual_loop1=0;
unsigned long Tiempo_previo = 0; //variable para medir tiempo inicial
unsigned long Tiempo_actual = 0; //variable para medir tiempo actual
int Read_Delay = 1000;     // Periodo de muestreo en milisegundos

// Variables para graficacion
bool graficar=false;
// Variables para Segundo nucleo
bool TareaCorriendo=false;
//Variables para el Control y sensores
float angulo=0;
float rawAngle=0;
float aceleracion=0;

//declaracion de librerias
//aceleration stepper
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
//AS5600
AS5600 as5600;
/*
float LeerAngulo(){
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(0x0C);
  Wire.endTransmission();

  // Read the 2 bytes of data from the AS5600
  Wire.requestFrom(AS5600_ADDRESS, 2);
  byte msb = Wire.read();
  byte lsb = Wire.read();

  // Combine the MSB and LSB to obtain the raw angle value
  int rawAngle = (msb << 8) | lsb;

  // Convert the raw angle to degrees
  float angle = rawAngle * 0.08789; // Each LSB represents 0.08789 degrees
  return angle;
}*/

TaskHandle_t Task1;
void loop2(void *parameter){
  
  for(;;){
    
      } //Serial.println("sEGUNDO CORE");
      vTaskDelay(1); //NECESARIO SI LA PRIORIDAD DE LA TAREA NO ES 0//resulta que de ahuevo se ocupa
}
  

//
unsigned long lastMsg=0;
void setup() {
  delay(100);
  Serial.begin(115200);

  Setpoint=EEPROM.get(0,Setpoint);
  Kp=EEPROM.get(10,Kp);
  Tao_I=EEPROM.get(20,Tao_I);
  Kd=EEPROM.get(30,Kd);

  inputString.reserve(200);//recerba hasta 200 caracteres en la memoria para los strings de entrada
  pinMode(LED_BUILTIN, OUTPUT);//inicializa el led
  
  while (!Serial);
  pinMode(ONOFF,INPUT_PULLDOWN);
  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(true);
    // If auto enable/disable need delays, just add (one or both):
    // stepper->setDelayToEnable(50);
    // stepper->setDelayToDisable(1000);
    //stepper->setSpeedInUs(4000);  // the parameter is us/step !!!
    stepper->setSpeedInTicks(200);
    stepper->setAcceleration(100);
    //stepper->move(1000);
  }
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);
   Wire.begin();
  //  as5600.begin(14,15);
  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  Serial.println(as5600.getAddress());
  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
}

void loop() {
  StaticJsonDocument<80> graph;
  char Salida_grafica[80];
  
  Tiempo_Actual_loop1=micros();
  if (stringComplete) {//entra si hay un string completo
    Serial.println(inputString);//Escribe en la consola el ultimo string
    if (inputString.equals("ON")){
      digitalWrite(LED_BUILTIN, HIGH);  
    }else if (inputString=="OFF"){
      digitalWrite(LED_BUILTIN, LOW);
    }else if (inputString=="STEPUP"){
      stepper->moveByAcceleration(500,true);
    }else if (inputString=="STEPDOWN"){
      stepper->moveByAcceleration(-500,true);
    }else if (inputString=="BRAKE"){
      stepper->forceStop();
    }else if (inputString=="SPEED"){
      Serial.print("VELOCIDAD ω = ");
      Serial.println(as5600.getAngularSpeed(AS5600_MODE_RPM));
    }else if (inputString=="ANGLE"){
      rawAngle=as5600.readAngle();
      angulo=rawAngle* 0.08789;
      Serial.print("ANGULO = ");
      Serial.println(angulo);
    }else if (inputString=="OLSTART"){
      if (TareaCorriendo)
      {
        vTaskSuspend(Task1);
      }graficar=true;
      Serial.println("runing");
      xTaskCreatePinnedToCore(loop2,"Task_1",2000,NULL,1,/*intenta cambiar prioridad a 0, original 1*/&Task1,0);
    }else if (inputString=="STOP"){
      if (TareaCorriendo)
      {
        vTaskSuspend(Task1);
        TareaCorriendo=false;
      }graficar=false;
    }else if (inputString=="READPID"){
      Setpoint=EEPROM.get(0,Setpoint);
      Kp=EEPROM.get(10,Kp);
      Tao_I=EEPROM.get(20,Tao_I);
      Kd=EEPROM.get(30,Kd);
      Serial.print("Setpoint: ");
      Serial.print(EEPROM.get(0,Setpoint));
      Serial.print("\n");
      Serial.print("Kp: ");
      Serial.print(EEPROM.get(10,Kp));
      Serial.print("\n");
      Serial.print("Tao_I: ");
      Serial.print(EEPROM.get(20,Tao_I));
      Serial.print("\n");
      Serial.print("Kd: ");
      Serial.print(EEPROM.get(30,Kd));
      Serial.print("\n");
    }else if (inputString=="graph"){
      graficar=!graficar;
    }else if (inputString=="STORESET"){
      EEPROM.commit();
      Serial.println("datos guardados");
    }else if (inputString.startsWith("CHANGESETPOINT/")){
      Serial.println("comando para cambio de SETPOINT");
      int index = inputString.indexOf('/');
      int length = inputString.length();
      String SETPOINT_string = inputString.substring(index+1, length);
      Serial.println("setpoint: "+SETPOINT_string);
      Setpoint=SETPOINT_string.toFloat();
      EEPROM.put(0,Setpoint);
    }else if (inputString.startsWith("CHANGEPID/")&&inputString.endsWith("/")){//CHANGEPID/1/2/3/
      Serial.println("comando para cambio de variables PID");
      int index = inputString.indexOf('/');
      int length = inputString.length();
      String Numeros = inputString.substring(index+1, length);
      Serial.println(Numeros);
      index = Numeros.indexOf('/');
      length = Numeros.length();
      String Kp_string =Numeros.substring(0, index);
      Numeros = Numeros.substring(index+1, length);
      //Serial.println(Numeros);
      index = Numeros.indexOf('/');
      length = Numeros.length();
      String Ki_string =Numeros.substring(0, index);
      Numeros = Numeros.substring(index+1, length);
      //Serial.println(Numeros);
      index = Numeros.indexOf('/');
      length = Numeros.length();
      String Kd_string =Numeros.substring(0, index);
      //Numeros = Numeros.substring(index+1, length);
      //Serial.println(Numeros);
      Serial.println("Ganancia proporcional: "+Kp_string);
      Kp=Kp_string.toFloat();
      EEPROM.put(10,Kp);
      Serial.println("Ganancia integral: "+Ki_string);
      Tao_I=Ki_string.toFloat();
      //Tao_I=1/Tao_I;
      EEPROM.put(20,Tao_I);
      Serial.println("Ganancia Derivativa: "+Kd_string);
      Kd=Kd_string.toFloat();
      EEPROM.put(30,Kd);
    }
    inputString = "";// Borra el string:
    stringComplete = false;//Resetea la recepcion
  }
  if (Tiempo_Actual_loop1-Tiempo_Previo_loop1>=1000000)
  {
    
    //calculo del angulo
    if (graficar)
    {
      graph["angle"] = angulo;
      graph["aceleration"] = aceleracion;
      graph["setpoint"] = Setpoint;
      serializeJson(graph, Salida_grafica);
      Serial.println(Salida_grafica);
    }
  }
  
  }
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();// get the new byte:
    inputString += inChar;    // add it to the inputString:
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      inputString.remove(inputString.lastIndexOf("\n"));//Elimina el caracter salto de linea
      inputString.remove(inputString.lastIndexOf("\r"));//Elimina el caracter Carriage return
      stringComplete = true;
    }
  }
}