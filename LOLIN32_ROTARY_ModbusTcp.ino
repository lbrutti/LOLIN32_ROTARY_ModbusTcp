/*
  Modbus-Arduino Example - Test Analog Input (Modbus IP ESP8266)
  Read Analog sensor on Pin ADC (ADC input between 0 ... 1V)
  Original library
  Copyright by André Sarmento Barbosa
  http://github.com/andresarmento/modbus-arduino

  Current version
  (c)2017 Alexander Emelianov (a.m.emelianov@gmail.com)
  https://github.com/emelianov/modbus-esp8266
*/


#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"

#ifdef ESP8266
#include <ESP8266WiFi.h>
#else //ESP32
#include <WiFi.h>
#endif

#include <ModbusIP_ESP8266.h>

#include "wiFiSecrets.h"

#include <Wire.h>
#include "Adafruit_HTU21DF.h"

#define ROTARY_ENCODER_A_PIN 21
#define ROTARY_ENCODER_B_PIN 32
#define ROTARY_ENCODER_BUTTON_PIN 25
#define ROTARY_ENCODER_VCC_PIN -1 /* 27 put -1 of Rotary encoder Vcc is connected directly to 3,3V; else you can use declared output pin for powering rotary encoder */

//depending on your encoder - try 1,2 or 4 to get expected behaviour
//#define ROTARY_ENCODER_STEPS 1
//#define ROTARY_ENCODER_STEPS 2
#define ROTARY_ENCODER_STEPS 4

#define DEBUG 1

//instead of changing here, rather change numbers above
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);


//Modbus Registers Offsets
const int STALE_ROTARY_VALUE_HREG = 51000;
const int ROTARY_VALUE_HREG = 52000;

//ModbusIP object
ModbusIP mb;

long ts;
int enc_value;

uint8_t write_to_stale = 1;
uint8_t connection_available = 1;
void rotary_onButtonClick()
{
  static unsigned long lastTimePressed = 0;
  //ignore multiple press in that time milliseconds
  if (millis() - lastTimePressed < 500)
  {
    return;
  }
  lastTimePressed = millis();
  Serial.print("button pressed at ");
  Serial.println(millis());
}

void rotary_loop()
{
  //dont print anything unless value changed
  if (!rotaryEncoder.encoderChanged())
  {
    return;
  }

  Serial.print("Value: ");
  Serial.println(rotaryEncoder.readEncoder());
}



bool cbConn(IPAddress ip) {
  if (DEBUG) {
    Serial.print("MODBUSIP_MAX_CLIENTS : "); Serial.println(MODBUSIP_MAX_CLIENTS);
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
    delay(20);
    Serial.print("Connecting : ");
    Serial.println(ip);
    Serial.print("sending A0 : "); Serial.println(enc_value);
    digitalWrite(LED_BUILTIN, HIGH);

  }
  if (connection_available) {
    if (DEBUG) {
      Serial.print(" connection_available : "); Serial.println(connection_available);
    }
    connection_available = 0;
    return true;
  } else {
    if (DEBUG) {
      Serial.print(" connection_not_available : "); Serial.println(connection_available);
    }

    return connection_available;
  }

}
bool cbDisconn(IPAddress ip) {
  connection_available = 1 ;
  if (DEBUG) {
    Serial.print("Disconnecting : ");
    Serial.println(ip);
  }


  return true;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  //we must initialize rotary encoder
  rotaryEncoder.begin();

  rotaryEncoder.setup(
    [] { rotaryEncoder.readEncoder_ISR(); },
    [] { rotary_onButtonClick(); });

  //set boundaries and if values should cycle or not
  //in this example we will set possible values between 0 and 1000;
  bool circleValues = false;
  rotaryEncoder.setBoundaries(0, 1000, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)

  /*Rotary acceleration introduced 25.2.2021.
     in case range to select is huge, for example - select a value between 0 and 1000 and we want 785
     without accelerateion you need long time to get to that number
     Using acceleration, faster you turn, faster will the value raise.
     For fine tuning slow down.
  */
  //rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
  rotaryEncoder.setAcceleration(250); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration

  mb.onConnect(cbConn);
  mb.onDisconnect(cbDisconn);

  mb.server();    //Start Modbus IP
  mb.addHreg(ROTARY_VALUE_HREG);
  if (write_to_stale) {
    mb.addHreg(STALE_ROTARY_VALUE_HREG);
  }
  ts = millis();
}




void loop() {
  //Call once inside loop() - all magic here
  mb.task();
  //Read each two seconds
  if (millis() > ts + 2000) {

    if (write_to_stale) {
      //write old value in stale register
      mb.Hreg(STALE_ROTARY_VALUE_HREG, enc_value);
    }


    if (rotaryEncoder.encoderChanged())
    {
      enc_value = rotaryEncoder.readEncoder();
      Serial.print("Value: ");
      Serial.println(enc_value);
      //update value and set in current data hreg
      mb.Hreg(ROTARY_VALUE_HREG, enc_value);
    }

    ts = millis();

  }
  delay(10);
}
