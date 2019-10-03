/*
   Vario2HoTT
   Ziege-One
   v1.0
 
 Arduino pro Mini 5v/16mHz w/ Atmega 328

 
 /////Pin Belegung////
 D0: 
 D1: 
 D2: 
 D3: RX / TX Softserial HoTT V4
 D4: 
 D5: 
 D6: 
 D7: 
 D8: 
 D9: 
 D10: 
 D11: 
 D12: 
 D13: LED, um die Kommunikation zu visualisieren
 
 A0: 
 A1: 
 A2: 
 A3: 
 A4: MS5611 / SDA
 A5: MS5611 / SCL
 
 */
 
// ======== Vario2HoTT  =======================================

#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "Message.h"
#include <inttypes.h>

#include <Wire.h>
#include "MS5611.h"

#include "RunningAverage.h"

// LED 
#define LEDPIN_PINMODE    pinMode (LED_BUILTIN, OUTPUT);
#define LEDPIN_OFF        digitalWrite(LED_BUILTIN, LOW);
#define LEDPIN_ON         digitalWrite(LED_BUILTIN, HIGH);

//#define Debug                             // Ein/Aus Debugging

// Time interval [ms] for display updates:
const unsigned long Debug_INTERVAL = 5000;
static unsigned long lastTime_Debug =0;       // in ms
static unsigned long lastTime_climbrate =0;   // in ms
unsigned long timer = millis();               // in ms



// Luftdruck beim einschalten
double referencePressure; 
// Real Werte
float Altitude;
float Altitude_old;
float M1s;
float M3s;
float M10s;
float MaxAltitude;
float MinAltitude;

// Hott Werte für Telemetry
uint16_t altitude = 500;
uint16_t m1s = 30000;
uint16_t m3s = 30000;
uint16_t m10s = 30000;
uint16_t maxAltitude = 500;
uint16_t minAltitude = 500;

// Duchschnittswerte Anlegen
RunningAverage cr1s(5);
RunningAverage cr3s(15);
RunningAverage cr10s(50);

// Einfügen Message.cpp Funktionen für Initialisierung und Hauptprogramm
GMessage message;

// Einfügen MS5611.cpp Funktionen für Initialisierung und Hauptprogramm
MS5611 ms5611;


// ======== Setup & Initialisierung =======================================
void setup()
{
  
  Serial.begin (115200); // 115200 für Debugging
  
  LEDPIN_PINMODE
  LEDPIN_ON
  delay(200);
  LEDPIN_OFF
  delay(200);
  LEDPIN_ON
  delay(200);
  LEDPIN_OFF
  delay(200);
  LEDPIN_ON
  delay(200);
  LEDPIN_OFF
  
  // Initialisierung Graupner HoTT Protokoll
  message.init();

  // Initialisierung MS5611 Sensor
  ms5611.begin();

  // Luftdruck beim einschalten 
  referencePressure = ms5611.readPressure();  

  // Duchschnittswerte reset
  cr1s.clear();
  cr3s.clear();
  cr10s.clear();
}

// ======== Haupt Schleife  =======================================
void loop()  {
  timer=millis();
    #ifdef Debug
      //Für Debugging an Serial 115200
      
      if (timer-lastTime_Debug > Debug_INTERVAL)  // wenn Debug_INTERVAL ms vergangen sind
      {
      message.debug();
      lastTime_Debug = timer;
      }
    #endif
  // Keine Kommunikation
  LEDPIN_OFF 
  
  // Senden und Update Graupner HoTT Telemetry
  message.main_loop();

  // MS5611 Sensor auslesen
  if (timer-lastTime_climbrate > 200)  // wenn 200ms vergangen sind
  {
    long realPressure = ms5611.readPressure();  
    Altitude = ms5611.getAltitude(realPressure, referencePressure);  
    float rate = (Altitude - Altitude_old) * 5; //5 Messwerte pro 1s
    Altitude_old = Altitude;
    lastTime_climbrate = timer;
  
    cr1s.addValue(rate);
    cr3s.addValue(rate);
    cr10s.addValue(rate);

    M1s = cr1s.getAverage();
    M3s = cr3s.getAverage();
    M10s = cr10s.getAverage();

    altitude = Altitude + 500;
    m1s = 30000 + M1s * 100;
    m3s = 30000 + M3s * 100;
    m10s = 30000 + M10s * 100;

    if (Altitude > MaxAltitude)
    {
      MaxAltitude = Altitude;
      maxAltitude = altitude;
    }
    if (Altitude < MinAltitude)
    {
      MinAltitude = Altitude;
      minAltitude = altitude;
    }   
  }
}
