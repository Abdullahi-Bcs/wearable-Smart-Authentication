// BLYNK APP CONTROL
#define BLYNK_TEMPLATE_ID "TMPL2Mef02DGm"
#define BLYNK_DEVICE_NAME "IOT BASED SMART PATIENT FALL AND GPS SYSTEM"
#define BLYNK_AUTH_TOKEN "-ZWFoHi5tKOYpIUhzdppYsJqvbhOsgJP"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <SimpleTimer.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#define SENSOR_PIN  17 // ESP32 pin GIOP17 connected to DS18B20 sensor's DATA pin
#include <LiquidCrystal_I2C.h> // library for I2C LCD  

#define REPORTING_PERIOD_MS     4000

OneWire oneWire(SENSOR_PIN);
DallasTemperature DS18B20(&oneWire);
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display
SimpleTimer timer;
//BlynkTimer timer;
float temp_C; // temperature in Celsius
float temp_F; // temperature in Fahrenheit

// Create a PulseOximeter object
PulseOximeter pox;

// Time at which the last beat occurred
uint32_t tsLastReport = 0;

// Callback routine is executed when a pulse is detected
void onBeatDetected() {
    Serial.println("Beat!");
}

//------------Indicators of the system -------
int greenled_Pin = 14;
int redled_Pin = 12;
int Activebuzzer_Pin = 27;
//-----------------------------------------

//Temperature sensor declarations
// GND - GND ESP32
// VCC + 3.3V ESP32
int DHTPin = 15;          // What digital pin we're connected to
#define Type DHT22     // DHT 11
DHT dht(DHTPin, Type);
float humidity;
float tempC;
float tempF;


//BLYNK APP AND INTENET CONNECTIONS
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Setsom-tech";
char pass[] = "0614444259";
void setup() {
  Serial.begin(9600);
  DS18B20.begin();
  Blynk.begin(auth, ssid, pass); 
  pinMode(greenled_Pin, OUTPUT);
  pinMode(redled_Pin, OUTPUT);
  pinMode(Activebuzzer_Pin, OUTPUT);
  
  lcd.begin();
  lcd.backlight();
  lcd.clear();  
  dht.begin();

  lcd.setCursor(0,0);
  lcd.print("IOT PATIENT HEALTH");
  lcd.setCursor(0,1);
  lcd.print("CONTROL SYSTEM");
  delay(5000);
  lcd.clear();
  
  lcd.setCursor(0,0);
  lcd.print("TEMP=0");

  lcd.setCursor(9,0);
  lcd.print("BDY=0");

  lcd.setCursor(2,1);
  lcd.print("HEART=0");

      timer.setInterval(4000L, Body_Sensor);
//    timer.setInterval(10000L, DHTSensor2);
//    timer.setInterval(1000L, Heart_Sensor); 

    Serial.print("Initializing pulse oximeter..");

    // Initialize sensor
    if (!pox.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }

  // Configure sensor to use 7.6mA for LED drive
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

    // Register a callback routine
  pox.setOnBeatDetectedCallback(onBeatDetected);
}
//--------------------

void Body_Sensor() { 
  DS18B20.requestTemperatures();       // send the command to get temperatures
  temp_C = DS18B20.getTempCByIndex(0);  // read temperature in °C
  temp_F = tempC * 9 / 5 + 32; // convert °C to °F
//   Blynk.virtualWrite(V1, temp_F); //display the moisture percent. 
//  Blynk.virtualWrite(V0, temp_C); //display the moisture percent.
  Blynk.virtualWrite(V1, temp_F); //display the moisture percent.  
//  humidity = dht.readHumidity();
//  tempC = dht.readTemperature(); // or 
//  tempF = dht.readTemperature(true); //for Fahrenheit
  Serial.print("Temperature: ");
  Serial.print(temp_C);    // print the temperature in °C
  Serial.print("°C");
  Serial.print("  ~  ");  // separator between °C and °F
  Serial.print(temp_F);    // print the temperature in °F
  Serial.println("°F");
  if (temp_C>=38){
    //Blynk.virtualWrite(V5, HIGH);
    digitalWrite(redled_Pin,HIGH);
    digitalWrite(Activebuzzer_Pin,HIGH);
    delay(900);
    //Blynk.virtualWrite(V5, LOW);
    digitalWrite(redled_Pin,LOW);
    digitalWrite(Activebuzzer_Pin,LOW);
    delay(900);    
  }
  else{
    //Blynk.virtualWrite(V5, LOW);
    digitalWrite(redled_Pin,LOW);
    digitalWrite(Activebuzzer_Pin,LOW);
  }
  //delay(500);
  lcd.setCursor(9,0);
  lcd.print("BODY=");
  lcd.print(temp_C);
  Blynk.virtualWrite(V2, temp_C); //display the moisture percent.
  
  } 

//-------------
void loop() {
    // Read from the sensor
 digitalWrite(greenled_Pin,HIGH);
//
    humidity = dht.readHumidity();
    tempC = dht.readTemperature(); // or 
//   Blynk.virtualWrite(V0, tempC);
//   Blynk.virtualWrite(V1, humidity);
//   tempF = dht.readTemperature(true); //for Fahrenheit

    timer.run();
    Blynk.run();   
    pox.update();
    // Grab the updated heart rate and SpO2 levels
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {

        Serial.print("Room Temperature: ");
        Serial.print(tempC);
        Serial.println("°C");
        
        
        Serial.print("Room Humidity: ");
        Serial.print(humidity);
        Serial.println("%");
        lcd.setCursor(0,0);
        lcd.print("TP=");
        lcd.print(tempC);
   //   Blynk.virtualWrite(V0, tempC);
//      Blynk.virtualWrite(V1, humidity);
      
        Serial.print("Heart rate:");
        Serial.print(pox.getHeartRate());
        
        Serial.print("bpm / SpO2:");
        Serial.print(pox.getSpO2());
        Serial.println("%");

        
        lcd.setCursor(2,1);
        lcd.print("HEART: ");
        lcd.print(pox.getHeartRate());        
        Blynk.virtualWrite(V3,pox.getHeartRate());
        Blynk.virtualWrite(V4,pox.getSpO2());
        
        tsLastReport = millis();
    }

}
