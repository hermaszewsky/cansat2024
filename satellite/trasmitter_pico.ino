//#include <Wire.h>
// Include required libraries
#include <SPI.h>
#include <LoRa.h>
//#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3

#include "Adafruit_SHT4x.h"
#include "SparkFunBMP384.h"
 
BMP384 pressureSensor;
//SFE_UBLOX_GNSS myGNSS;
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
//for Raspberry Pi Pico:
uint8_t i2cAddress = BMP384_I2C_ADDRESS_DEFAULT; // 0x77
//#define myWire1 Wire1

const int csPin = 17;
const int resetPin = 1;
const int irqPin = 0;

// Message counter
byte msgCount = 0;
 
void setup() {
  
  //Wire.begin();
  Serial.begin(115200);
  delay(5000);
  while (!Serial)
    ;
  // Setup LoRa module
  LoRa.setPins(csPin, resetPin, irqPin);

   Serial.println("LoRa Sender Test");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Connected!");

  delay(2000);

  Serial.println("Adafruit SHT4x test");
  if (! sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
    while (1) delay(1);
  }
  Serial.println("Found SHT4x sensor");
  Serial.print("Serial number 0x");
  Serial.println(sht4.readSerial(), HEX);

  sht4.setPrecision(SHT4X_HIGH_PRECISION);

  Serial.println("BMP384 test");
  while(pressureSensor.beginI2C(i2cAddress) != BMP3_OK)
  {   
      Serial.println("Error: BMP384 not connected, check wiring and I2C address!");
      delay(1000);
  }
  Serial.println("BMP384 connected!");
  
}
 


void loop() {
  bmp3_data data;
  int8_t err = pressureSensor.getSensorData(&data);
  if(err == BMP3_OK) {
    Serial.print("P "); Serial.print(data.pressure); Serial.println("hPa");
  }
  else {
      Serial.print("Error getting data from sensor! Error code: ");
      Serial.println(err);
  }

  sensors_event_t humidity, temp;
  uint32_t timestamp = millis();
  sht4.getEvent(&humidity, &temp);
  timestamp = millis() - timestamp;

  Serial.print("T "); Serial.print(temp.temperature); Serial.println(" C");

  Serial.print("Read duration (ms): ");
  Serial.println(timestamp);
 
  Serial.print("Sending packet: ");
  Serial.println(msgCount);
 
  // Send packet
  LoRa.beginPacket();
  LoRa.print("MSG: ");
  LoRa.print(msgCount);  
  LoRa.print(" T: ");
  LoRa.print(temp.temperature);
  LoRa.print(" P: ");
  LoRa.println(data.pressure);
  LoRa.endPacket();
  Serial.println("Lora sent ended");

  // Increment packet counter
  msgCount++;
 
  // 5-second delay
  delay(5000);
}