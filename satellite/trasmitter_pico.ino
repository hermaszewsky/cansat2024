/*
  LoRa Demo 1 Sender
  lora-demo1-send.ino
  Sends test packet with packet count
  Requires LoRa Library by Sandeep Mistry - https://github.com/sandeepmistry/arduino-LoRa
  
  DroneBot Workshop 2023
  https://dronebotworkshop.com
*/
 
// Include required libraries
#include <SPI.h>
#include <LoRa.h>
 
// Define the pins used by the LoRa module
// for Arduino UNO
// const int csPin = 4;     // LoRa radio chip select
// const int resetPin = 2;  // LoRa radio reset
// const int irqPin = 3;    // Must be a hardware interrupt pin

//for Raspberry Pi Pico:
const int csPin = 17;
const int resetPin = 1;
const int irqPin = 0;

// Message counter
byte msgCount = 0;
 
void setup() {
 
  Serial.begin(9600);
  delay(5000);
  while (!Serial)
    ;
 
  // Setup LoRa module
  LoRa.setPins(csPin, resetPin, irqPin);
 
  Serial.println("LoRa Sender Test");
 
  // Start LoRa module at local frequency
  // 433E6 for Asia
  // 866E6 for Europe
  // 915E6 for North America
 
  if (!LoRa.begin(433E6)) {
    
    while (1) {
      Serial.println("Starting LoRa failed!");
      delay(1000);
    }
      ;
  }
}
 
void loop() {
 
  Serial.print("Sending packet: ");
  Serial.println(msgCount);
 
  // Send packet
  LoRa.beginPacket();
  LoRa.print("Packet ");
  LoRa.print(msgCount);
  LoRa.endPacket();
 
  // Increment packet counter
  msgCount++;
 
  // 5-second delay
  delay(5000);
}