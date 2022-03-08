/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/
*********/

#include <Arduino.h>
#include <main.h>

#define LED_1 1
#define LED_2 2
#define LED_3 3
#define LED_4 4

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_2, HIGH);
  Serial.println("LED is on");
  delay(1000);
  digitalWrite(LED_2, LOW);
  Serial.println("LED is off");
  delay(1000);
}