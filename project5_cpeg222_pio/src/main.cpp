#include <Arduino.h>
#include <Wire.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  int result = myFunction(2, 3);
  Wire.begin(address);
}


void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("CPEG222 Project 5 - BME280 Values");
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}