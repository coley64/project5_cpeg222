/***************************************************************************

 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_NeoPixel.h>
#include <HardwareTimer.h>
#ifndef SSD_Array_h
#define SSD_Array_h

#include "stm32f4xx.h"
#ifdef __cplusplus
extern "C" {
#endif
void SSD_init(void);
void SSD_update(int digitSelect, int value, int decimalPoint);
#ifdef __cplusplus
}
#endif
#endif // SSD_Array.h

#define LED_PIN PA0
#define LED_COUNT 4
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
volatile bool buttonPressed = false;
volatile int state = 0; // 0 = Temp(C), 1 = Temp(F), 2 = Humidity(%), 3 = Pressure(atm)
Adafruit_BME280 bme; // I2C

volatile bool buttonFlag = false;
volatile bool serialFlag = false;

float tempC = 0.0f;
float tempF = 0.0f;
float humidity = 0.0f;
float atm = 0.0f;

int digitSelect = 0;

unsigned long delayTime;
void printValues();
void buttonISR();
void setColorForState(int state);
void timerISR();
void serial_timerISR();

void setup() {
    Serial.begin(9600);
    while(!Serial);    // lets get serial running!
    Serial.println("BME280 test");
    pinMode(PC13, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PC13), buttonISR, FALLING);

    unsigned status;
    status = bme.begin(0x76); 

    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    }
    
    Serial.println("-- Default Test: Sensor Online! --");
    
    // choose delay b/n uart prints- lets go for 5 seconds
    delayTime = 4500; // 2500 prev

    Serial.println();
    strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
    strip.setBrightness(25); // Set BRIGHTNESS to about 1/5 (max = 255)
    setColorForState(state); // Set initial color
    strip.show();            // Turn OFF all pixels ASAP

    // init timer
    HardwareTimer *MyTim2 = new HardwareTimer(TIM2);
    MyTim2->setOverflow(500, HERTZ_FORMAT); // 500 Hz = 2 ms firing, should be enough for ssd right?
    MyTim2->attachInterrupt(timerISR);
    MyTim2->resume();

    HardwareTimer *MyTim3 = new HardwareTimer(TIM3);
    MyTim3->setOverflow(5000000, MICROSEC_FORMAT); // 5 second
    MyTim3->attachInterrupt(serial_timerISR);
    MyTim3->resume();

    SSD_init(); // init 7-seg display
}


void loop() { 
    tempC = bme.readTemperature();
    tempF = tempC * 9.0 / 5.0 + 32.0;
    humidity = bme.readHumidity();
    atm = bme.readPressure() / 101325.0F;
    if (buttonFlag) {
        buttonFlag = false;
        state = (state + 1) % 4;
        setColorForState(state);
    }
    if (serialFlag){
        serialFlag = false;
        printValues();
    }
}

void printValues() {
    Serial.print("Temp(C) = " + 
    String(tempC) + 
    ",\tTemp(F) = " +
    String(tempF) +
    +",\tRelHum(%) = " +
    String(humidity) +
    ",\tPress(atm) = " +
    String(atm) +
    "\n"
    // "\nState: " + String(state) + "\n" // state debugging
    );
}

void buttonISR() {
    buttonFlag = true;
 
}

// vscode Copilot-generated function... it helps😁!
void setColorForState(int state) {
    switch(state) {
        case 0: // Temp(C)
            for(int i=0; i<LED_COUNT; i++) {
                strip.setPixelColor(i, strip.Color(255, 0, 0)); // Red
            }
            break;
        case 1: // Temp(F)
            for(int i=0; i<LED_COUNT; i++) {
                strip.setPixelColor(i, strip.Color(128, 0, 128)); // Purple
            }
            break;
        case 2: // Humidity(%)
            for(int i=0; i<LED_COUNT; i++) {
                strip.setPixelColor(i, strip.Color(0, 0, 255)); // Blue
            }
            break;
        case 3: // Pressure(atm)
            for(int i=0; i<LED_COUNT; i++) {
                strip.setPixelColor(i, strip.Color(0, 255, 0)); // Green
            }
            break;
        default:
            for(int i=0; i<LED_COUNT; i++) {
                strip.setPixelColor(i, strip.Color(0, 0, 0)); // Off
            }
            break;
    }
    strip.show();
}

void timerISR() {
    switch(state) {
        case 0: // Temp(C)
            SSD_update(digitSelect, tempC*100, 2);
            break;
        case 1: // Temp(F)
            SSD_update(digitSelect, tempF*100, 2);
            break;
        case 2: // Humidity(%)
            SSD_update(digitSelect, humidity*100, 2);
            break;
        case 3: // Pressure(atm)
            SSD_update(digitSelect, atm*1000, 1);
            break;
    }
    digitSelect = (digitSelect + 1) % 4;
}

void serial_timerISR(){
    serialFlag = true;
}
