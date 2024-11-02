/****************************************************************************
Copyright 2021 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
****************************************************************************/

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include <Arduino.h>
#include <Bluepad32.h>

#include <ESP32Servo.h>
#include <ESP32SharpIR.h>
#include <QTRSensors.h>
#include <Arduino_APDS9960.h>

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];
#define LED 2

//Color Sensor
#define APDS9960_INT 0
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ 100000

TwoWire I2C_0 = TwoWire(0);
APDS9960 sensor = APDS9960(I2C_0, APDS9960_INT);

#define TIME_OUT 2500;
QTRSensors qtr;

const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

#define SensorPin 12
ESP32SharpIR pig(ESP32SharpIR::GP2Y0A21YK0F, SensorPin);

// This callback gets called any time a new gamepad is connected.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }
}


// Arduino setup function. Runs in CPU 1
void setup() {
    // Setup the Bluepad32 callbacks
    //BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    //BP32.forgetBluetoothKeys();

    ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

    pinMode(LED, OUTPUT);

//IRsensor
    pig.setFilterRate(1.0f);

// color sensor
    I2C_0.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    sensor.setInterruptPin(APDS9960_INT);
    sensor.begin();
    Serial.begin(115200);
// QTR line sensor
    
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){33, 25, 32, 26}, SensorCount);

    //calibrate
    /*
    digitalWrite(LED, HIGH);
    for (int i = 0; i < 400; i++)
    {
        qtr.calibrate();
    }

    for (uint8_t i = 0; i < SensorCount; i++)
    {
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print(' ');
    }
    Serial.println();

    // print the calibration maximum values measured when emitters were on
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        Serial.print(qtr.calibrationOn.maximum[i]);
        Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    delay(1000);
    digitalWrite(LED, LOW);
    */

    // TODO: Write your setup code here
    Serial.print("Distance: ");
}

void CheckDistance()
{
    float Distance = pig.getDistanceFloat();
    Serial.print(Distance);
    //Serial.print("\n");
    if (Distance < 5.0f)
    {
        digitalWrite(LED, HIGH);
    }
    else 
    {
        digitalWrite(LED, LOW);
    }
}

// Arduino loop function. Runs in CPU 1
void loop() {
    //BP32.update();
    /*
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        GamepadPtr myGamepad = myGamepads[i];
        if (myGamepad && myGamepad->isConnected()) {
            // TODO: Write your controller code here

        }
    }
    */
CheckDistance();

//color sensor
    /*
    while(!sensor.colorAvailable())
    {
        delay(5);
    }

    int r, g, b, a;
    sensor.readColor(r, g, b, a);

    Serial.print("r = ");
    Serial.print(r);
    Serial.print("\n");
    Serial.print("g = ");
    Serial.print(g);
    Serial.print("\n");
    Serial.print("b = ");
    Serial.print(b);
    Serial.print("\n");

    delay(5000);
    */

    //QTR Sensor 
    /*
    qtr.readLineBlack(sensorValues);
    int i=0;
    int line1= sensorValues[i=0];
    int line2= sensorValues[i=1];
    int line3= sensorValues[i=2]; 
    int line4= sensorValues[i=3];

    
    for (int i=0; i < SensorCount; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.println();
    

    Serial.print("Sensor 1: ");
    Serial.print(line1);
    Serial.print("\n");
    Serial.print("Sensor 2: ");
    Serial.print(line2);
    Serial.print("\n");
    Serial.print("Sensor 3: ");
    Serial.print(line3);
    Serial.print("\n");
    Serial.print("Sensor 4: ");
    Serial.print(line4);
    Serial.print("\n");
    */
    delay(250);
    

    //digitalWrite(LED, HIGH);  // turn the LED on (HIGH is the voltage level)
     //delay(5000);                      // wait for a second
     //digitalWrite(LED, LOW);   // turn the LED off by making the voltage LOW
    //delay(5000); 

    // TODO: Write your periodic code here

    vTaskDelay(1);
}
