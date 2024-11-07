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

//IRsensor
#define SensorPin 12
ESP32SharpIR pig(ESP32SharpIR::GP2Y0A21YK0F, SensorPin);

//Motor pins
//left
#define IN1  27  // Control pin 1
#define IN2  14  // Control pin 2
#define ENA  18  // PWM pin #1

//right
#define IN3  4  // Control pin 3
#define IN4  2  // Control pin 4
#define ENB  15  // PWM pin #2

//Color sensor variables
int rMax, bMax,gMax;
int rMin, bMin,gMin;
int RGB[3];

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

    while(!sensor.colorAvailable())
    {
        delay(5);
    }

    RememberColor();

}

// ignore stuff with IR sensor
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

void RememberColor()
{
    // modify to better remember color on inital pick
    int r, g, b, a;

    /*
    sensor.readColor(r, g, b, a);
    rMax = r;
    gMax = g;
    bMax = b;
    rMin = r;
    gMin = g;
    bMin = b;
    */

    sensor.readColor(r, g, b, a);

    RGB[1] = r;
    RGB[2] = g;
    RGB[3] = b;
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

//color sensor

    //try to make the rotate until it detects the right color
    // in which then it moves forward

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

    vTaskDelay(1);
}

//use function to control wheel mode and speed
void SpinForward(int INA, int INB, int PWM, int speed)
{
    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
    analogWrite(PWM, speed);
}

void SpinBackward(int INA, int INB, int PWM, int speed)
{
    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
    analogWrite(PWM, speed);
}

void StopRotation(int INA, int INB)
{
    digitalWrite(INA, LOW);
    digitalWrite(INB, LOW);
}
