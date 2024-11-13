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

int lsp, rsp;
int lfspeed = 200;

#define TIME_OUT 2500;

//IRsensor
#define LeftPin 15
ESP32SharpIR left(ESP32SharpIR::GP2Y0A21YK0F, LeftPin);

#define RightPin 2
ESP32SharpIR right(ESP32SharpIR::GP2Y0A21YK0F, RightPin);

#define FrontPin 4
ESP32SharpIR front(ESP32SharpIR::GP2Y0A21YK0F, FrontPin);

//Motor pins
//left
#define IN1  19  // Control pin 1
#define IN2  5  // Control pin 2
#define ENA  18  // PWM pin #1

//right
#define IN3  3  // Control pin 3
#define IN4  1  // Control pin 4
#define ENB  23  // PWM pin #2

float DistThreshold = 10;

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

void MoveForward()
{
    SpinForward(IN1, IN2, ENA, lfspeed);
    SpinForward(IN3, IN4, ENB, lfspeed);
}

void stop()
{
    StopRotation(IN1, IN2);
    StopRotation(IN3, IN4);
}

void rotateLeft()
{
    StopRotation(IN1, IN2); //left
    SpinForward(IN3, IN4, ENB, lfspeed); //right
}

void rotateRight()
{
    StopRotation(IN3, IN4); //right
    SpinForward(IN1, IN2, ENA, lfspeed); //left
}

// Arduino setup function. Runs in CPU 1
void setup() {
    // Setup the Bluepad32 callbacks
    //BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    //BP32.forgetBluetoothKeys();
    Serial.begin(115200);
    ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

    pinMode(LED, OUTPUT);

//IRsensor
    left.setFilterRate(1.0f);
    right.setFilterRate(1.0f);
    front.setFilterRate(1.0f);
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

    float frontDist = front.getDistanceFloat();
    float leftDist = left.getDistanceFloat();
    float rightDist = right.getDistanceFloat();

    Serial.print("front distance: ");
    Serial.print(frontDist);
    Serial.print("\n");
    Serial.print("left distance: ");
    Serial.print(leftDist);
    Serial.print("\n");
    Serial.print("right distance: ");
    Serial.print(rightDist);
    Serial.print("\n");

    /*
    if (frontDist > DistThreshold)
    {
        MoveForward();
    }
    else if (leftDist < rightDist)
    {
        stop();
        while (frontDist <= DistThreshold)
        {
            rotateRight();
        }
    }
    else if (leftDist > rightDist)
    {
        stop();
        while (frontDist <= DistThreshold)
        {
            rotateLeft();
        }
    }
    */
    

    delay(500);

    vTaskDelay(1);
}


