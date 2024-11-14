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
#endif  !CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#include <bits/stdc++.h>

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

Servo myServo;
#define servoPin 15


#define TIME_OUT 2500;

//IRsensor
#define SensorPin 12

//Motor pins
//left
#define IN1  19  // Control pin 1
#define IN2  5  // Control pin 2
#define ENA  18  // PWM pin #1

//right
#define IN3  3  // Control pin 3
#define IN4  1  // Control pin 4
#define ENB  23  // PWM pin #2

//Color sensor variables
float RGB[3];

int servoPWM = 1500;
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

void RememberColor()
{
    
    // modify to better remember color on inital pick    
    int r, g, b, a;
    float Rtotal =0, Gtotal =0, Btotal=0;
    StopRotation(IN1, IN2); //stop spin left motor 
    StopRotation(IN3, IN4); // stop spin right motor 

    for (int i = 0; i < 400; i++)
    {
        sensor.readColor(r, g, b, a);
        Rtotal += r; 
        Gtotal += g; 
        Btotal += b; 
        
    }
    RGB[0] = Rtotal/400;
    RGB[1] = Gtotal/400;
    RGB[2] = Btotal/400;

    Serial.print(RGB[0]);
    Serial.print(RGB[1]);
    Serial.print(RGB[2]);
}

// Arduino setup function. Runs in CPU 1
void setup() {
    //Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();

    ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

    pinMode(LED, OUTPUT);

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
    myServo.attach(servoPin);
}
bool checkAuto = true;
// Arduino loop function. Runs in CPU 1
void loop() {
    
    if(checkAuto){
        
            //color sensor

    //try to make the rotate until it detects the right color
    // in which then it moves forward
    while(!sensor.colorAvailable())
    {
        delay(5);
    }
    int r, g, b, a;


    SpinForward(IN1, IN2, ENA, 90);
    SpinForward(IN3, IN4, ENB, 90);
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

    
    if (abs(RGB[0]-r) < 2 && abs(RGB[1]-g) < 2 && abs(RGB[2]-b) < 2 )
    {
        StopRotation(IN1, IN2); //stop spin left motor 
        StopRotation(IN3, IN4); // stop spin right motor
        Serial.print("matched");
        Serial.print("\n");
        myServo.write(1750);
    }
    else
    {
        myServo.write(1500);
    }
    delay(500);

    vTaskDelay(500);
    }
    if (!checkAuto){
         BP32.update();
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        GamepadPtr controller = myGamepads[i];
        if (controller && controller->isConnected()) {
            // divide by 512 to make max values -1 and 1
            double leftX = controller->axisX(); // [-512,512] might give error or not work properly if axisX 
            double leftY = -controller->axisY(); // [-512,512]
            int MAX_WHEEL_SPEED = 200;
            int leftWheelSpeed = (leftY / 512.00) * MAX_WHEEL_SPEED;
            int rightWheelSpeed = (leftY / 512.00) * MAX_WHEEL_SPEED;
            // make code more efficient by combining both statements and combining moving forwards and backwards
            if (leftX > 0) { // x joystick value greater than 0 (rightside)
                rightWheelSpeed -= (leftX / 512.00) * MAX_WHEEL_SPEED;
                //Serial.println(controller->axisRX());
                if (leftY >= 0) {
                    SpinForward(IN1, IN2, ENA, abs(leftWheelSpeed));
                    SpinForward(IN3, IN4, ENB, abs(rightWheelSpeed));
                } else if (leftY < 0) {
                    SpinBackward(IN1, IN2, ENA, abs(leftWheelSpeed));
                    SpinBackward(IN3, IN4, ENB, abs(rightWheelSpeed));
                }
            } else if (leftX < 0) { // x joystick value less than 0 (leftside)
                leftWheelSpeed -= (leftX / 512.00) * MAX_WHEEL_SPEED;
                //Serial.println(controller->axisRX());
                if (leftY >= 0) {
                    SpinForward(IN1, IN2, ENA, abs(leftWheelSpeed));
                    SpinForward(IN3, IN4, ENB, abs(rightWheelSpeed));
                } else if (leftY < 0) {
                    SpinBackward(IN1, IN2, ENA, abs(leftWheelSpeed));
                    SpinBackward(IN3, IN4, ENB, abs(rightWheelSpeed));
                }
            } else {
                if (leftY > 0) {
                    SpinForward(IN1, IN2, ENA, abs(leftWheelSpeed));
                    SpinForward(IN3, IN4, ENB, abs(rightWheelSpeed));
                } else if (leftY <= 0) {
                    SpinBackward(IN1, IN2, ENA, abs(leftWheelSpeed));
                    SpinBackward(IN3, IN4, ENB, abs(rightWheelSpeed));
                }
            }
            
            Serial.println(leftX);
            Serial.println(leftY);
            //if(controller->axisRX() == 0) { // stop motor 1
                //Serial.println(controller->axisRX());
                // StopRotation(IN1, IN2); //stop spin left motor 
                // StopRotation(IN3, IN4); // stop spin right motor 
            //}

        }
    }
 }
   BP32.update();
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        GamepadPtr controller = myGamepads[i];
        if (controller && controller->isConnected()) {
            if (controller->miscBack()) {
             checkAuto = false;
            }
        }
    }
}
