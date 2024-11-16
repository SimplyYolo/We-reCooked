#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO
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

#define TIME_OUT 2500;
QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 200;

float Kp = 0;
float Kd = 0;
float Ki = 0 ;

int threshold[SensorCount]; 
int Maximum[SensorCount];
int Minimum[SensorCount];

//Motor pins
//left
#define IN1  19 // Control pin 1
#define IN2  5 // Control pin 2
#define ENA  18  // PWM pin #1

//right
#define IN3  16  // Control pin 3
#define IN4  17  // Control pin 4
#define ENB  23  // PWM pin #2
int count = 0;
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

void SpinForward(int INA, int INB, int PWM, int speed)
{
    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
    analogWrite(PWM, speed);
}

void SpinBackward(int INA, int INB, int PWM, int speed)
{
    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
    analogWrite(PWM, speed);
}

void StopRotation(int INA, int INB)
{
    digitalWrite(INA, LOW);
    digitalWrite(INB, LOW);
}

void SpinCalibrate(int SensorCount)
{
    digitalWrite(LED, HIGH);
    for (int i = 0; i < 400; i++)
    {
       SpinForward(IN1,IN2,ENA, 250); //spin left motor forward
        SpinBackward(IN3, IN4, ENB, 250); //spin right motor backward
        qtr.calibrate();
        qtr.readLineBlack(sensorValues);
        for (int j = 0; j < SensorCount; j++)
        {
            if (sensorValues[j] > Maximum[j])
            {
                Maximum[j] = sensorValues[j];   
            }
            if (sensorValues[j] < Minimum[j])
            {
                Minimum[j] = sensorValues[j];   
            }
        }
        
    }

    // print values
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        Serial.print(Maximum[i]);
        Serial.print(' ');
    }
    Serial.println();

    for (uint8_t i = 0; i < SensorCount; i++)
    {
        Serial.print(Minimum[i]);
        Serial.print(' ');
    }
    Serial.println();

    //Threshold Calculations
    for ( int i = 0; i < SensorCount; i++)
    {
    threshold[i] = (Minimum[i] + Maximum[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
    }
    StopRotation(IN1, IN2); //stop spin left motor 
    StopRotation(IN3, IN4); // stop spin right motor 
    digitalWrite(LED, LOW);
    Serial.print("\n");
}

// Arduino setup function. Runs in CPU 1
void setup() {
    // Setup the Bluepad32 callbacks
    //BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    //BP32.forgetBluetoothKeys();
    
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        Maximum[i] = 0;
        Minimum[i] = 4095;
    }

    //ESP32PWM::allocateTimer(0);
	//ESP32PWM::allocateTimer(1);
	//ESP32PWM::allocateTimer(2);
	//ESP32PWM::allocateTimer(3);

    pinMode(LED, OUTPUT);
    
    

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);

// QTR line sensor
    
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){32, 25, 26, 33, 13, 12}, SensorCount);

    //calibrate
    Serial.begin(115200);

    // TODO: Write your setup code here
    SpinCalibrate(SensorCount);
    //Serial.print("Distance: ");
    for (int i = 0; i < SensorCount; i++)
    {
        Serial.print("Line ");
        Serial.print(i);
        Serial.print("\t");
    }
     
}



void lineFollow()
{
    error = (sensorValues[1] - sensorValues[4]); //error can be negative

P = error;
I = I + error;
D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed + PIDvalue;
  rsp = lfspeed - PIDvalue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 250;
  }
  if (rsp < 0) {
    rsp = 0;
  }
SpinForward(IN1, IN2, ENA, lsp); //left
SpinForward(IN3, IN4, ENB, rsp); //right
Serial.print("\n");
Serial.print("Left speed");
Serial.print("\t");
Serial.print("Right speed");
Serial.print("\n");
Serial.print(lsp);
Serial.print("\t");
Serial.print(rsp);
}

void PrintLine()
{
    for (int i=0; i < SensorCount; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.print('\n');

}

// Arduino loop function. Runs in CPU 1
void loop() {

    //QTR Sensor 
    qtr.readLineBlack(sensorValues);
    PrintLine();
    
        if (sensorValues[0] > threshold[0] &&  sensorValues[5] < threshold[5])
        {
            lsp = 0; 
            rsp = lfspeed;
            StopRotation(IN1, IN2); //left
            SpinForward(IN3, IN4, ENB, lfspeed); //right
            Serial.print("Extreme left");
        }
        else if (sensorValues[0] < threshold[0] &&  sensorValues[5] > threshold[5])
        {
            lsp = lfspeed; 
            rsp = 0;
            StopRotation(IN3, IN4); //right
            SpinForward(IN1, IN2, ENA, lfspeed); //left
            Serial.print("Extreme right");
        }
        else if ( ((sensorValues[2] + sensorValues[3])/2) > ((threshold[2] + threshold[3])/2) )
        {
        Kp = 0.0006 * (1000 - ((sensorValues[2] + sensorValues[3])/2));
        Kd = 10 * Kp;
        //Ki = 0.0001;
        lineFollow();
        Serial.print("Forward");
        }

    /*
    for (int i=0; i < SensorCount; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.println();
    */

    delay(250);

    vTaskDelay(1);
}
