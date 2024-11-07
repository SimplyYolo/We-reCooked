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

#define TIME_OUT 2500;
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 200;

float Kp = 0;
float Kd = 0;
float Ki = 0 ;

int threshold[7];

//Motor pins
//left
#define IN1  27  // Control pin 1
#define IN2  14  // Control pin 2
#define ENA  12  // PWM pin #1

//right
#define IN3  4  // Control pin 3
#define IN4  2  // Control pin 4
#define ENB  15  // PWM pin #2

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

void SpinCalibrate(int SensorCount)
{
    digitalWrite(LED, HIGH);
    for (int i = 0; i < 3000; i++)
    {
        SpinForward(IN1, IN2, ENA, 50); //spin left motor forward
        SpinBackward(IN3, IN4, ENB, 50); //spin right motor backward
        qtr.calibrate();
    }

    // print values
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print(' ');
    }
    Serial.println();

    for (uint8_t i = 0; i < SensorCount; i++)
    {
        Serial.print(qtr.calibrationOn.maximum[i]);
        Serial.print(' ');
    }
    Serial.println();

    //Threshold Calculations
    for ( int i = 0; i < 7; i++)
    {
    threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
    }
    StopRotation(IN1, IN2); //stop spin left motor 
    StopRotation(IN3, IN4); // stop spin right motor 
    digitalWrite(LED, LOW);
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

    Serial.begin(115200);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
// QTR line sensor
    
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){32, 33, 25, 26, 19, 18, 5, 17}, SensorCount);

    //calibrate
    SpinCalibrate(SensorCount);

    // TODO: Write your setup code here
    //Serial.print("Distance: ");
}

void lineFollow()
{
    error = (analogRead(2) - analogRead(5)); //error can be negative

P = error;
I = I + error;
D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < 0) {
    rsp = 0;
  }
SpinForward(IN1, IN2, ENA, lsp); //left
SpinForward(IN3, IN4, ENB, rsp); //right
}

// Arduino loop function. Runs in CPU 1
void loop() {

    //QTR Sensor 
    qtr.readLineBlack(sensorValues);
    
    while(true)
    {
        if (sensorValues[0] > threshold[0] &&  sensorValues[7] < threshold[7])
        {
            lsp = 0; 
            rsp = lfspeed;
            StopRotation(IN1, IN2); //left
            SpinForward(IN3, IN4, ENB, lfspeed); //right
        }
        else if (sensorValues[0] < threshold[0] &&  sensorValues[7] > threshold[7])
        {
            lsp = lfspeed; 
            rsp = 0;
            StopRotation(IN3, IN4); //right
            SpinForward(IN1, IN2, ENA, lfspeed); //left
        }
        else if ( ((sensorValues[3] + sensorValues[4])/2) > ((threshold[3] + threshold[4])/2) )
        {
        Kp = 0.0006 * (1000 - analogRead(3));
        Kd = 10 * Kp;
        //Ki = 0.0001;
        lineFollow();
        }
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
