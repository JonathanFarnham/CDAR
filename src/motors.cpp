#include "config.h"
#include "motors.h"
#include <Arduino.h>

//Volatile Variables for interrupt context
volatile long left_ticks = 0;
volatile long right_ticks = 0;

//Ramping Settings
#define RAMP_INTERVAL 5 //Update speed every 5ms
#define RAMP_STEP     2 //Change PWM by 4 units per step
// 255 / 2 * 5ms = ~637ms to full speed

//internal speed state
float currentSpeedL = 0;
float currentSpeedR = 0;
float targetSpeedL = 0;
float targetSpeedR = 0;

unsigned long lastRampTime = 0;

//Interrupt Service Routines
void IRAM_ATTR isr_left()
{
    left_ticks++;
}

void IRAM_ATTR isr_right()
{
    right_ticks++;
}

void initMotors()
{
    //Setup Left Motor
    pinMode(MOTOR_LEFT_EN, OUTPUT);
    pinMode(MOTOR_LEFT_IN1, OUTPUT);
    pinMode(MOTOR_LEFT_IN2, OUTPUT);

    //Setup Right Motor
    pinMode(MOTOR_RIGHT_EN, OUTPUT);
    pinMode(MOTOR_RIGHT_IN1, OUTPUT);
    pinMode(MOTOR_RIGHT_IN2, OUTPUT);

    //Encoder Pin Setup
    pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);

    //Attach Interrupts to Encoder
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), isr_left, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), isr_right, RISING);
    

    currentSpeedL = 0; targetSpeedL = 0;
    currentSpeedR = 0; targetSpeedR = 0;
    stopBot();
}

void resetEncoders()
{
    left_ticks = 0;
    right_ticks = 0;
}

//Get average distance
long getEncoderAvg()
{
    return (left_ticks + right_ticks) / 2;
}

void setMotorHardware(int speed, int pinIn1, int pinIn2, int pinPWM)
{
    if (speed > 0)
    {
        //Forward
        digitalWrite(pinIn1, HIGH);
        digitalWrite(pinIn2, LOW);
        analogWrite(pinPWM, speed);
    } else if (speed < 0)
    {
        //Backward
        digitalWrite(pinIn1, LOW);
        digitalWrite(pinIn2, HIGH);
        analogWrite(pinPWM, -speed);
    } else
    {
        //Stop
        digitalWrite(pinIn1, LOW);
        digitalWrite(pinIn2, LOW);
        analogWrite(pinPWM, 0);
    }
}

void updateMotorSpeeds()
{
    if (millis() - lastRampTime >= RAMP_INTERVAL) {
        lastRampTime = millis();

        // 1. Ramp Left Motor
        if (currentSpeedL < targetSpeedL) {
            currentSpeedL += RAMP_STEP;
            if (currentSpeedL > targetSpeedL) currentSpeedL = targetSpeedL;
        } else if (currentSpeedL > targetSpeedL) {
            currentSpeedL -= RAMP_STEP;
            if (currentSpeedL < targetSpeedL) currentSpeedL = targetSpeedL;
        }

        // 2. Ramp Right Motor
        if (currentSpeedR < targetSpeedR) {
            currentSpeedR += RAMP_STEP;
            if (currentSpeedR > targetSpeedR) currentSpeedR = targetSpeedR;
        } else if (currentSpeedR > targetSpeedR) {
            currentSpeedR -= RAMP_STEP;
            if (currentSpeedR < targetSpeedR) currentSpeedR = targetSpeedR;
        }

        // 3. Apply to Hardware
        setMotorHardware((int)currentSpeedL, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_LEFT_EN);
        setMotorHardware((int)currentSpeedR, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_RIGHT_EN);
    }
}

//Manual Commands
void stopBot()
{
    targetSpeedL = 0;
    targetSpeedR = 0;
}

void moveForward() 
{
    targetSpeedL = SPEED_MOVE;
    targetSpeedR = SPEED_MOVE;
}

void moveBackward() 
{
    targetSpeedL = -SPEED_MOVE;
    targetSpeedR = -SPEED_MOVE;
}

void turnLeft() 
{
    // Left motor back, Right motor fwd
    targetSpeedL = -SPEED_TURN;
    targetSpeedR = SPEED_TURN;
}

void turnRight() 
{
    // Left motor fwd, Right motor back
    targetSpeedL = SPEED_TURN;
    targetSpeedR = -SPEED_TURN;
}

//Grid Commands w/ Ramping
void driveStraight(int speed)
{
    targetSpeedL = speed;
    targetSpeedR = -speed;
}

void turnClockwise(int speed) 
{
    targetSpeedL = speed;
    targetSpeedR = -speed;
}

void turnCounterClockwise(int speed) 
{
    targetSpeedL = -speed;
    targetSpeedR = speed;
}