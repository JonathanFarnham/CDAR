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

//Kickstart State Tracking
bool kickstartActiveL = false;
bool kickstartActiveR = false;
unsigned long kickstartTimerL = 0;
unsigned long kickstartTimerR = 0;

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

long getEncoderLeft()
{
    return left_ticks;
}

long getEncoderRight()
{
    return right_ticks;
}

void setMotorHardware(int speed, int pinIn1, int pinIn2, int pinPWM, bool invert)
{
    //If the motor is physically inverted, flip the target speed
    if (invert)
    {
        speed = -speed;
    }
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
    unsigned long currentMillis = millis();

    // 1. Trigger Kickstart if starting from a dead stop
    if ((int)currentSpeedL == 0 && targetSpeedL != 0 && !kickstartActiveL) {
        kickstartActiveL = true;
        kickstartTimerL = currentMillis;
    }
    if ((int)currentSpeedR == 0 && targetSpeedR != 0 && !kickstartActiveR) {
        kickstartActiveR = true;
        kickstartTimerR = currentMillis;
    }

    // 2. Handle Ramping (Updates every RAMP_INTERVAL)
    bool updateHardware = false;
    if (currentMillis - lastRampTime >= RAMP_INTERVAL) {
        lastRampTime = currentMillis;
        updateHardware = true;

        // Ramp Left (only if the kickstart burst is finished)
        if (!kickstartActiveL) {
            if (currentSpeedL < targetSpeedL) {
                currentSpeedL += RAMP_STEP;
                if (currentSpeedL > targetSpeedL) currentSpeedL = targetSpeedL;
            } else if (currentSpeedL > targetSpeedL) {
                currentSpeedL -= RAMP_STEP;
                if (currentSpeedL < targetSpeedL) currentSpeedL = targetSpeedL;
            }
        }

        // Ramp Right (only if the kickstart burst is finished)
        if (!kickstartActiveR) {
            if (currentSpeedR < targetSpeedR) {
                currentSpeedR += RAMP_STEP;
                if (currentSpeedR > targetSpeedR) currentSpeedR = targetSpeedR;
            } else if (currentSpeedR > targetSpeedR) {
                currentSpeedR -= RAMP_STEP;
                if (currentSpeedR < targetSpeedR) currentSpeedR = targetSpeedR;
            }
        }
    }

    // 3. Apply Hardware Signals
    if (kickstartActiveL || kickstartActiveR || updateHardware) {
        
        // --- Left Motor Application ---
        if (kickstartActiveL) {
            if (currentMillis - kickstartTimerL < KICKSTART_DURATION) {
                // Apply full power burst in the target direction
                int kickSpeed = (targetSpeedL > 0) ? KICKSTART_PWM : -KICKSTART_PWM;
                setMotorHardware(kickSpeed, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_LEFT_EN, MOTOR_LEFT_INVERT);
            } else {
                // Burst finished: snap to the minimum moving speed, then let normal ramping resume
                kickstartActiveL = false;
                currentSpeedL = (targetSpeedL > 0) ? MIN_MOVING_PWM : -MIN_MOVING_PWM;
                
                // Safety check in case the commanded target speed is very low
                if (abs(targetSpeedL) < MIN_MOVING_PWM) currentSpeedL = targetSpeedL; 
                
                setMotorHardware((int)currentSpeedL, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_LEFT_EN, MOTOR_LEFT_INVERT);
            }
        } else if (updateHardware) {
            setMotorHardware((int)currentSpeedL, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_LEFT_EN, MOTOR_LEFT_INVERT);
        }

        // --- Right Motor Application ---
        if (kickstartActiveR) {
            if (currentMillis - kickstartTimerR < KICKSTART_DURATION) {
                // Apply full power burst in the target direction
                int kickSpeed = (targetSpeedR > 0) ? KICKSTART_PWM : -KICKSTART_PWM;
                setMotorHardware(kickSpeed, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_RIGHT_EN, MOTOR_RIGHT_INVERT);
            } else {
                // Burst finished: snap to the minimum moving speed, then let normal ramping resume
                kickstartActiveR = false;
                currentSpeedR = (targetSpeedR > 0) ? MIN_MOVING_PWM : -MIN_MOVING_PWM;
                
                // Safety check in case the commanded target speed is very low
                if (abs(targetSpeedR) < MIN_MOVING_PWM) currentSpeedR = targetSpeedR; 
                
                setMotorHardware((int)currentSpeedR, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_RIGHT_EN, MOTOR_RIGHT_INVERT);
            }
        } else if (updateHardware) {
            setMotorHardware((int)currentSpeedR, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_RIGHT_EN, MOTOR_RIGHT_INVERT);
        }
    }
}

//Manual Commands
void stopBot()
{
    targetSpeedL = 0;
    targetSpeedR = 0;
}

void moveForward(int speed) 
{
    targetSpeedL = speed;
    targetSpeedR = speed;
}

void moveBackward(int speed) 
{
    targetSpeedL = -speed;
    targetSpeedR = -speed;
}

void turnLeft(int speed) 
{
    // Left motor forward (gearbox back), Right motor back (gearbox fwd)
    targetSpeedL = speed;
    targetSpeedR = -speed;
}

void turnRight(int speed) 
{
    // Left motor back (gearbox fwd), Right motor fwd (gearbox back)
    targetSpeedL = -speed;
    targetSpeedR = speed;
}

//Grid Commands w/ Ramping
void driveStraight(int speed)
{
    targetSpeedL = speed;
    targetSpeedR = speed;
}

void turnClockwise(int speed) //turn right in grid mode
{
    //right motor fwd gearbox back, left motor back gearbox fwd
    targetSpeedL = -speed;
    targetSpeedR = speed;
}

void turnCounterClockwise(int speed) //turn left in grid mode
{
    //left gearbox back (left motor fwd) right gearbox fwd (right motor back)
    targetSpeedL = speed;
    targetSpeedR = -speed;
}