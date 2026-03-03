#include "drive_system.h"
#include "motor_hardware.h"
#include "pid_controller.h"
#include "config.h"

//State Variable
float targetRPM_L = 0;
float targetRPM_R = 0;
float currentRPM_L = 0;
float currentRPM_R = 0;

//PID Instances
PIDController pidL(PID_KP, PID_KI, PID_KD);
PIDController pidR(PID_KP, PID_KI, PID_KD);

//Velocity Calculation
unsigned long lastCalcTime = 0;
long lastTicks_L = 0;
long lastTicks_R = 0;

void initDriveSystem()
{
    initMotorHardware();
    pidL.reset();
    pidR.reset();
}

void setTargetRPM(float leftRPM, float rightRPM)
{
    targetRPM_L = leftRPM;
    targetRPM_R = rightRPM;
}

void stopAll()
{
    targetRPM_L = 0;
    targetRPM_R = 0;
    pidL.reset();
    pidR.reset();
    setMotorRaw(0, 0);
}

void updateDriveSystem()
{
    unsigned long now = millis();
    long dt_ms = now - lastCalcTime;

    //Run strictly at the defined interval
    if (dt_ms >= CALC_INTERVAL)
    {
        lastCalcTime = now;
        float dt_sec = dt_ms / 1000.0;

        //Calculate Current RPM------------------>
        long currTicksL = getTicksLeft();
        long currTicksR = getTicksRight();

        long deltaL = currTicksL - lastTicks_L;
        long deltaR = currTicksR - lastTicks_R;

        lastTicks_L = currTicksL;
        lastTicks_R = currTicksR;

        //calculate raw RPM
        float rawRPM_L = (deltaL / dt_sec) * 60 / COUNTS_PER_REV;
        float rawRPM_R = (deltaR / dt_sec) * 60.0 / COUNTS_PER_REV;

        //Low Pass Filter
        currentRPM_L = (currentRPM_L * 0.7) + (rawRPM_L * 0.3);
        currentRPM_R = (currentRPM_R * 0.7) + (rawRPM_R * 0.3);

        //PID CONTROL---------------------------->
        int outputL = 0;
        int outputR = 0;
        
        //Left Motor
        if (targetRPM_L == 0)
        {
            outputL = 0;
            pidL.reset();
        } else 
        {
            outputL = (int)pidL.compute(targetRPM_L, currentRPM_L, dt_sec);
        }

        //Right Motor
        if (targetRPM_R == 0)
        {
            outputR = 0;
            pidR.reset();
        } else
        {
            outputR = (int)pidR.compute(targetRPM_R, currentRPM_R, dt_sec);
        }

        //Constrain Output ---------------->
        outputL = constrain(outputL, -255, 255);
        outputR = constrain(outputR, -255, 255);

        setMotorRaw(outputL, outputR);
    }
}

float getCurrentRPMLeft() { return currentRPM_L; }
float getCurrentRPMRight() { return currentRPM_R; }

float getTargetRPMLeft() { return targetRPM_L; }
float getTargetRPMRight() { return targetRPM_R; }