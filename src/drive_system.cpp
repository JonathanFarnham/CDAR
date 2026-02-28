#include "drive_system.h"
#include "motor_hardware.h"
#include "pid_controller.h"
#include "config.h"

//State Variable
float targetRPM_L = 0;
float targetRPM_R = 0;
float currentRPM_L = 0;
float currentRPM_R = 0;

//Kickstart Flags
bool kickstartActive_L = false;
unsigned long kickstartStart_L = 0;
bool kickstartActive_R = false;
unsigned long kickstartStart_R = 0;

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
}

void setTargetRPM(float leftRPM, float rightRPM)
{
    //If start from 0 trigger kickstart
    if (targetRPM_L == 0 && leftRPM != 0)
    {
        kickstartActive_L = true;
        kickstartStart_L = millis();
        pidL.reset();
    }
    if (targetRPM_R == 0 && rightRPM != 0)
    {
        kickstartActive_R = true;
        kickstartStart_R = millis();
        pidR.reset();
    }

    targetRPM_L = leftRPM;
    targetRPM_R = rightRPM;
}

void stopAll()
{
    targetRPM_L = 0;
    targetRPM_R = 0;
    kickstartActive_L = false;
    kickstartActive_R = false;
    pidL.reset();
    pidR.reset();
    setMotorRaw(0, 0);
}

void updateDriveSystem()
{
    unsigned long now = millis();
    long dt_ms = now - lastCalcTime;

    if (dt_ms >= CALC_INTERVAL)
    {
        lastCalcTime = now;
        float dt_sec = dt_ms / 1000.0;

        // 1. Calculate Current RPM
        long currTicksL = getTicksLeft();
        long currTicksR = getTicksRight();

        long deltaL = currTicksL - lastTicks_L;
        long deltaR = currTicksR - lastTicks_R;

        lastTicks_L = currTicksL;
        lastTicks_R = currTicksR;

        // RPM = (ticks / dt) * (60s / 1m) / (ticks_per_rev)
        float rawRPM_L = (deltaL / dt_sec) * 60.0 / COUNTS_PER_REV;
        float rawRPM_R = (deltaR / dt_sec) * 60.0 / COUNTS_PER_REV;

        // Simple Low Pass Filter to smooth out jitter from low-res encoders
        currentRPM_L = (currentRPM_L * 0.7) + (rawRPM_L * 0.3);
        currentRPM_R = (currentRPM_R * 0.7) + (rawRPM_R * 0.3);

        // 2. Control Logic
        int outputL = 0;
        int outputR = 0;

        // --- LEFT MOTOR CONTROL ---
        if (targetRPM_L == 0)
        {
            outputL = 0;
            kickstartActive_L = false;
        } 
        else if (kickstartActive_L)
        {
            // Kickstart Mode-> Max Power Open Loop
            outputL = (targetRPM_L > 0) ? KICKSTART_PWM : -KICKSTART_PWM;
            if (now - kickstartStart_L > KICKSTART_MS)
            {
                kickstartActive_L = false; // Transition to PID
            }
        } 
        else 
        {
            // PID Mode
            float pidOut = pidL.compute(targetRPM_L, currentRPM_L, dt_sec);
            outputL = (int)pidOut;
        }

        // --- RIGHT MOTOR CONTROL ---
        if (targetRPM_R == 0)
        {
            outputR = 0;
            kickstartActive_R = false;
        } 
        else if (kickstartActive_R)
        {
            outputR = (targetRPM_R > 0) ? KICKSTART_PWM : -KICKSTART_PWM;
            if (now - kickstartStart_R > KICKSTART_MS)
            {
                kickstartActive_R = false;
            }
        } 
        else
        {
            float pidOut = pidR.compute(targetRPM_R, currentRPM_R, dt_sec);
            outputR = (int)pidOut;
        }

        // 3. Constrain and Output
        outputL = constrain(outputL, -255, 255);
        outputR = constrain(outputR, -255, 255);
        
        setMotorRaw(outputL, outputR);
    }
}

float getCurrentRPMLeft() { return currentRPM_L; }
float getCurrentRPMRight() { return currentRPM_R; }

float getTargetRPMLeft() { return targetRPM_L; }
float getTargetRPMRight() { return targetRPM_R; }