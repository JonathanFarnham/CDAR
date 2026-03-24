#include "drive_system.h"
#include "motor_hardware.h"
#include "pid_controller.h"
#include "config.h"

//State Variable
float targetRPM_L = 0; //Final Target Speed
float targetRPM_R = 0;
float activeTargetRPM_L = 0; //The current speed PID is chasing
float activeTargetRPM_R = 0;
float currentRPM_L = 0;
float currentRPM_R = 0;
//Ramp settings -> step RPM per calc cycle
const float RAMP_STEP_RPM = 30.0;

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
    activeTargetRPM_L = 0;
    activeTargetRPM_R = 0;
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

        //Ramping Start Logic ----------->
        // Left Motor Ramp
        if (abs(targetRPM_L - activeTargetRPM_L) <= RAMP_STEP_RPM) 
        {
            activeTargetRPM_L = targetRPM_L; // Snap to target if close
        } else if (targetRPM_L > activeTargetRPM_L) 
        {
            activeTargetRPM_L += RAMP_STEP_RPM; // Ramp up
        } else 
        {
            activeTargetRPM_L -= RAMP_STEP_RPM; // Ramp down
        }

        // Right Motor Ramp
        if (abs(targetRPM_R - activeTargetRPM_R) <= RAMP_STEP_RPM) 
        {
            activeTargetRPM_R = targetRPM_R;
        } else if (targetRPM_R > activeTargetRPM_R) 
        {
            activeTargetRPM_R += RAMP_STEP_RPM;
        } else 
        {
            activeTargetRPM_R -= RAMP_STEP_RPM;
        }
        
        /*
        //Cross Coupling to Reduce Straight Line Drift
        if (targetRPM_L == targetRPM_R && targetRPM_L > 0) //Only Apply to straight line command
        {
            //Calculate difference in ticks accumulated between sides
            long tickDiff = getTicksLeft() - getTicksRight();

            //Proportional Correction
            float syncKp = 1.5;
            float syncAdjustment = tickDiff * syncKp;
            syncAdjustment = constrain(syncAdjustment, -40, 40);

            //Apply correction to active target
            activeTargetRPM_L -= syncAdjustment;
            activeTargetRPM_R += syncAdjustment;
        }
        */

        //PID CONTROL---------------------------->
        int outputL = 0;
        int outputR = 0;
        
        //Left Motor
        if (targetRPM_L == 0) // Always check the final target for hard stops
        {
            outputL = 0;
            activeTargetRPM_L = 0;
            pidL.reset();
        } else 
        {
            // Feed the ACTIVE target into the PID, not the final target
            outputL = (int)pidL.compute(activeTargetRPM_L, currentRPM_L, dt_sec);
        }

        //Right Motor
        if (targetRPM_R == 0)
        {
            outputR = 0;
            activeTargetRPM_R = 0;
            pidR.reset();
        } else
        {
            outputR = (int)pidR.compute(activeTargetRPM_R, currentRPM_R, dt_sec);
        }

        //Constrain Output ---------------->
        outputL = constrain(outputL, -200, 200);
        outputR = constrain(outputR, -200, 200);

        setMotorRaw(outputL, outputR);
    }
}

float getCurrentRPMLeft() { return currentRPM_L; }
float getCurrentRPMRight() { return currentRPM_R; }

float getTargetRPMLeft() { return activeTargetRPM_L; }
float getTargetRPMRight() { return activeTargetRPM_R; }