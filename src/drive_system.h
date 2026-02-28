#pragma once

void initDriveSystem();

void updateDriveSystem(); //Call this immediately in main loop

//Set Targets in RPM
void setTargetRPM(float leftRPM, float rightRPM);
void stopAll();

//Get Telemetry
float getCurrentRPMLeft();
float getCurrentRPMRight();

//Get Targets for Debugging
float getTargetRPMLeft();
float getTargetRPMRight();