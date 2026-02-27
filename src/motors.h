#pragma once

void initMotors();
void stopBot();

//Manual Control Functions
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();

//Grid Control Functions
void driveStraight(int speed);
void turnCounterClockwise(int speed);
void turnClockwise(int speed);

//Encoders
void resetEncoders();
long getEncoderAvg();

//Ramping Function
void updateMotorSpeeds();