#pragma once

void initMotors();
void stopBot();

//Manual Control Functions
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);
void turnRight(int speed);

//Grid Control Functions
void driveStraight(int speed);
void turnCounterClockwise(int speed);
void turnClockwise(int speed);

//Encoders
void resetEncoders();
long getEncoderAvg();

//Ramping Function
void updateMotorSpeeds();