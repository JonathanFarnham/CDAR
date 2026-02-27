#pragma once
#include <Arduino.h>

//Wifi Settings
//IP for site is 192.168.4.1
#define WIFI_SSID "CDAR_Control"
#define WIFI_PASS "password"

//Left Motor
#define MOTOR_LEFT_EN 13
#define MOTOR_LEFT_IN1 12
#define MOTOR_LEFT_IN2 14
#define ENCODER_LEFT_A 21
#define ENCODER_LEFT_B 22

//Right Motor
#define MOTOR_RIGHT_EN 27
#define MOTOR_RIGHT_IN1 32
#define MOTOR_RIGHT_IN2 33
#define ENCODER_RIGHT_A 25
#define ENCODER_RIGHT_B 26

//Motor Speeds
#define SPEED_MOVE 80
#define SPEED_TURN 140

//Motor Kickstart
#define KICKSTART_PWM 180
#define KICKSTART_DURATION 20
#define MIN_MOVING_PWM 20

//Motor Inversion Controls
#define MOTOR_LEFT_INVERT false //change to true if left wheel drives backwards when asked to go forward
#define MOTOR_RIGHT_INVERT false //change to true if the right wheel drives backwards when asked to go forward

//Robot Physical Parameters (I NEED TO UPDATE THESE)
#define TICKS_PER_FOOT 282
#define TICKS_PER_TURN 145
#define SPEED_GRID_FWD 180
#define SPEED_GRID_TURN 180
