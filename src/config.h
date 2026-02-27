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

//****NEED TO FIX CURRENTLY TURNING CODE IS ACTUALLY MOVING STRAIGHT AND STRAIGHT CODE IS THE TURN*/
//Implement a speed control slider to manual control mode so that we can test different speeds without reflashing

//Robot Physical Parameters (I NEED TO UPDATE THESE)
#define TICKS_PER_FOOT 282
#define TICKS_PER_TURN 145
#define SPEED_GRID_FWD 180
#define SPEED_GRID_TURN 180
