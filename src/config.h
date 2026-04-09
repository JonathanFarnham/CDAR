#pragma once
#include <Arduino.h>

//Wifi Settings
//IP for site is 192.168.4.1
#define WIFI_SSID "CDAR_Control"
#define WIFI_PASS "password"

//Left Motor Pin Setup
#define MOTOR_LEFT_EN 13
#define MOTOR_LEFT_IN1 12
#define MOTOR_LEFT_IN2 14
#define ENCODER_LEFT_A 21
#define ENCODER_LEFT_B 22

//Right Motor Pin Setup
#define MOTOR_RIGHT_EN 27
#define MOTOR_RIGHT_IN1 32
#define MOTOR_RIGHT_IN2 33
#define ENCODER_RIGHT_A 25
#define ENCODER_RIGHT_B 26

//Physics and Encoder
#define COUNTS_PER_REV 28
#define MAX_RPM_HARDWARE 5600
#define MAX_OPERATING_RPM 3000

//PID Tuning + Wheel Trim
//KP: proportional value power per error unit
//Ki: Integral (accumulates error to overcome friction)
//kd: derivative (dampens oscillation)
#define PID_KP 0.8
#define PID_KI 0.3
#define PID_KD 0.00
#define CALC_INTERVAL 100 //Calculate velocity every 100ms
#define WHEEL_TRIM 1.03   //Reduces Speed of Left Motor to account for mechanical difference and tolerance

//Kickstart Settings
#define KICKSTART_MS 50
#define KICKSTART_PWM 0
#define MIN_REGULATED_RPM 40

//Inversion
#define MOTOR_LEFT_INVERT false
#define MOTOR_RIGHT_INVERT true

//Robot Parameters (signifigant adjustments needed)
#define TICKS_PER_FOOT 50
#define TICKS_PER_TURN 54

//Grid Speed Values
#define SPEED_GRID_RPM 300
#define SPEED_TURN_RPM 200
