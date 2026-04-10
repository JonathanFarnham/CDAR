#include "mpu_handler.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "config.h"

Adafruit_MPU6050 mpu;

//Volatile flag for the interrupt
volatile bool mpuDataReady = false;

float currentYaw = 0.0;
unsigned long lastMPUTime = 0;
float gyroZ_offset = 0.0;

//ISR
void IRAM_ATTR dmpDataReady()
{
    mpuDataReady = true;
}

void initMPU()
{
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050");
        return;
    }

    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    //Lower sample rate to free up CPU
    mpu.setSampleRateDivisor(9);

    pinMode(MPU_INT_PIN, INPUT);

    Wire.beginTransmission(0x68); 
    Wire.write(0x38); 
    Wire.write(0x01); 
    Wire.endTransmission();

    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
    
    lastMPUTime = micros();
}

void calibrateMPU()
{
    Serial.println("Calibrating gyro... DO NOT MOVE");
    float sumZ = 0;
    
    // Clear any pending interrupts
    mpuDataReady = false; 

    for(int i = 0; i < 200; i++)
    {
        unsigned long waitStart = millis();
        while(!mpuDataReady && (millis() - waitStart < 20)) { yield(); }
        mpuDataReady = false;

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        sumZ += g.gyro.z;
    }
    gyroZ_offset = sumZ / 200.0;
    
    // Reset yaw and timer right after calibration finishes
    currentYaw = 0.0;
    lastMPUTime = micros();
    mpuDataReady = false;
    
    Serial.print("Calibration complete. Offset: ");
    Serial.println(gyroZ_offset);
}

void updateMPU()
{
    //Only Execute the slow I2C read if told
    if (mpuDataReady)
    {
        mpuDataReady = false; //Reset flag

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp); //Clear MPU internal interrupt state

        unsigned long now = micros();
        float dt = (now - lastMPUTime) / 1000000.0; //Delta time in seconds
        lastMPUTime = now;

        float zDegSec = (g.gyro.z - gyroZ_offset) * 57.2958;

        if (abs(zDegSec) > 1.0)
        {
            currentYaw += zDegSec * dt;
        }
    }
}

float getYawAngle()
{
    return currentYaw;
}

void resetYaw()
{
    currentYaw = 0.0;
    lastMPUTime = micros(); //Reset Timer so dt doesnt spike
    mpuDataReady = false; //clear interrupts triggered during delay
}