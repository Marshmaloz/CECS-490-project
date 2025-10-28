#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>

// Externs provided by imu.cpp
extern float roll_Input;
extern float pitch_Input;
extern float yaw_Input;

// API
void initIMU();
void calibrateIMU();
void read_IMU_and_Calculate_Angles();
void zeroYaw_init();
#endif // IMU_H
