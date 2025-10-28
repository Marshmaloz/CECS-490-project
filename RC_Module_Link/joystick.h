#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>
#include <Servo.h>

// Shared variables (so main can use them)
extern int ch1_value, ch2_value, ch3_value, ch4_value;
extern int ch1_updated, ch2_updated, ch3_updated, ch4_updated;
extern int mid_Ch1, mid_Ch2, mid_Ch3, mid_Ch4;
extern const int tolerance;

// Structure for received data
struct Received_data {
  byte ch1;
  byte ch2;
  byte ch3;
  byte ch4;
};

// Function declarations
void joystick_Servo_Signal(const Received_data &received_data);
void armESCs(Servo& ch1, Servo& ch2, Servo& ch3, Servo& ch4);

#endif
