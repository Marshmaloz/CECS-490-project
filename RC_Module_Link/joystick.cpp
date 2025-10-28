#include "joystick.h"

int ch1_value = 0, ch2_value = 0, ch3_value = 0, ch4_value = 0;
int ch1_updated = 0, ch2_updated = 0, ch3_updated = 0, ch4_updated = 0;
int mid_Ch1 = 0, mid_Ch2 = 0, mid_Ch3 = 0, mid_Ch4 = 0;
const int tolerance = 3;

void joystick_Servo_Signal(const Received_data &received_data) {
  ch1_value = received_data.ch1;
  ch2_value = received_data.ch2;
  ch3_value = received_data.ch3;
  ch4_value = received_data.ch4;

  if (abs(ch1_value - mid_Ch1) < tolerance)
    ch1_updated = 1500;
  else if (ch1_value > mid_Ch1)
    ch1_updated = map(ch1_value, mid_Ch1 + tolerance, 255, 1500, 2000);
  else
    ch1_updated = map(ch1_value, 0, mid_Ch1 - tolerance, 1000, 1500);

  if (abs(ch2_value - mid_Ch2) < tolerance)
    ch2_updated = 1000;
  else if (ch2_value > mid_Ch2)
    ch2_updated = map(ch2_value, mid_Ch2 + tolerance, 255, 1000, 2000);
  else
    ch2_updated = map(ch2_value, 0, mid_Ch2 - tolerance, 1000, 1000);

  if (abs(ch3_value - mid_Ch3) < tolerance)
    ch3_updated = 1500;
  else if (ch3_value > mid_Ch3)
    ch3_updated = map(ch3_value, mid_Ch3 + tolerance, 255, 1500, 2000);
  else
    ch3_updated = map(ch3_value, 0, mid_Ch3 - tolerance, 1000, 1500);

  if (abs(ch4_value - mid_Ch4) < tolerance)
    ch4_updated = 1500;
  else if (ch4_value > mid_Ch4)
    ch4_updated = map(ch4_value, mid_Ch4 + tolerance, 255, 1500, 2000);
  else
    ch4_updated = map(ch4_value, 0, mid_Ch4 - tolerance, 1000, 1500);
}

void armESCs(Servo& ch1, Servo& ch2, Servo& ch3, Servo& ch4) {
  for (int i = 0; i < 200; i++) {
    ch1.writeMicroseconds(1000);
    ch2.writeMicroseconds(1000);
    ch3.writeMicroseconds(1000);
    ch4.writeMicroseconds(1000);
    delay(10);
  }
}
