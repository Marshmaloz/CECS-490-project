#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <FaBo9Axis_MPU9250.h>
#include <PID_v1.h>
#include "imu.h"
#include "joystick.h"   // gives you Received_data + externs
#include "MS5611.h"

const uint64_t pipeIn = 0xE8E8F0F0E1LL;   // same as transmitter
RF24 radio(9, 10);                        // CSN, CE
MS5611 MS5611(0x77);
// Single instance of the packet we receive
Received_data received_data;

Servo ChannelOne_One;
Servo ChannelOne_Two;
Servo ChannelThree_One;
Servo ChannelFour_One;

struct Telemetry {
  int16_t roll;     // example
  int16_t pitch;
  int16_t yaw; // battery in mV
  int16_t M1;
  int16_t M2;
  int16_t M3;
  int16_t M4;
};

double rollSet = 0,  rollInput = 0,  rollOut = 0;
double pitchSet = 0, pitchInput = 0, pitchOut = 0;
double yawSet = 0, yawInput =0, yawOut = 0;

// Tunings (start here; you’ll tune later)
double Kp_r = 3.0, Ki_r = 0.0, Kd_r = 0.03;
double Kp_p = 3.0, Ki_p = 0.0, Kd_p = 0.03;
double Kp_y = 3.0, Ki_y = 0.0, Kd_y = 0.03;

const double MAX_YAW_RATE = 150.0;
const int IDLE_THRESH = 1030;//minimium motor thorttle threshold
const double MAX_ANGLE = 20.0;  // deg (start small)

int m1,m2,m3,m4;//store the motors PWM values
float pressure_bias;
float Avg;
// Create controllers
PID rollPID (&rollInput,  &rollOut,  &rollSet,  Kp_r, Kp_r ? Ki_r : 0, Kd_r, DIRECT);
PID pitchPID(&pitchInput, &pitchOut, &pitchSet, Kp_p, Kp_p ? Ki_p : 0, Kd_p, DIRECT);
PID yawPID  (&yawInput,   &yawOut,   &yawSet,   Kp_y, Kp_y ? Ki_y : 0, Kd_y, DIRECT);// Mixer limits (microseconds of correction added to throttle)
const int MIX_LIMIT = 350;  // keep headroom so you don’t clip at 2000/1000
unsigned long last_Time = 0;

void receive_the_data() {
  if (radio.available()) {
    radio.read(&received_data, sizeof(Received_data));
    Telemetry t;
    t.roll  = (int16_t)(roll_Input * 10);
    t.pitch = (int16_t)(pitch_Input * 10);
    t.yaw   = (int16_t)(yaw_Input   * 10);\
    t.M1 = (m1);
    t.M2 = (m2);
    t.M3 = (m3);
    t.M4 = (m4);
    radio.writeAckPayload(1, &t, sizeof(t));
    last_Time = millis();
  }
}

void Pressure_calibration(){
  for(int i = 0; i < 10; i++){
     MS5611.read();
     Avg += MS5611.getAltitude();
     pressure_bias = (Avg/10.0f);
  }
}
void setup() {
  Serial.begin(115200);
  //attach the servo motor pulses to a digital Pin
  ChannelOne_One.attach(2);
  ChannelOne_Two.attach(3);
  ChannelThree_One.attach(4);
  ChannelFour_One.attach(5);

  // arm the ESCs pass in all the servo motor pluses
  armESCs(ChannelOne_One, ChannelOne_Two, ChannelThree_One, ChannelFour_One);

  // start with midpoints
  received_data.ch1 = 127;
  received_data.ch2 = 127;
  received_data.ch3 = 127;
  received_data.ch4 = 127;

  // radio setup
  radio.begin();
  delay(5);                            // let it wake up
  radio.setPALevel(RF24_PA_LOW,false);       // try LOW or MEDIUM first
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(100);               // or 76 / 108
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.setRetries(5, 15);             // delay,count  (0-15 each) => ~1500 µs, 15 tries
// radio.enableDynamicPayloads();     // optional if both ends enable
  radio.openReadingPipe(1, pipeIn);
  radio.flush_rx();
  radio.startListening();

  receive_the_data();   // sample once

  // initialize joystick mids (these externs are defined in joystick.cpp)
  mid_Ch1 = received_data.ch1;
  mid_Ch2 = received_data.ch2;
  mid_Ch3 = received_data.ch3;
  mid_Ch4 = received_data.ch4;

  // IMU
  initIMU();
  calibrateIMU();
  delay(100);

  //init MS5611
  MS5611.begin();
  Pressure_calibration();//calibrate

  zeroYaw_init();

  rollPID.SetOutputLimits(-MIX_LIMIT, MIX_LIMIT);
  pitchPID.SetOutputLimits(-MIX_LIMIT, MIX_LIMIT);
  yawPID.SetOutputLimits(-MIX_LIMIT, MIX_LIMIT);
// ~200 Hz loop target (adjust if your loop is slower/faster)
  rollPID.SetSampleTime(5);      // ms
  pitchPID.SetSampleTime(5);     // ms
  yawPID.SetSampleTime(5);

  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);
  yawPID.SetMode(AUTOMATIC);
}

void loop() {
  receive_the_data();

  // map joystick bytes -> 1000..2000 µs (updates extern ch*_updated)
  joystick_Servo_Signal(received_data);

  // IMU update
  read_IMU_and_Calculate_Angles();

  rollSet  = constrain((ch3_updated - 1500) * (MAX_ANGLE / 500.0), -MAX_ANGLE, MAX_ANGLE);
  pitchSet = constrain((ch4_updated - 1500) * (MAX_ANGLE / 500.0), -MAX_ANGLE, MAX_ANGLE);
  yawSet = constrain((ch1_updated - 1500) * (MAX_YAW_RATE / 500.0), -MAX_YAW_RATE, MAX_YAW_RATE);
  
  // --- 3. Integrate Rate into Angle Setpoint (Yaw Angle Hold Logic) ---
  rollInput  = roll_Input;   // deg
  pitchInput = pitch_Input;  // deg
  yawInput = yaw_Input;
// Run PIDs (compute corrections in microseconds-worth of authority)
  rollPID.Compute();
  pitchPID.Compute();
  yawPID.Compute();

  int throttle = ch2_updated;

  if(throttle <= IDLE_THRESH){
    m1 = throttle;
    m2 = throttle;
    m3 = throttle;
    m4 = throttle;
  }else{
    m1 = throttle + pitchOut + rollOut + yawOut;   // front-left
    m2 = throttle + pitchOut - rollOut - yawOut;   // front-right
    m3 = throttle - pitchOut - rollOut + yawOut;   // rear-right
    m4 = throttle - pitchOut + rollOut - yawOut;   // rear-left
  }

// Constrain to ESC range
  m1 = constrain(m1, 1000, 2000);
  m2 = constrain(m2, 1000, 2000);
  m3 = constrain(m3, 1000, 2000);
  m4 = constrain(m4, 1000, 2000);
  // drive servos with the mapped values
  ChannelOne_One.writeMicroseconds(m1);
  ChannelOne_Two.writeMicroseconds(m2);
  ChannelThree_One.writeMicroseconds(m3);
  ChannelFour_One.writeMicroseconds(m4);

  MS5611.read();
  Serial.println(MS5611.getAltitude()-pressure_bias);
/*
  Serial.print("Ch1: ");
  Serial.print(roll_Input);
  Serial.print("   Ch2: ");
  Serial.print(pitch_Input);
  Serial.print("     Ch3: ");
  Serial.print(yaw_Input);
  Serial.print("        Ch4: ");
  Serial.println(m4);
*/

}
