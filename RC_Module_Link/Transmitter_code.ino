#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//// --- RF24 setup ---
const uint64_t my_radio_pipe = 0xE8E8F0F0E1LL; // same on the receiver
RF24 radio(9, 10);

struct Data_to_be_sent{
  byte ch1;
  byte ch2;
  byte ch3;
  byte ch4;
};

struct Telemetry {
  int16_t roll_cdeg;    // 0.1° units (e.g., 123 => 12.3°)
  int16_t pitch_cdeg;
  int16_t yaw_cdeg;
}; 
// --- Data packet (<=32 bytes for nRF24) ---
Data_to_be_sent sent_data;

int cal_ch1_mid = 0, cal_ch2_mid = 0, cal_ch3_mid = 0, cal_ch4_mid = 0;
const int MAX_VALUE = 1023;
const int DEAD_ZONE = 5;
const int tolerance = 3;

void setup() {
  Serial.begin(115200);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A4,INPUT);


  cal_ch1_mid = analogRead(A0);
  cal_ch2_mid = analogRead(A1);
  cal_ch3_mid = analogRead(A2);
  cal_ch4_mid = analogRead(A4);
  
  // Radio
  radio.begin();
  delay(5);
  radio.setPALevel(RF24_PA_LOW,false);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(100);
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.setRetries(5, 15);
// radio.enableDynamicPayloads();     // optional if RX also enables
  radio.openWritingPipe(my_radio_pipe);
  radio.stopListening();
  radio.flush_tx();
  // Default mid
  sent_data.ch1 = 127;
  sent_data.ch2 = 127;
  sent_data.ch3 = 127;
  sent_data.ch4 = 127;
 

  // Simple, one-time calibration at boot
}
void joystick_mapping(){
  int value_1 = analogRead(A0);//x
  int value_2 = analogRead(A1);//y
  int value_3 = analogRead(A2);//x joystick
  int value_4 = analogRead(A4);//y

  if (abs(value_1 - cal_ch1_mid) < DEAD_ZONE) {//for example if joystick reads 505-505 that is zero and less then 5 so force to 127
   sent_data.ch1 = 127; // Force to neutral value which maps to 0 - 255 to send over RC communcation one btye
  } else if (value_1 > cal_ch1_mid) {//if the value for channel one is greater than the mid point sampled earlier, map the center value plus the deadzone - 1023 to 127 which is the mid point to 255
   sent_data.ch1 = map(value_1, cal_ch1_mid + DEAD_ZONE, (MAX_VALUE - tolerance), 127, 255);//then map t
  } else {//if the joystick is pulled in the opposite direction in which is less than the center going towards zero
    sent_data.ch1 = map(value_1, 0, cal_ch1_mid - DEAD_ZONE, 0, 127);// start from zero and go up to the center value minus the deadzone for center and map to 0 - 127
  }

  // --- Channel 2 with Dead Zone ---
  if (abs(value_2 - cal_ch2_mid) < DEAD_ZONE) {
    sent_data.ch2 = 127;
  } else if (value_2 > cal_ch2_mid) {
    sent_data.ch2 = map(value_2, cal_ch2_mid + DEAD_ZONE, (MAX_VALUE - tolerance), 127, 255);
  } else {
    sent_data.ch2 = map(value_2, 0, cal_ch2_mid - DEAD_ZONE, 0, 127);
  }

  if (abs(value_3 - cal_ch3_mid) < DEAD_ZONE) {//for example if joystick reads 505-505 that is zero and less then 5 so force to 127
   sent_data.ch3 = 127; // Force to neutral value which maps to 0 - 255 to send over RC communcation one btye
  } else if (value_3 > cal_ch3_mid) {//if the value for channel one is greater than the mid point sampled earlier, map the center value plus the deadzone - 1023 to 127 which is the mid point to 255
   sent_data.ch3 = map(value_3, cal_ch3_mid + DEAD_ZONE, (MAX_VALUE - tolerance), 127, 255);//then map t
  } else {//if the joystick is pulled in the opposite direction in which is less than the center going towards zero
    sent_data.ch3 = map(value_3, 0, cal_ch3_mid - DEAD_ZONE, 0, 127);// start from zero and go up to the center value minus the deadzone for center and map to 0 - 127
  }

  // --- Channel 2 with Dead Zone ---
  if (abs(value_4 - cal_ch4_mid) < DEAD_ZONE) {
    sent_data.ch4 = 127;
  } else if (value_4 > cal_ch4_mid) {
    sent_data.ch4 = map(value_4, cal_ch4_mid + DEAD_ZONE, (MAX_VALUE - tolerance), 127, 255);
  } else {
    sent_data.ch4 = map(value_4, 0, cal_ch4_mid - DEAD_ZONE, 0, 127);
  }
}

void loop() {
  // Read, map to 0..255 using calibration
  joystick_mapping();
  

  // Send
  radio.write(&sent_data, sizeof(Data_to_be_sent));
  if (radio.isAckPayloadAvailable()) {
    Telemetry t;
    radio.read(&t, sizeof(t));
  // e.g., print:
    Serial.print(" Roll: "); Serial.print(t.roll_cdeg/10.0);
    Serial.print("  Pitch: "); Serial.print(t.pitch_cdeg/10.0);
    Serial.print("  Yaw: "); Serial.println(t.yaw_cdeg/10.0);
  }

}

