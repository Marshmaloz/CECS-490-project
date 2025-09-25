#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//// --- RF24 setup ---
const uint64_t my_radio_pipe = 0xE8E8F0F0E1LL; // same on the receiver
RF24 radio(9, 10);


// --- Data packet (<=32 bytes for nRF24) ---
struct Data_to_be_sent {
  byte ch1;
  byte ch2;
};
Data_to_be_sent sent_data;
const int MAX_VALUE = 1023;
const int DEAD_ZONE = 5;
const int tolerance = 3;

int cal_ch1_mid = 0;
int cal_ch2_mid = 0;

void setup() {
  Serial.begin(9600);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);

  cal_ch1_mid = analogRead(A0);
  cal_ch2_mid = analogRead(A1);
  // Radio
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(my_radio_pipe);

  // Default mid
  sent_data.ch1 = 127;
  sent_data.ch2 = 127;

  // Simple, one-time calibration at boot
}
void loop() {
  // Read, map to 0..255 using calibration

  // Set reversed=true if you want to invert that axis
  int value_1 = analogRead(A0);
  int value_2 = analogRead(A1);

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

  // Send
  radio.write(&sent_data, sizeof(Data_to_be_sent));

  // Debug prinitng statments
  Serial.print("ch1: "); 
  Serial.print( sent_data.ch1 );
  Serial.print("   ch2: "); 
  Serial.println(sent_data.ch2);

}
