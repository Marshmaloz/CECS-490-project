#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

const uint64_t pipeIn = 0xE8E8F0F0E1LL;     //Remember that this code is the same as in the transmitter
RF24 radio(9, 10);  //CSN and CE pins

// The sizeof this struct should not exceed 32 bytes
struct Received_data {
  byte ch1;
  byte ch2;
};

int ch1_value = 0;
int ch2_value = 0;

int mid_Ch1 = 0;
int mid_Ch2 = 0;

int ch1_updated = 0;
int ch2_updated = 0;
const int tolerance = 5;

Received_data received_data;

Servo ChannelOne_One;
Servo ChannelOne_Two;


/**************************************************/

void setup()
{
  Serial.begin(9600);

  ChannelOne_One.attach(2);//attaches pwm signal to D2
  ChannelOne_Two.attach(3);

  received_data.ch1 = 127;//x axis
  received_data.ch2 = 127;//y channel

 
  //Once again, begin and radio configuration
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);  
  radio.openReadingPipe(1,pipeIn);
  
  //We start the radio comunication
  radio.startListening();

  receive_the_data(); //call the function after enabling the radio
  //sample both channels once
  mid_Ch1 = received_data.ch1;
  mid_Ch2 = received_data.ch2;

}

/**************************************************/

unsigned long last_Time = 0;

//We create the function that will read the data each certain time
void receive_the_data()
{
  if(radio.available()) {
    radio.read(&received_data, sizeof(Received_data));
    last_Time = millis(); //Here we receive the data
  }
}

/**************************************************/

void loop()
{
  //Receive the radio data
  receive_the_data();
  ch1_value = received_data.ch1;
  ch2_value = received_data.ch2;

  if (abs(ch1_value - mid_Ch1) < tolerance) {
    ch1_updated = 1500;
  } else if (ch1_value > mid_Ch1) {
    ch1_updated = map(ch1_value, mid_Ch1 + tolerance, 255, 1500, 2000);
  } else {
    ch1_updated = map(ch1_value, 0, mid_Ch1 - tolerance, 1000, 1500);
  }
  
  if (abs(ch2_value - mid_Ch2) < tolerance) {
    ch2_updated = 1500;
  } else if (ch2_value > mid_Ch2) {
    ch2_updated = map(ch2_value, mid_Ch2 + tolerance, 255, 1500, 2000);
  } else {
    ch2_updated = map(ch2_value, 0, mid_Ch2 - tolerance, 1000, 1500);
  }

  ChannelOne_One.writeMicroseconds(ch1_updated);
  ChannelOne_Two.writeMicroseconds(ch2_updated);

  Serial.print("ch1: ");
  Serial.print(ch1_updated);
  Serial.print("   ch2: ");
  Serial.println(ch2_updated);
  
}
  
