#include <ros.h>
#include <sensor_msgs/Range.h>//PING msgs
#include <std_msgs/Float64.h>//Motor msg
#include <std_msgs/Bool.h>
#include <Wire.h>
#include "AHRSProtocol.h"             // navX-Sensor Register Definition header file

byte data[8];

#define ITERATION_DELAY_MS                   0
#define NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT  0x32
#define NUM_BYTES_TO_READ                    8

ros::NodeHandle nh;

int ENCA[]={19,2};//right left
int ENCB[]={18,3};
int pos[]={0,0};

int MotorSpeed [] = {6,4,5}; //Right,Back,Left
int MotorDirectionRight [] = {29,22,26};//Right,Back,Left
int MotorDirectionLeft [] = {28,25,27};//Right,Back,Left

int PINGEcho[]={42,44,46};//left,front/right
int PINGTrig[]={43,45,47};
char *pingString[] = {" Left ","Front ", " Right "};


sensor_msgs::Range rangeR_msg;
sensor_msgs::Range rangeL_msg;
sensor_msgs::Range rangeF_msg;

std_msgs::Float64 yaw_msg;

std_msgs::Float64 ENCR_msg;
std_msgs::Float64 ENCL_msg;
ros::Publisher pub_yaw ("yaw", &yaw_msg);

ros::Publisher pub_ENCR_POS ("ENCR_POS", &ENCR_msg);
ros::Publisher pub_ENCL_POS ("ENCL_POS", &ENCL_msg);

ros::Publisher pub_range_left ("range_left_ping", &rangeL_msg);
ros::Publisher pub_range_right ("range_right_ping", &rangeR_msg);
ros::Publisher pub_range_front ("range_front_ping", &rangeF_msg);

  // switch (j){
  //   case 0:
  //   ENCR_msg.data=pos[j];
  //   pub_ENCR_POS.publish (&ENCR_msg);
  //   //Serial.println("Cas0");
  //   break;
  //    case 1:
  //   ENCL_msg.data=pos[j];
  //   pub_ENCR_POS.publish (&ENCL_msg);
  //  //Serial.println("Cas1");
  //   break;
  // }
void EncToZero (const std_msgs::Bool &msg){
  if (msg.data==true){
  pos [0] = 0;
  pos [1] = 0;
  }
}

void messageCdRight (const std_msgs::Float64 &msg){
  if (msg.data<0) { 
  analogWrite(MotorSpeed [0], abs(msg.data));
  digitalWrite( MotorDirectionRight [0], HIGH);
  digitalWrite(MotorDirectionLeft [0], LOW);
  }
   if (msg.data>0) { 
  analogWrite(MotorSpeed [0],abs(msg.data));
  digitalWrite(MotorDirectionLeft [0], HIGH);
  digitalWrite(MotorDirectionRight [0], LOW);
  }
   if (msg.data==0) { 
  analogWrite(MotorSpeed [0],abs(msg.data));
  digitalWrite(MotorDirectionLeft [0], LOW);
  digitalWrite(MotorDirectionRight [0], LOW);
  }
};
void messageCdLeft (const std_msgs::Float64 &msg){
 if (msg.data<0) { 
  analogWrite(MotorSpeed [2], abs(msg.data));
  digitalWrite( MotorDirectionRight [2], HIGH);
  digitalWrite(MotorDirectionLeft [2], LOW);
  }
   if (msg.data>0) { 
  analogWrite(MotorSpeed [2], abs(msg.data));
  digitalWrite(MotorDirectionLeft [2], HIGH);
  digitalWrite(MotorDirectionRight [2], LOW);
  }
    if (msg.data==0) {   
  analogWrite(MotorSpeed [2],abs(msg.data));
  digitalWrite(MotorDirectionLeft [2], LOW);
  digitalWrite(MotorDirectionRight [2], LOW);
  }
};
void messageCdBack (const std_msgs::Float64 &msg){
  if (msg.data<0) { 
  analogWrite(MotorSpeed [1], abs(msg.data));
  digitalWrite( MotorDirectionRight [1], HIGH);
  digitalWrite(MotorDirectionLeft [1], LOW);
  }
   if (msg.data>0) { 
  analogWrite(MotorSpeed [1], abs(msg.data));
  digitalWrite(MotorDirectionLeft [1], HIGH);
  digitalWrite(MotorDirectionRight [1], LOW);
  }
    if (msg.data==0) { 
  analogWrite(MotorSpeed [1], abs(msg.data));
  digitalWrite(MotorDirectionLeft [1], LOW);
  digitalWrite(MotorDirectionRight [1], LOW);
  }
};


ros::Subscriber<std_msgs::Bool> ENCZero("ENC_zero", &EncToZero);
 ros::Subscriber<std_msgs::Float64> subR("v_right", &messageCdRight);
 ros::Subscriber<std_msgs::Float64> subL("v_left", &messageCdLeft);
 ros::Subscriber<std_msgs::Float64> subB("v_back", &messageCdBack);

void setup() {

   Wire.begin(); // join i2c bus (address optional for master)

  for ( int i = 0; i < sizeof(data); i++ ) {
      data[i] = 0;
  }

nh.initNode();
//attachInterrupt(digitalPinToInterrupt(21),readEncoder<0>,RISING);
//attachInterrupt(digitalPinToInterrupt(19),readEncoder<1>,RISING);
nh.subscribe(ENCZero);
nh.subscribe(subL);
nh.subscribe(subR);
nh.subscribe(subB);
nh.advertise(pub_ENCR_POS);
nh.advertise(pub_yaw);
nh.advertise(pub_ENCL_POS);
nh.advertise(pub_range_front);
nh.advertise(pub_range_left);
nh.advertise(pub_range_right);
// Serial.begin(9600);
unsigned long echo;

for(int i=0; i < 3; i++) {
pinMode(MotorDirectionRight[i], OUTPUT);
pinMode(MotorDirectionLeft[i], OUTPUT);
pinMode(MotorSpeed[i], OUTPUT);
digitalWrite(MotorDirectionLeft[i], LOW);
digitalWrite(MotorDirectionRight[i], LOW);
pinMode(PINGTrig[i], OUTPUT);
pinMode(PINGEcho[i], INPUT);
}
for(int i=0; i < 2; i++) {
pinMode(ENCA[i], INPUT);
pinMode(ENCB[i], INPUT);
  }

}

int register_address = NAVX_REG_YAW_L;

unsigned long ping(int index){
unsigned long echo,cm;

  digitalWrite(PINGTrig[index], LOW);
  delayMicroseconds(5);
  digitalWrite(PINGTrig[index], HIGH);


  // Выставив высокий уровень сигнала, ждем около 10 микросекунд. В этот момент датчик будет посылать сигналы с частотой 40 КГц.
  delayMicroseconds(10);
  digitalWrite(PINGTrig[index], LOW);
  echo = pulseIn(PINGEcho[index], HIGH);
  cm = (echo / 2) / 29.1; 
    switch (index){
    case 0:
    rangeL_msg.range=cm;
    pub_range_left.publish (&rangeL_msg);
    break;
  case 1:
    rangeF_msg.range=cm;
    pub_range_front.publish (&rangeF_msg);
    break;
  case 2:
    rangeR_msg.range=cm;
    pub_range_right.publish (&rangeR_msg);
    break;
  }
  return cm ; //convert to CM then to inches

}
//Ping loop
void loop() {
  unsigned long ultrasoundValue;
  for(int i=0; i < 3; i++){
    ping(i); 

  }
  int i = 0;
  /* Transmit I2C data request */
  Wire.beginTransmission(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT); // Begin transmitting to navX-Sensor
  Wire.write(register_address);                                // Sends starting register address
  Wire.write(NUM_BYTES_TO_READ);                               // Send number of bytes to read
  Wire.endTransmission();                                      // Stop transmitting
  
  /* Receive the echoed value back */
  Wire.beginTransmission(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT); // Begin transmitting to navX-Sensor
  Wire.requestFrom(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT, NUM_BYTES_TO_READ);    // Send number of bytes to read

  while(Wire.available()) {                                    // Read data (slave may send less than requested)
     data[i++] = Wire.read();
  }
  Wire.endTransmission();                                      // Stop transmitting

  /* Decode received data to floating-point orientation values */
  yaw_msg.data =     IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&data[0]);   // The cast is needed on arduino

  pub_yaw.publish (&yaw_msg);

ENCL_msg.data=pos[0];
pub_ENCL_POS.publish (&ENCL_msg);
ENCR_msg.data=pos[1];
pub_ENCR_POS.publish (&ENCR_msg);
  nh.spinOnce(); 

}

template <int j>
void readEncoder(){

  int b = digitalRead(ENCB[j]);
  if(b > 0){
    pos[j]++;
    
  }
  else{
    pos[j]--;
  }
}
