#include <VB_MPU9250.h>
#include <ros.h>
#include <sensor_msgs/Range.h>//PING msgs
#include <std_msgs/Float64.h>//Motor msgs
#include <geometry_msgs/Twist.h> // imu msg
#include <math.h>
#include <Scheduler.h>
VB_MPU9250 imu;

#define sampleFreq 512.0f // sample frequency in Hz
#define betaDef 0.1f      // 2 * proportional gain

int ENCA []={49,51,53}; /// left, back, right
int ENCB []={48,50,52}; /// left, back, right
int posL=0;
int posR=0;
int posB=0;

ros::NodeHandle nh;

int MotorSpeed []={3,4,5}; //Right,Back,Left
int MotorDirectionRight [] = {22,25,29};//Right,Back,Left
int MotorDirectionLeft [] = {24,28,27};//Right,Back,Left

int PINGEcho[]={42,44,46};//left,front/right
int PINGTrig[]={43,45,47};
char *pingString[] = {" Left ","Front ", " Right "};

geometry_msgs::Twist  yaw_msg;
geometry_msgs::Twist  roll_msg;
geometry_msgs::Twist  pitch_msg;
ros::Publisher pub_yaw ("yaw", &yaw_msg);
ros::Publisher pub_roll ("roll", &roll_msg);
ros::Publisher pub_pitch ("pitch", &pitch_msg);
bool imu_connection;

// Таймеры
unsigned long timer = 0;
float timeStep      = 0.01;

uint32_t LastSec    = 0;
float deltaT        = 0.3f;
volatile float beta = betaDef;                             // 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame

float angles[3];



sensor_msgs::Range rangeR_msg;
sensor_msgs::Range rangeL_msg;
sensor_msgs::Range rangeF_msg;

std_msgs::Float64 EnR_msg;
std_msgs::Float64 EnL_msg;
std_msgs::Float64 EnB_msg;

ros::Publisher pub_EnR ("range_left_enc", &EnR_msg);
ros::Publisher pub_EnL ("range_right_enc", &EnL_msg);
ros::Publisher pub_EnB ("range_front_enc", &EnB_msg);

ros::Publisher pub_range_left ("range_left_ping", &rangeL_msg);
ros::Publisher pub_range_right ("range_right_ping", &rangeR_msg);
ros::Publisher pub_range_front ("range_front_ping", &rangeF_msg);

void messageCdRight (const std_msgs::Float64 &msg){
  if (msg.data<0) { 
  analogWrite(MotorSpeed [0], abs(msg.data));
  digitalWrite( MotorDirectionRight [0], HIGH);
  digitalWrite(MotorDirectionLeft [0], LOW);
  }
   if (msg.data>0) { 
  analogWrite(MotorSpeed [0], msg.data);
  digitalWrite(MotorDirectionLeft [0], HIGH);
  digitalWrite(MotorDirectionRight [0], LOW);
  }
    if (msg.data==0) { 
  analogWrite(MotorSpeed [0], msg.data);
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
  analogWrite(MotorSpeed [2], msg.data);
  digitalWrite(MotorDirectionLeft [2], HIGH);
  digitalWrite(MotorDirectionRight [2], LOW);
  }
    if (msg.data==0) { 
  analogWrite(MotorSpeed [2], msg.data);
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
  analogWrite(MotorSpeed [1], msg.data);
  digitalWrite(MotorDirectionLeft [1], HIGH);
  digitalWrite(MotorDirectionRight [1], LOW);
  }
    if (msg.data==0) { 
  analogWrite(MotorSpeed [1], msg.data);
  digitalWrite(MotorDirectionLeft [1], LOW);
  digitalWrite(MotorDirectionRight [1], LOW);
  }
};

 ros::Subscriber<std_msgs::Float64> subR("v_right", &messageCdRight);
 ros::Subscriber<std_msgs::Float64> subL("v_left", &messageCdLeft);
 ros::Subscriber<std_msgs::Float64> subB("v_back", &messageCdBack);

void setup() {
  nh.initNode();
nh.advertise(pub_range_left);
nh.advertise(pub_range_right);
nh.advertise(pub_range_front);
nh.advertise(pub_EnR);
nh.advertise(pub_EnL);
nh.advertise(pub_EnB);
unsigned long echo;

for(int i=0; i < 3; i++) {
pinMode(PINGTrig[i], OUTPUT);
pinMode(PINGEcho[i], INPUT);
pinMode(ENCA[i], INPUT);
pinMode(ENCB[i], INPUT);
  }

Scheduler.startLoop(setupMotor,loopMotor);
Scheduler.startLoop(loopIMU);
Scheduler.startLoop(loopNCoder);

}
void readEncoder(){
  int b = digitalRead(ENCB[1]);
  int r = digitalRead(ENCB[2]);
  int l = digitalRead(ENCB[0]);
  if(b > 0){
    posB++;
     pub_range_front.publish (&EnB_msg);
  }
  else{
     if (b=0) {
       posB=posB;
       pub_range_front.publish (&EnB_msg);
  }
  else {posB--;}
  pub_range_front.publish (&EnB_msg);
  }
    if(l > 0){
    posL++;
    pub_range_front.publish (&EnL_msg);
  }
  else{
       if (l=0) {
       posL=posL;
       pub_range_front.publish (&EnL_msg);
  }
  else {posL--;}
  }
    if(r > 0){
    posL++;
    pub_range_front.publish (&EnL_msg);
  }
  else{
       if (r=0) {
       posR=posR;
       pub_range_front.publish (&EnR_msg);
  }
  else {posR--;
  pub_range_front.publish (&EnR_msg);}
  }
}
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
    delay(50);

  }
  nh.spinOnce();
  delay(50); 

}
//Motor loop
void setupMotor() {
for(int i=0; i < 3; i++) {
pinMode(MotorDirectionRight[i], OUTPUT);
pinMode(MotorDirectionLeft[i], OUTPUT);
pinMode(MotorSpeed[i], OUTPUT);
digitalWrite(MotorDirectionLeft[i], LOW);
digitalWrite(MotorDirectionRight[i], LOW);
}
nh.initNode();
nh.subscribe(subL);
nh.subscribe(subR);
nh.subscribe(subB);
}

void loopMotor() {
 nh.spinOnce();
  delay(1);
}

void setupIMU() {
 nh.initNode();
  nh.advertise(pub_yaw);
  nh.advertise(pub_roll);
  nh.advertise(pub_pitch);
}
void loopIMU() {
 uint8_t i;
    // put your main code here, to run repeatedly:
    if (imu_connection) {
        IMU_GetYawPitchRoll(angles, millis());
        pitch_msg.linear.x =angles[2];
        pitch_msg.linear.y =angles[0];
        pitch_msg.linear.z =angles[1];
        pub_pitch.publish (&pitch_msg);
        pub_roll.publish (&roll_msg);
        pub_yaw.publish (&yaw_msg); 
    }
    nh.spinOnce();
  delay(10); 
}


void loopNCoder() {
   readEncoder();
   nh.spinOnce();
    delay(10); // Пауза 3 секунды

  
}

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0        = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1        = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2        = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3        = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    q0 += qDot1 * deltaT;
    q1 += qDot2 * deltaT;
    q2 += qDot3 * deltaT;
    q3 += qDot4 * deltaT;
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y     = x;
    long i      = *(long *)&y;
    i           = 0x5f3759df - (i >> 1);
    y           = *(float *)&i;
    y           = y * (1.5f - (halfx * y * y));
    return y;
    /*unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
    float tmp = *(float*)&i;
    return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
      */
}

void IMU_GetQuater(void) {
    float MotionVal[9];

    imu.read();
    MotionVal[0] = imu.gx;
    MotionVal[1] = imu.gy;
    MotionVal[2] = imu.gz;

    MotionVal[3] = imu.ax_raw;
    MotionVal[4] = imu.ay_raw;
    MotionVal[5] = imu.az_raw;

    MotionVal[6] = imu.mx_raw;
    MotionVal[7] = imu.my_raw;
    MotionVal[8] = imu.mz_raw;

    //MadgwickAHRSupdate((float)MotionVal[0] * 0.0175, (float)MotionVal[1] * 0.0175, (float)MotionVal[2] * 0.0175, (float)MotionVal[3], (float)MotionVal[4],
                      // (float)MotionVal[5], (float)MotionVal[6], (float)MotionVal[7], (float)MotionVal[8]);
    MadgwickAHRSupdateIMU((float)MotionVal[0] * 0.0175, (float)MotionVal[1] * 0.0175, (float)MotionVal[2] * 0.0175,
    (float)MotionVal[3], (float)MotionVal[4], (float)MotionVal[5]);
}

void IMU_GetYawPitchRoll(float *Angles, uint32_t S) {
    deltaT  = ((float)(S - LastSec)) / 1000.0;
    LastSec = S;

    IMU_GetQuater();
    Angles[1] = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;                                 // pitch
    Angles[2] = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; // roll
    Angles[0] = atan2(-2 * q1 * q2 - 2 * q0 * q3, 2 * q2 * q2 + 2 * q3 * q3 - 1) * 57.3;
}