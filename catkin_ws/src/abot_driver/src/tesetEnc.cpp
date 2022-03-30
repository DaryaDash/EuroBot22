#include <ros/ros.h>
#include <wiringPi.h>

constexpr uint8_t ENCODER_1_PIN_A = 15; // Wiring pi 0 = BCM 17
constexpr uint8_t ENCODER_1_PIN_B = 16; // Wiring pi 2 = BCM 27
constexpr uint8_t ENCODER_2_PIN_A = 11; // Wiring pi 5 = BCM 24
constexpr uint8_t ENCODER_2_PIN_B = 13; // Wiring pi 6 = BCM 25
constexpr uint8_t ENCODER_3_PIN_A = 18; // Wiring pi 5 = BCM 24
constexpr uint8_t ENCODER_3_PIN_B = 22; // Wiring pi 6 = BCM 25
left_wheel_velocity_pub = _node.advertise<std_msgs::Float64>("l", 1);
r_wheel_velocity_pub = _node.advertise<std_msgs::Float64>("r", 1);

int main (){
    if (true){
uint8_t val_A = digitalRead(ENCODER_1_PIN_A);
uint8_t val_B = digitalRead(ENCODER_1_PIN_B );
 ctd::cout <<"a" <<val_A<<std::endl;
 ctd::cout <<"a" <<val_A<<std::endl;
left_wheel_velocity_pub.publish(val_A);
r_wheel_velocity_pub.publish(val_B);
    }
}