#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <AHRS.h>
#include <math.h>

float derive(float prevVal, float currVal, float prevTime, float currTime)
{

	return (currVal - prevVal) / (currTime - prevTime);

}

int main(int argc, char** argv){
	
	ros::init(argc, argv, "navx_node");
	ros::NodeHandle nh;
	//ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("Imu", 10);
	ros::Subscriber sub = nh.subscribe("zeroYaw",10, ZeroYawcb);
	ros::Publisher pub = nh.advertise<std_msgs::Float64>("yaw", 10);
	ros::Rate period(10);

	//sensor_msgs::Imu imu;		
	std_msgs::Float64 CurrY;

	AHRS navx = AHRS("/dev/ttyACM0");
	//initialize valus to calculate angular velocity for the IMU message
	//float prevTime = navx.GetLastSensorTimestamp() / 1000.0;
	//float prevPitch = navx.GetPitch() * degToRad; //about x-axis (empirically y-axis)
	//float prevRoll = navx.GetRoll() * degToRad; //about y-axis (empirically x-axis)
	float prevYaw = navx.GetYaw() * degToRad; //about z-axis

	//fill and send message
	while(ros::ok()){

		//header
		//imu.header.frame_id = "imu";

		//orientation
		//imu.orientation.x = navx.GetQuaternionX();
		//imu.orientation.y = navx.GetQuaternionY();
		//imu.orientation.z = navx.GetQuaternionZ();
		//imu.orientation.w = navx.GetQuaternionW();

		//linear acceleration
		//imu.linear_acceleration.x = navx.GetWorldLinearAccelX();
		//imu.linear_acceleration.y = navx.GetWorldLinearAccelY();
		//imu.linear_acceleration.z = navx.GetWorldLinearAccelZ();
	
		//angular velocity
		//float currTime = navx.GetLastSensorTimestamp() / 1000.0;
		//float currPitch = navx.GetPitch() * degToRad;
		//float currRoll = navx.GetRoll() * degToRad;
		float currYaw = navx.GetYaw(); //* degToRad;
		CurrY.data=currYaw;
		pub.publish(CurrY);
		
		//these are flipped based on the diagram on the sensor
		//imu.angular_velocity.y = derive(prevPitch, currPitch, prevTime, currTime);
		//imu.angular_velocity.x = derive(prevRoll, currRoll, prevTime, currTime);
		//imu.angular_velocity.z = derive(prevYaw, currYaw, prevTime, currTime);	

		//prevTime = currTime;
		//prevPitch = currPitch;
		//prevRoll = currRoll;
		prevYaw = currYaw;

		//pub.publish(imu);
		ros::spinOnce();
		period.sleep();
	}
	return 0;
}
