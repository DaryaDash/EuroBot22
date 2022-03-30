#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <navx/AHRS.h>
#include <chrono>
#include <thread>
#include <iomanip>
#include <signal.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"


		volatile sig_atomic_t sflag = 0;

		void handle_sig(int sig)
		{
		    sflag = 1;
		}


int main(int argc, char **argv) {


    ros::init(argc, argv, "navx_node");
     
     
     ros::NodeHandle n;
     
     
     ros::Publisher navx_pub = n.advertise<std_msgs::Float64>("yaw",10);
     
     
    ros::Rate loop_rate(10);
    

    signal(SIGINT, handle_sig);

    AHRS com = AHRS("/dev/ttyACM0");



    while( ros::ok()){
    
    
        std_msgs::Float64 msg;
 
        
    	 msg.data = com.GetYaw();
    
         navx_pub.publish (msg);
       
        
        if(sflag){
            sflag = 0;
            com.Close();
            break; 
        }
    ros::spinOnce;
    loop_rate.sleep();
    }

    return 0;
}
