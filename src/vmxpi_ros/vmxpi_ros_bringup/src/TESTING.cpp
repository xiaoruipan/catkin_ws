#include "ros/ros.h"
#include "ros/service_client.h"
#include "Servo_ros.h"
#include "VMXPi.h"
#include "std_msgs/Float32.h"
#include "Servo_ros.h"
#include <vmxpi_ros/Float.h>


double servo_angle;
int channel1 = 12;
int channel2 = 13;

// Returns the angle value set by the Servo motor
void servo_angle_callback(const std_msgs::Float32::ConstPtr& msg)
{
   servo_angle = msg->data;
}

int main(int argc, char **argv)
{
   system("/usr/local/frc/bin/frcKillRobot.sh"); //Terminal call to kill the robot manager used for WPILib before running the executable.
   ros::init(argc, argv, "servo_node");

   /**
    * Constructor
    * Servo's ros threads (publishers and services) will run asynchronously in the background
    */

   ros::NodeHandle nh; //internal reference to the ROS node that the program will use to interact with the ROS system
   VMXPi vmx(true, (uint8_t)50); //realtime bool and the update rate to use for the VMXPi AHRS/IMU interface, default is 50hz within a valid range of 4-200Hz

   ros::Subscriber servo_angle_sub1, servo_angle_sub2;

   ServoROS servo1(&nh, &vmx, channel1);
   ServoROS servo2(&nh, &vmx, channel2);

   // Use these to directly access data
   servo1.GetAngle(); //returns a double;
   servo1.GetMinAngle(); //returns a double
   servo1.GetMaxAngle(); //returns a double
   servo2.GetAngle(); //returns a double;
   servo2.GetMinAngle(); //returns a double
   servo2.GetMaxAngle(); //returns a double
   // Setting the servo angle
   float angle1 = -132; //Range -150° - 150°
   float angle2 = 129;
   vmxpi_ros::Float::Request req1, req2;
   vmxpi_ros::Float::Response res;
   req1.data = angle1;
   req2.data = angle2;
   servo1.SetAngle(req1, res);
   servo2.SetAngle(req2, res);


   // Subscribing to Servo angle topic to access the angle data
   servo_angle_sub1 = nh.subscribe("channel/channel_index/servo/angle", 1, servo_angle_callback); //channel_index is the input channel set in the constructor
servo_angle_sub2 = nh.subscribe("channel/channel_index/servo/angle", 1, servo_angle_callback);
   ros::spin(); //ros::spin() will enter a loop, pumping callbacks to obtain the latest sensor data

   return 0;
}
