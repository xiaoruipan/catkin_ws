#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ArmTestingNode");
    ros::NodeHandle nh;
    ros::Publisher ArmTestPub = nh.advertise<std_msgs::Float32MultiArray>("ArmCoordinatesTopic", 10);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {  
        std_msgs::Float32MultiArray numbers;  
        numbers.data.push_back(0.10);
        numbers.data.push_back(0.20);
        loop_rate.sleep();
    }

    return 0;
}