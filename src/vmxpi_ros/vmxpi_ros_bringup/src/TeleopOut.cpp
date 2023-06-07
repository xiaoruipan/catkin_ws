#include <ros/ros.h>
#include <signal.h>
#include <string>
#include <std_msgs/String.h>
#include <iostream>

std::string key, JoystickKey;

void KeyboardInputCallback(const std_msgs::String::ConstPtr& msg)
{
    key = msg->data;
}
void JoystickInputCallback(const std_msgs::String::ConstPtr& msg)
{
    JoystickKey = msg->data;
}

int main(int argc, char **argv)
{
    // puts("Reading from keyboard");
    // ros::init(argc, argv, "Keyboard_Input_Subscriber");
    puts("Reading from Joystick");
    ros::init(argc, argv, "Joystick_Input_Subscriber");
    ros::NodeHandle nh;
    // ros::Subscriber KeyboardInputSub;
    ros::Subscriber JoyInputSub;
    JoyInputSub = nh.subscribe("JoystickAxisTopic", 1, JoystickInputCallback);
    ROS_INFO("%s", JoystickKey.c_str());
    ros::spin();
    return 0;
}
