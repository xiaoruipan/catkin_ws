#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

ros::Publisher joystickOutputPub;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // Access the joystick axes and buttons
    float x_axis = joy->axes[0];  // Adjust the index according to your joystick mapping
    float y_axis = joy->axes[1];  // Adjust the index according to your joystick mapping
    //This is the test for github pushing

    //This is the test to show that it's working
    // Define the joystick control thresholds
    float threshold = 0.5;  // Adjust the threshold value according to your joystick sensitivity

    // Check joystick axes for specific actions
    std::string message;
    if (x_axis < -threshold)
    {
        message = "LEFT";
    }
    else if (x_axis > threshold)
    {
        message = "RIGHT";
    }
    else if (y_axis < -threshold)
    {
        message = "UP";
    }
    else if (y_axis > threshold)
    {
        message = "DOWN";
    }
    else
    {
        message = "IDLE";
    }

    std_msgs::String outputMsg;
    outputMsg.data = message;

    joystickOutputPub.publish(outputMsg);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Joystick_Input_Node");
    ros::NodeHandle nh;
    ros::Subscriber joySub = nh.subscribe<sensor_msgs::Joy>("joy", 1, joyCallback);
    joystickOutputPub = nh.advertise<std_msgs::String>("plswork", 1);

    ros::spin();
    return 0;
}
