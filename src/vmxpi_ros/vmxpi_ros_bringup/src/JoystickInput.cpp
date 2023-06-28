#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

std_msgs::String axis_message;
std_msgs::Float32 axis_float;

void JoystickInput(const sensor_msgs::Joy::ConstPtr& joy)
{
    // Ensure the joystick message has enough axes
    if (joy->axes.size() >= 2)
    {
        float x_axis = joy->axes[0];
        float y_axis = joy->axes[1];
        float rotation_axis = joy->axes[3];

        if (x_axis > 0.1)
        {
            axis_message.data = "X Axis Negative";
            axis_float.data = x_axis;
        }
        else if (x_axis < -0.1)
        {
            axis_message.data = "X Axis Positive";
            axis_float.data = x_axis;
        }
        else if (y_axis > 0.1)
        {
            axis_message.data = "Y Axis Positive";
            axis_float.data = y_axis;
        }
        else if (y_axis < -0.1)
        {
            axis_message.data = "Y Axis Negative";
            axis_float.data = y_axis;
        }
        else if (rotation_axis > 0.1)
        {
            axis_message.data = "Rotation Clockwise";
            axis_float.data = rotation_axis;
        }
        else if (rotation_axis < -0.1)
        {
            axis_message.data = "Rotation Counter Clockwise";
            axis_float.data = rotation_axis;
        }
        else 
        {
            axis_message.data = "No Movement is detected";
        }
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Joystick_Input_Node");
    ros::NodeHandle nh;
    ros::Publisher Joystick_Publisher = nh.advertise<std_msgs::String>("JoystickAxisTopic", 1);
    ros::Publisher axis_value_publisher = nh.advertise<std_msgs::Float32>("AxisValue",1 );
    ros::Subscriber joystickSub = nh.subscribe<sensor_msgs::Joy>("joy", 10, JoystickInput);
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();  // Call any pending callbacks

        Joystick_Publisher.publish(axis_message);
        axis_value_publisher.publish(axis_float);
        loop_rate.sleep();
    }

    return 0;
}
