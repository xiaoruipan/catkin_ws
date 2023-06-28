#include "ros/ros.h"
#include "Servo_ros.h"
#include "VMXPi.h"
#include <std_msgs/Float32.h>
#include <iostream>

float ShoulderServoAngle = 0.0f;
float ElbowServoAngle = 0.0f;
int channel1 = 12;
int channel2 = 13;

void ShoulderOffsetCallback(const std_msgs::Float32::ConstPtr& msg)
{
    ShoulderServoAngle = msg->data;
    ROS_INFO("Received slider value for Shoulder: %.2f", ShoulderServoAngle);
}

void ElbowOffsetCallback(const std_msgs::Float32::ConstPtr& msg)
{
    ElbowServoAngle = msg->data;
    ROS_INFO("Received slider value for Elbow: %.2f", ElbowServoAngle);
}

int main(int argc, char **argv)
{
    system("/usr/local/frc/bin/frcKillRobot.sh");
    ros::init(argc, argv, "Arm_Node");
    ros::NodeHandle nh;

    VMXPi vmx(true, (uint8_t)50);
    ServoROS ShoulderServo(&nh, &vmx, channel1);
    ServoROS ElbowServo(&nh, &vmx, channel2);

    ros::Subscriber ShoulderOffset = nh.subscribe<std_msgs::Float32>("slider_value", 10, ShoulderOffsetCallback);
    ros::Subscriber ElbowOffset = nh.subscribe<std_msgs::Float32>("slider_value_1", 10, ElbowOffsetCallback);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        vmxpi_ros::Float::Request ShoulderReq, ElbowReq;
        vmxpi_ros::Float::Response res;
        ShoulderReq.data = ShoulderServoAngle;
        ElbowReq.data = ElbowServoAngle;

        ShoulderServo.SetAngle(ShoulderReq, res);
        ElbowServo.SetAngle(ElbowReq, res);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


// #include "ros/ros.h"
// #include "Servo_ros.h"
// #include "VMXPi.h"

// #include <std_msgs/Float32.h>
// #include "std_msgs/Int32.h"
// #include "std_msgs/Float32MultiArray.h"
// #include "ros/service_client.h"
// #include "math.h"
// #include <iostream>
// #include <termios.h>
// #include <signal.h>


// double servo_angle;
// float ShoulderServoAngle, ElbowServoAngle;
// int channel1 = 12;
// int channel2 = 13;

// void servo_angle_callback(const std_msgs::Float32::ConstPtr& msg)
// {
//     servo_angle = msg->data;
// }
// void ShoulderOffsetCallback(const std_msgs::Float32::ConstPtr& msg)
// {
//     ShoulderServoAngle = msg->data;
//     ROS_INFO("Received slider value: %.2f", ShoulderServoAngle);
// }
// void ElbowOffsetCallback(const std_msgs::Float32::ConstPtr& msg)
// {
//     ElbowServoAngle = msg->data;
// }
// int main(int argc, char **argv)
// {
//     system("/usr/local/frc/bin/frcKillRobot.sh");
//     ros::init(argc, argv, "Arm_Node");
//     ros::NodeHandle nh;
//     VMXPi vmx(true, (uint8_t)50);
//     ros::Subscriber Shoulder_Servo_Sub, Elbow_Servo_Sub;
//     ros::Subscriber ShoulderOffset = nh.subscribe<std_msgs::Float32>("slider_value", 10, ShoulderOffsetCallback);
//     ros::Subscriber ElbowOffset = nh.subscribe<std_msgs::Float32>("slider_value_1", 10, ElbowOffsetCallback);
//     ServoROS ShoulderServo(&nh, &vmx, channel1);
//     ServoROS ElbowServo(&nh, &vmx, channel2);
//     ShoulderServo.GetAngle();
//     ShoulderServo.GetMinAngle();
//     ShoulderServo.GetMaxAngle();
//     ElbowServo.GetAngle();
//     ElbowServo.GetMinAngle();
//     ElbowServo.GetMaxAngle();
//     vmxpi_ros::Float::Request ShoulderReq, ElbowReq;
//     vmxpi_ros::Float::Response res;
//     ShoulderReq.data = ShoulderServoAngle;
//     ElbowReq.data = ElbowServoAngle;
//     ShoulderServo.SetAngle(ShoulderReq, res);
//     ElbowServo.SetAngle(ElbowReq, res);
//     ROS_INFO("%f", ShoulderServoAngle);
//     Shoulder_Servo_Sub = nh.subscribe("channel/channel_index/servo/angle", 1, servo_angle_callback);
//     Elbow_Servo_Sub = nh.subscribe("channel/channel_index/servo/angle", 1, servo_angle_callback);
//     ros::spin();
//     return 0;
// }
/* double Pi = 3.14159265;
float Shoulder_Servo_Angle, Elbow_Servo_Angle;
int Shoulderchannel = 12;
int Elbowchannel = 13;
float Shoulder_Ratio = 4;
float Elbow_Ratio = 2;
double x, y;

void Shoulder_Servo_Angle_callback(const std_msgs::Float32::ConstPtr& msg)
{
    Shoulder_Servo_Angle = msg->data;
}
void Elbow_Servo_Angle_callback(const std_msgs::Float32::ConstPtr& msg)
{
    Elbow_Servo_Angle = msg->data;
}
void CoordinatesInput_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    x = msg->data[0];
    y = msg->data[1];
}
float toDegrees(float x)
{
    float Conversion = x * 57.2958;
    return Conversion;
}
void InvKin(double x,double y)
{
    double a, b, c, alpha, beta, A, B; 
    a = 0.24; // Y default lengths of the arm
    c = 0.33; // X
    b = sqrt(x * x + y * y);
    alpha = acos((b * b + c * c - a * a) / ( 2 * b * c));
    beta = acos((a * a + c * c - b * b) / (2 * a * c));
    B = Pi - beta;
    A = alpha + atan2(y,x);
    Shoulder_Servo_Angle = toDegrees(A) * Shoulder_Ratio;
    Elbow_Servo_Angle = toDegrees(B) * Elbow_Ratio;
}

int main(int argc, char **argv)
{
    system("/usr/local/frc/bin/frcKillRobot.sh");
    ros::init(argc, argv, "Arm_Node");
    ros::NodeHandle nh;
    VMXPi vmx(true, (uint8_t)50);
    ros::ServiceClient ShouldersetAngle, ElbowsetAngle;
    ros::Subscriber Shoulder_Servo_Angle_sub;
    ros::Subscriber Elbow_Servo_Angle_sub;
    ros::Subscriber CoordinatesInput;
    CoordinatesInput = nh.subscribe<std_msgs::Float32MultiArray>("ArmCoordinatesTopic", 10, CoordinatesInput_callback);
    Shoulder_Servo_Angle_sub = nh.subscribe("channel/"+ std::to_string(Shoulderchannel) + "/servo/angle", 1, Shoulder_Servo_Angle_callback);
    Elbow_Servo_Angle_sub = nh.subscribe("channel/"+ std::to_string(Elbowchannel) + "/servo/angle", 1, Elbow_Servo_Angle_callback);
    InvKin(x,y);
    ServoROS Shoulderservo(&nh, &vmx, Shoulderchannel);
    ServoROS Elbowservo(&nh, &vmx, Elbowchannel);
    Shoulderservo.GetAngle();
    Shoulderservo.GetMinAngle();
    Shoulderservo.GetMaxAngle();
    Elbowservo.GetAngle();
    Elbowservo.GetMinAngle();
    Elbowservo.GetMaxAngle();
    ShouldersetAngle = nh.serviceClient<vmxpi_ros::Float>("channel/"+ std::to_string(Shoulderchannel) + "/servo/set_angle");
    ElbowsetAngle = nh.serviceClient<vmxpi_ros::Float>("channel/"+ std::to_string(Elbowchannel) + "/servo/set_angle");
    vmxpi_ros::Float::Request req1, req2;
    vmxpi_ros::Float::Response res;
    req1.data = Shoulder_Servo_Angle;
    req2.data = Elbow_Servo_Angle;
    Shoulderservo.SetAngle(req1, res);
    Elbowservo.SetAngle(req2, res);
    ros::spin();
} Not working*/