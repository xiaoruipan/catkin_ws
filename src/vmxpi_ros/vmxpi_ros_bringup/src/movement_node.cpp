#include "TitanDriver_ros_wrapper.h"
#include "navX_ros_wrapper.h"
#include "Cobra_ros.h"
#include "Sharp_ros.h"
#include "Servo_ros.h"
#include "Ultrasonic_ros.h"
#include "IOwd_ros.h"
#include "DI_ros.h"
#include "DO_ros.h"
#include <unistd.h>

#include <dynamic_reconfigure/server.h>
#include <vmxpi_ros_bringup/MotorSpeedConfig.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <vmxpi_ros/Float.h>
#include <cmath>

//variables for subscriber variables
string keyboard_typed;
std::vector<float> joystick_axes; //axes measurements from a joystick
string axis_message; //Gets readings from plswork topic
float AxisValue;

static double leftSpeed, rightSpeed, backSpeed, right_encoder, back_encoder, left_encoder;
static double left_count, right_count, back_count;
static double ocv_leftSpeed, ocv_rightSpeed, ocv_backSpeed;
static double displacex, displacey, magnitude_t, angle, angle_t;
static double PI = 3.14159265;

// Callbacks for OpenCV object tracking
void lmotorCallback(const std_msgs::Float32::ConstPtr& msg)
{
    ocv_leftSpeed = msg->data;
}
void rmotorCallback(const std_msgs::Float32::ConstPtr& msg)
{
    ocv_rightSpeed = msg->data;
}
void bmotorCallback(const std_msgs::Float32::ConstPtr& msg)
{
    ocv_backSpeed = msg->data;
}

// Callbacks for Encoder Distance values
void motor0Callback(const std_msgs::Float32::ConstPtr& msg)
{
    right_encoder = msg->data;
}
void motor1Callback(const std_msgs::Float32::ConstPtr& msg)
{
    back_encoder = msg->data;
}
void motor3Callback(const std_msgs::Float32::ConstPtr& msg)
{
    left_encoder = msg->data;
}
void angleCallback(const std_msgs::Float32::ConstPtr& msg)
{
    angle = abs(msg->data);
}
void yawCallback(const std_msgs::Float32::ConstPtr& msg)
{
    angle_t = msg->data;
}

// Callbacks for Encoder count values
void enc0Callback(const std_msgs::Int32::ConstPtr& msg)
{
    right_count = msg->data;
}
void enc1Callback(const std_msgs::Int32::ConstPtr& msg)
{
    back_count = msg->data;
}
void enc3Callback(const std_msgs::Int32::ConstPtr& msg)
{
    left_count = msg->data;
}

class DynamicReconfig {
    bool flag = true;
    double integrator, prevError, differentiator, prevMeasurement, output;
    double prev_angle = 0.0;

public:
    ros::ServiceClient set_m_speed, enable_client, disable_client;
    ros::ServiceClient resetAngle, res_encoder_client, stop_motors_client;

    ros::Subscriber lmotor_sub, rmotor_sub, bmotor_sub;
    ros::Subscriber motor0_dist, motor1_dist, motor3_dist, angle_sub, yawAngle_sub;
    ros::Subscriber enc0_sub, enc1_sub, enc3_sub;
    ros::Publisher displacex_pub, displacey_pub, magnitude_pub, lmotor_PID_pub, rmotor_PID_pub, bmotor_PID_pub, error_pub;

    double tau = 0.02, T = 0.02;
    //double tolerance = 1.0; // Tolerance of Encoder distances(mm) and angle(deg)
    double kP, kI, kD, error;
    double limMin = -1.0, limMax = 1.0, limMinInt = -0.5, limMaxInt = 0.5;
    bool atSetpoint;

    DynamicReconfig(ros::NodeHandle *nh) {
        set_m_speed = nh->serviceClient<vmxpi_ros::MotorSpeed>("titan/set_motor_speed");
        lmotor_sub = nh->subscribe("auto_motor_left", 1, lmotorCallback);
        rmotor_sub = nh->subscribe("auto_motor_right", 1, rmotorCallback);
        bmotor_sub = nh->subscribe("auto_motor_back", 1, bmotorCallback);

        motor0_dist = nh->subscribe("titan/encoder0/distance", 1, motor0Callback);
        motor1_dist = nh->subscribe("titan/encoder1/distance", 1, motor1Callback);
        motor3_dist = nh->subscribe("titan/encoder3/distance", 1, motor3Callback);
        angle_sub = nh->subscribe("navx/angle", 1, angleCallback);
        yawAngle_sub = nh->subscribe("navx/yaw", 1, yawCallback);

        enc0_sub = nh->subscribe("titan/encoder0/count", 1, enc0Callback);
        enc1_sub = nh->subscribe("titan/encoder1/count", 1, enc1Callback);
        enc3_sub = nh->subscribe("titan/encoder3/count", 1, enc3Callback);

        displacex_pub = nh->advertise<std_msgs::Float32>("displace_x", 1);
        displacey_pub = nh->advertise<std_msgs::Float32>("displace_y", 1);
        magnitude_pub = nh->advertise<std_msgs::Float32>("magnitude", 1);

        lmotor_PID_pub = nh->advertise<std_msgs::Float32>("lmotor_PID", 1);
        rmotor_PID_pub = nh->advertise<std_msgs::Float32>("rmotor_PID", 1);
        bmotor_PID_pub = nh->advertise<std_msgs::Float32>("bmotor_PID", 1);
        error_pub = nh->advertise<std_msgs::Float32>("error", 1);

        enable_client = nh->serviceClient<std_srvs::Trigger>("titan/enable");
        disable_client = nh->serviceClient<std_srvs::Trigger>("titan/disable");

        resetAngle = nh->serviceClient<std_srvs::Empty>("reset_navx");
        res_encoder_client = nh->serviceClient<std_srvs::Trigger>("titan/reset_encoder");
        stop_motors_client = nh->serviceClient<std_srvs::Trigger>("titan/stop_motors");
    }

    void PIDReset()
    {
        kP = 0.0;
        kI = 0.0;
        kD = 0.0;
        error = 0.0;
        integrator = 0.0;
        prevError = 0.0;
        differentiator = 0.0;
        prevMeasurement = 0.0;
        atSetpoint = false;
        output = 0.0;
    }

    double calculate(double setPoint, double measurement, double tolerance)
    {
        /*
         * Error
         */
        error = setPoint - measurement;

        /*
         * Setpoint check
         */
        if (abs(error) <= tolerance)
        {
            atSetpoint = true;
            return 0.0;
        }
        else
        {
            atSetpoint = false;
        }

        /*
         * Proportional
         */
        double proportional = kP * error;

        /*
         * Integral
         */
        integrator = integrator + 0.5 * kI * T * (error + prevError);

        /*
         * Anti Wind up
         */
        if (integrator > limMaxInt)
        {
            integrator = limMaxInt;
        }
        else if (integrator < limMinInt)
        {
            integrator = limMinInt;
        }

        /*
         * Band limit derivative
         */
        differentiator = -(2.0 * kD * (measurement - prevMeasurement) + (2.0 * tau - T) * differentiator) / (2.0 * tau + T);

        /*
         * Compute
         */
        output = proportional + integrator + differentiator;

        /*
         * Clamp
         */
        if (output > limMax)
        {
            output = limMax;
        }
        else if (output < limMin)
        {
            output = limMin;
        }

        /*
         * Store variables
         */
        prevError = error;
        prevMeasurement = measurement;

        /*
         * Return final value
         */
        return output;
    }

    void encoder2dist()
    {
        //Displace forward and back
        displacey = ((left_encoder * (sqrt(3) / 2)) + ((right_encoder * (sqrt(3) / 2)) * -1)) * -0.68;
        //displacey = (left_encoder + right_encoder) / -1.5;
        //displacey = left_encoder * (sqrt(3) / 2);
        displacey /= 1000;

        //Dispace left and right
        //displacex = back_encoder;
        displacex = (back_encoder + (left_encoder * -0.5) + (right_encoder * -0.5)) * 0.70;
        displacex /= 1000;

        magnitude_t = sqrt(pow(abs(displacex), 2.0) + pow(abs(displacey), 2.0));
    }

    void PubDisplacements()
    {
        std_msgs::Float32 msg;
        msg.data = displacex;
        displacex_pub.publish(msg);
        msg.data = displacey;
        displacey_pub.publish(msg);
        msg.data = magnitude_t;
        magnitude_pub.publish(msg);

        msg.data = leftSpeed;
        lmotor_PID_pub.publish(msg);

        msg.data = rightSpeed;
        rmotor_PID_pub.publish(msg);

        msg.data = backSpeed;
        bmotor_PID_pub.publish(msg);

        msg.data = error;
        error_pub.publish(msg);
    }

    void holonomicDrive(double x, double y, double z)
    {
        rightSpeed = -(x / 2) + ((y * (sqrt(3) / 2))) + z;
        leftSpeed = -(x / 2) - (y * sqrt(3) / 2) + z;
        backSpeed = x + z;

        double max = abs(rightSpeed);
        if (abs(leftSpeed) > max)
        {
            max = abs(leftSpeed);
        }
        if (abs(backSpeed) > max)
        {
            max = abs(backSpeed);
        }
        if (max > 1)
        {
            rightSpeed /= max;
            leftSpeed /= max;
            backSpeed /= max;
        }
    }

    void reset()
    {
        std_srvs::Trigger msg1;
        stop_motors_client.call(msg1); // Stops motors
        res_encoder_client.call(msg1); // Resets displacement encoders
        std_srvs::Empty msg2;
        resetAngle.call(msg2); // Resets yaw variable
    }

    void setPID(double Kp, double Ki, double Kd)
    {
        kP = Kp;
        kI = Ki;
        kD = Kd;
    }
    
    void stop_motors()
    {
        vmxpi_ros::MotorSpeed msg1;

        msg1.request.speed = 0.0;
        msg1.request.motor = 0;
        set_m_speed.call(msg1);

        msg1.request.speed = 0.0;
        msg1.request.motor = 1;
        set_m_speed.call(msg1);

        msg1.request.speed = 0.0;
        msg1.request.motor = 3;
        set_m_speed.call(msg1);
    }

    /**
     *robot.setMovements(1.0,-90); goes forward
     *robot.setMovements(1.0,90); goes backwards
     *robot.setMovements(1.0,0); goes left
     *robot.setMovements(1.0,180); goes right
    */
    void setMovements(double magnitude, double Angle)
    {
        if (prev_angle > Angle)
        {
            Angle = Angle - prev_angle + 360.0;
        }
        else
            Angle = Angle - prev_angle;

        double target_x_displace = magnitude * cos(abs(Angle) * PI / 180.0);
        double target_y_displace = magnitude * sin(abs(Angle) * PI / 180.0);

        ros::Rate loop_rate(50);
        while (ros::ok())
        {
            holonomicDrive(0.0, 0.0, 0.0); //Check step to zero the motors

            if (flag == true)
            {
                reset();
                displacex = 0;
                displacey = 0;
                angle = 0;
                PIDReset();
                flag = false;
            }
            
            setPID(0.95, 0.0, 0.001);
            double x_drive = calculate(target_x_displace, displacex, 0.001);
            
            //setPID(0.9, 0.0, 0.005);
            double y_drive = calculate(target_y_displace, displacey, 0.001);
            
            setPID(0.050, 0.0, 0.0);
            double angle_drive = calculate(0.0, angle_t, 0.0);

            holonomicDrive(x_drive, y_drive, angle_drive);
            
            publish_motors();
            encoder2dist();
            PubDisplacements();

            if (magnitude_t >= abs(magnitude))
            {
                displacex = 0;
                displacey = 0;
                angle = 0;
                PIDReset();
                reset();
                flag = true;
                break;
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void setAngle(double target_angle)
    {
        ros::Rate loop_rate(100);
        while (ros::ok())
        {
            holonomicDrive(0.0, 0.0, 0.0); //Check step to zero the motors

            if (flag == true)
            {
                reset();
                flag = false;
            }

            setPID(0.1, 0.0, 0.0);
            double angle_drive = calculate(target_angle, angle_t, 1.0);
            if (angle < target_angle)
            {
                holonomicDrive(0.0, 0.0, -0.3); // Counter-clockwise is positive angle
            }
            else if (angle > target_angle)
            {
                holonomicDrive(0.0, 0.0, 0.3); // Clockwise is negative angle
            }

            publish_motors();
            encoder2dist();
            PubDisplacements();
            ROS_INFO("AFTER PUB DISPLACEMENT");

            if (angle > abs(target_angle))
            {
                reset();
                displacex = 0;
                displacey = 0;
                prev_angle += angle;
                angle = 0;
                flag = true;
                break;
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void publish_motors()
    {
        vmxpi_ros::MotorSpeed msg1;

        msg1.request.speed = rightSpeed;
        msg1.request.motor = 0;
        set_m_speed.call(msg1);

        msg1.request.speed = backSpeed;
        msg1.request.motor = 1;
        set_m_speed.call(msg1);

        msg1.request.speed = leftSpeed;
        msg1.request.motor = 3;
        set_m_speed.call(msg1);
    }

    void callback(vmxpi_ros_bringup::MotorSpeedConfig &config, uint32_t level) {
        std_srvs::Trigger msg;
        if (config.enabled)
            enable_client.call(msg);
        else
            disable_client.call(msg);
        
        //setMovements(1.0, 45.0);
        //ros::Duration(1.0).sleep(); // sleep for 1 second
        //setMovements(1.0, 225.0);
        
        //setMovements(1.0, 60.0);
        //ros::Duration(1.0).sleep(); // sleep for 1 second
        //setMovements(1.0, 240.0);

        /*while (ros::ok())
        {
            holonomicDrive(0.0, 0.0, 0.0);
            publish_motors();
            encoder2dist();
            PubDisplacements();
        }*/

        // Uncomment to set the movements of the robot
        for (int i = 0; i < 1; i++)
        {
             reset();
             setMovements(1.0, 0.0); // (magnitude (m), angle (deg))
             ros::Duration(1.5).sleep(); // sleep for 1 second
             
             setMovements(1.0, 90.0);
             ros::Duration(1.5).sleep(); // sleep for 1 second
             
             setMovements(sqrt(2.0), 225.0);
             ros::Duration(1.5).sleep(); // sleep for 1 second
             
        }

        // Uncomment for OpenCV Cube Tracking Demo
        /* vmxpi_ros::MotorSpeed msg1;

         config.motor0_speed = ocv_rightSpeed;
         msg1.request.speed = config.motor0_speed;
         msg1.request.motor = 0;
         set_m_speed.call(msg1);

         config.motor1_speed = ocv_backSpeed;
         msg1.request.speed = config.motor1_speed;
         msg1.request.motor = 1;
         set_m_speed.call(msg1);

         msg1.request.speed = config.motor2_speed;
         msg1.request.motor = 2;
         set_m_speed.call(msg1);

         config.motor3_speed = ocv_leftSpeed;
         msg1.request.speed = config.motor3_speed;
         msg1.request.motor = 3;
         set_m_speed.call(msg1);
         */
    }

};

// Callbacks for joystick axes
/*
 *Takes in left joystick input 
 */
void joystick_axes_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
   joystick_axes = msg->data;
}

/**
 * Callbacks for reading from plswork topic
*/
void plswork_callback(const std_msgs::String::ConstPtr& msg)
{
    axis_message = msg->data;
}

//Callbacks for keyboard input
void keyboard_typed_callback(const std_msgs::String::ConstPtr& msg)
{
    keyboard_typed = msg->data;
}

void AxisValues_callback(const std_msgs::Float32::ConstPtr& msg)
{
    AxisValue = msg->data;
}
 
int main(int argc, char **argv) {
    
    system("/usr/local/frc/bin/frcKillRobot.sh");
    
    ROS_INFO_STREAM("Main thread: " << syscall(SYS_gettid));

    ros::init(argc, argv, "movement_node");

    ros::NodeHandle nh;
    VMXPi vmx(true, (uint8_t)50);
    VMXErrorCode vmxerr;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    TitanDriverROSWrapper titan(&nh, &vmx);
    ROS_INFO("Titan driver is now started");

    navXROSWrapper navx(&nh, &vmx);
    ROS_INFO("navX driver is now started");

    ROS_INFO("BEFORE INSTANTIATING MOVEMENTS");

    ros::Duration(10.0).sleep();

    // ros::Subscriber keyboard_input_sub;
    // keyboard_input_sub = nh.subscribe("/keyboard_inputs", 1, keyboard_typed_callback)
    
    DynamicReconfig robot(&nh);

    // ros::Subscriber joystick_axes_sub;
    
    // // Subscribing to joy axes topic to access the controller data
    // joystick_axes_sub = nh.subscribe("/joy", 1, joystick_axes_callback);

    // Subscribes to JoystickAxisTopic
    ros::Subscriber axismessage_sub = nh.subscribe<std_msgs::String>("JoystickAxisTopic",1,plswork_callback);
    ros::Subscriber axisvalue_sub = nh.subscribe<std_msgs::Float32>("AxisValue",1,AxisValues_callback);

    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        ros::spinOnce();  // Call any pending callbacks
        ROS_INFO(" CURRENTLY READING CONTROLLER INPUT ");
        if(axis_message == "Y Axis Positive")
        {
            //forward
            ROS_INFO(" forwards ");
            robot.holonomicDrive(0.0, -AxisValue, 0.0);
            robot.publish_motors();
        } 
        else if(axis_message == "Y Axis Negative")
        {
            //backwards
            ROS_INFO(" backward ");
            robot.holonomicDrive(0.0, -AxisValue, 0.0);
            robot.publish_motors();
        }
        else if(axis_message == "X Axis Positive")
        {
            //right
            ROS_INFO(" right ");
            robot.holonomicDrive(AxisValue, 0.0, 0.0);
            robot.publish_motors();
        }
        else if(axis_message == "X Axis Negative")
        {
            //left
            ROS_INFO(" left ");
            robot.holonomicDrive(AxisValue, 0.0, 0.0);
            robot.publish_motors();
        }
        else if(axis_message == "Rotation Clockwise")
        {
            //clockwise rotation
            ROS_INFO(" clockwise ");
            robot.holonomicDrive(0.0, 0.0, AxisValue);
            robot.publish_motors();
        }
        else if(axis_message == "Rotation Counter Clockwise")
        {
            //counter-clockwise rotation
            ROS_INFO(" counter-clockwise ");
            robot.holonomicDrive(0.0, 0.0, AxisValue);
            robot.publish_motors();
        }
        else
        {
            //stop
            robot.stop_motors();
        }
        loop_rate.sleep();
    }
    /*  
    while(ros::ok()){
    //ROS_INFO(" forwards ");
    //robot.setMovements(0.5,-90);  //forward
    //robot.stop_motors();
    // ros::Duration(2.0).sleep();

    //ROS_INFO(" left ");
    //robot.setMovements(0.5,0);   //left 
    //robot.stop_motors();
    // ros::Duration(2.0).sleep();
    

    //ROS_INFO(" backward ");
    //robot.setMovements(0.5,90);   //backwards
    //robot.stop_motors();
    // ros::Duration(2.0).sleep();
    

    //ROS_INFO(" right ");
    //robot.setMovements(0.5,180);   //right
    //robot.stop_motors();
    // ros::Duration(2.0).sleep();
    
    }
    */
    
    // robot.holonomicDrive(0,0,0);
    // robot.publish_motors();
    // robot.PIDReset();
    // robot.reset();
    

    ROS_INFO("AFTER MOVEMENTS");
    

    ROS_INFO("ROS SHUTDOWN");
    ros::waitForShutdown();
    return 0;
}
