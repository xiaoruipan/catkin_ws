#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/Joy.h>
#include <string>
#include <std_msgs/String.h>
#include <iostream>
#include <termios.h>

#define LEFT_ARROW 0x44
#define UP_ARROW 0x41
#define DOWN_ARROW 0x42
#define RIGHT_ARROW 0x43

std::string getKeyInput()
{
    termios oldSettings, newSettings;
    tcgetattr(STDIN_FILENO, &oldSettings);
    newSettings = oldSettings;
    newSettings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);

    std::string key;
    char c;
    char arrowSequence[3] = {0};

    // Read the first character
    c = getchar();

    if (c == '\x1B') // Check if the first character is an escape character
    {
        // Read the second and third characters
        arrowSequence[0] = getchar();
        arrowSequence[1] = getchar();
        
        // Check if the sequence corresponds to an arrow key
        if (arrowSequence[0] == '[')
        {
            switch (arrowSequence[1])
            {
                case 'A':
                    key = "UP";
                    break;
                case 'B':
                    key = "DOWN";
                    break;
                case 'C':
                    key = "RIGHT";
                    break;
                case 'D':
                    key = "LEFT";
                    break;
            }
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);

    return key;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Keyboard_Input_Node");
    ros::NodeHandle nh;
    ros::Publisher KeyboardInputPub = nh.advertise<std_msgs::String>("plswork", 1);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {  
        std_msgs::String key;
        key.data = getKeyInput();
        KeyboardInputPub.publish(key);

        loop_rate.sleep();
    }

    return 0;
}
