#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <fcntl.h>          // FILE control
#include <termios.h>        // Terminal IO


#define FORWARD                     0x77 // w
#define BACK                        0x73 // s
#define LEFT                        0x61 // a
#define RIGHT                       0x64 // d
#define STOP                        0x78 // x

double r_speed = 0; 
double l_speed = 0; 
double ws_speed_step = 10, lr_speed_step = 5; 
double max_speed = 80; 
float linear_xv, linear_yv, linear_zv; 
float angular_zv, angular_xv, angular_yv;

int getch(void)
{
    struct termios oldt, newt;
    int ch;
    tcgetattr( STDIN_FILENO, &oldt );
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 1;
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
    return ch;
}

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
      ungetc(ch, stdin);
      return 1;
    }
    return 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_motor_control"); // inicjalizacja node'a
    ros::NodeHandle nh;

    int freq = 100; // ustawienie czestotliwosci 
    ros::Rate loop_rate(freq);
    
    std_msgs::Int32 right_speed_msg; 
    std_msgs::Int32 left_speed_msg; 
    ros::Publisher right_speed_msg_pub= nh.advertise<std_msgs::Int32>("motor/right_speed", 10); //
    ros::Publisher left_speed_msg_pub= nh.advertise<std_msgs::Int32>("motor/left_speed", 10); //

    std::string msg =
    "\n\
    Sterowanie\n\
    w - przod\n\
    s - tyl \n\
    a - lewo\n\
    d - prawo\n\
    x -stop\n\
    \n\
    ";

    ROS_INFO("%s", msg.c_str());
    
    
    while (ros::ok())
    { 
        if (kbhit())
        {
            char c = getch(); // wywolanie funkcji zbierajacej stan klawiatury 

            switch (c)
            {
            case FORWARD: 
                l_speed += ws_speed_step; 
                r_speed += ws_speed_step; 
                if(l_speed > max_speed) {l_speed = max_speed;}
                if(r_speed > max_speed) {r_speed = max_speed;}

                left_speed_msg.data = l_speed;
                right_speed_msg.data = r_speed;
                break; 

            case BACK: 
                l_speed -= ws_speed_step; 
                r_speed -= ws_speed_step; 
                if(l_speed < -max_speed) {l_speed = -max_speed;}
                if(r_speed < -max_speed) {r_speed = -max_speed;}

                left_speed_msg.data = l_speed;
                right_speed_msg.data = r_speed;
                break;

            case LEFT: 
                l_speed += lr_speed_step; 
                r_speed -= lr_speed_step; 
                if(l_speed > max_speed) {l_speed = max_speed;}
                if(r_speed < -max_speed) {r_speed = -max_speed;}

                left_speed_msg.data = l_speed;
                right_speed_msg.data = r_speed;
                break; 

            case RIGHT: 
                l_speed -= lr_speed_step; 
                r_speed += lr_speed_step; 
                if(l_speed < -max_speed) {l_speed = -max_speed;}
                if(r_speed > max_speed) {r_speed = max_speed;}

                left_speed_msg.data = l_speed;
                right_speed_msg.data = r_speed;
                break;

            case STOP:
                l_speed = 0; 
                r_speed = 0; 

                left_speed_msg.data = l_speed;
                right_speed_msg.data = r_speed;
                break; 

            default:
                break;
            }       
        }
        right_speed_msg_pub.publish(right_speed_msg); 
        left_speed_msg_pub.publish(left_speed_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}