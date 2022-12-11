#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose2D.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <iostream>

int left_pos = 0, right_pos = 0;
double right_wheel_pos = 0.0, left_wheel_pos = 0.0, prevLpos = 0.0, prevRpos=0.0; 
float wheelR, platformWidth = 0.14; // m  

float x = 0.0, y = 0.0, th = 0.0; 
float dx = 0.0, dy= 0.0, dth = 0.0, dt; 
double l_wheel_vel = 0, r_wheel_vel = 0; 
double bot_velocity = 0; 

void posA_callback(const std_msgs::Int32& msg_posA)
{
    left_pos = -msg_posA.data;
    left_wheel_pos += left_pos/4150.0; //m
    ROS_INFO("pozycja A : %f", left_wheel_pos);
}

void posB_callback(const std_msgs::Int32& msg_posB)
{
    right_pos= msg_posB.data; 
    right_wheel_pos += right_pos/4150.0; 
    ROS_INFO("pozycja B : %f", right_wheel_pos);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle node_obj; 

    int freq = 50; 
    ros::Rate loop_rate(freq); 

    ros::Subscriber posA_subscriber = node_obj.subscribe("/arduino_msgs/posA",10,posA_callback);
    ros::Subscriber posB_subscriber = node_obj.subscribe("/arduino_msgs/posB",10,posB_callback);
    ros::Publisher pose_publisher = node_obj.advertise<geometry_msgs::Pose2D>("mrosBot/pose",50);
    geometry_msgs::Pose2D pose_msg; 

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    while(ros::ok())
    {
        dt = (current_time - last_time).toSec();

        r_wheel_vel = (right_wheel_pos - prevRpos)/dt; 
        l_wheel_vel = (left_wheel_pos - prevLpos)/dt; 
        bot_velocity = (r_wheel_vel + l_wheel_vel)/2; 

        dth = ((r_wheel_vel - l_wheel_vel)/platformWidth)*dt; // m/s /m
        dx = (bot_velocity*cos(th))*dt; 
        dy = (bot_velocity*sin(th))*dt; 

        x += dx; 
        y += dy; 
        th += dth;   

        pose_msg.x = x;
        pose_msg.y = y; 
        pose_msg.theta = th; 

        prevRpos = right_wheel_pos;
        prevLpos = left_wheel_pos;

        pose_publisher.publish(pose_msg);

        ros::spinOnce();
        loop_rate.sleep();  
    }
    return 0; 
}