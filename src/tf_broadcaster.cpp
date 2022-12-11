#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>

std::string robot_frame_name = "base"; 

void robot_poseCallback(const geometry_msgs::Pose2DConstPtr& pose_msg)
{
    static tf2_ros::TransformBroadcaster brodcaster; 
    geometry_msgs::TransformStamped transform_msg; //header, child_frame_id, transform

    transform_msg.header.stamp = ros::Time::now();
    transform_msg.header.frame_id = "world"; // std::header w wiadomosci TransforStampped składa się z time stamp oraz frame id który odnosi się do parrent frame id 
    
    transform_msg.child_frame_id = robot_frame_name;

    transform_msg.transform.translation.x = pose_msg->x; 
    transform_msg.transform.translation.y = pose_msg->y; 
    transform_msg.transform.translation.z =0.0; //Pose2D ma tylko x i y

    tf2::Quaternion quat; 
    quat.setRPY(0,0,pose_msg->theta); //setRPY-roll, pitch, yawn

    transform_msg.transform.rotation.x = quat.x(); //quat jest wskaznikiem 
    transform_msg.transform.rotation.y = quat.y();
    transform_msg.transform.rotation.z = quat.z();
    transform_msg.transform.rotation.w = quat.w();

    brodcaster.sendTransform(transform_msg); 
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf2_broadcaster");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("mrosBot/pose", 10, &robot_poseCallback); 


    ros::spin();
    return 0; 
}
