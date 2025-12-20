#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

ros::Publisher vision_pub;
int msg_count = 0;

void vins_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    geometry_msgs::PoseStamped vision_pose;
    vision_pose.header.stamp = msg->header.stamp;
    vision_pose.header.frame_id = "world";  // 添加frame_id
    vision_pose.pose = msg->pose.pose;
    
    vision_pub.publish(vision_pose);
    
    // 可选：启动时打印一次确认
    if(++msg_count == 1) {
        ROS_INFO("Started publishing vision pose from imu_propagate");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_to_mavros");
    ros::NodeHandle nh("~");
    
    ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>(
        "/vins_fusion/imu_propagate", 100, vins_callback);
    
    vision_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/mavros/vision_pose/pose", 10);
    
    ROS_INFO("Waiting for VINS imu_propagate data...");
    ros::spin();
    
    return 0;
}