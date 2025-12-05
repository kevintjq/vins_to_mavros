#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

Eigen::Vector3d p_mav(0,0,0);
Eigen::Quaterniond q_mav(1,0,0,0);

void vins_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // 直接使用 VINS 的位置
    p_mav = Eigen::Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
    );

    // 直接使用 VINS 的四元数
    q_mav = Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_to_mavros");
    ros::NodeHandle nh("~");

    // 订阅 VINS ODOM （记得在 launch 中 remap）
    ros::Subscriber slam_sub =
        nh.subscribe<nav_msgs::Odometry>("odom", 10, vins_callback);

    // 发布到 mavros/vision_pose/pose
    ros::Publisher vision_pub =
        nh.advertise<geometry_msgs::PoseStamped>("vision_pose", 10);

    ros::Rate rate(100.0);

    while (ros::ok())
    {
        geometry_msgs::PoseStamped vision;

        vision.header.stamp = ros::Time::now();
        vision.header.frame_id = "world";   // PX4 要求 world/map

        // 位置
        vision.pose.position.x = p_mav.x();
        vision.pose.position.y = p_mav.y();
        vision.pose.position.z = p_mav.z();

        // 姿态
        vision.pose.orientation.w = q_mav.w();
        vision.pose.orientation.x = q_mav.x();
        vision.pose.orientation.y = q_mav.y();
        vision.pose.orientation.z = q_mav.z();

        vision_pub.publish(vision);

        // 打印调试信息
        ROS_INFO("\nposition:\n"
                 "   x: %.6f\n   y: %.6f\n   z: %.6f\n"
                 "orientation:\n"
                 "   x: %.6f\n   y: %.6f\n   z: %.6f\n   w: %.6f",
                 p_mav.x(), p_mav.y(), p_mav.z(),
                 q_mav.x(), q_mav.y(), q_mav.z(), q_mav.w());

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
