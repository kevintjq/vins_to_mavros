# VINS → MAVROS Vision Pose 转发模块

将 **VINS-Fusion 输出的原始位姿数据** 转发到  
`mavros/vision_pose/pose`，用于 **PX4 无人机视觉定位**（Vision Position Estimate）

## 📌 功能简介

- 订阅 VINS-Fusion 输出的位姿（Odometry / Pose）
- 转换为 `geometry_msgs/PoseStamped` 格式
- 发布至 MAVROS 的 `mavros/vision_pose/pose`
- 用于 PX4 EKF2 的视觉定位输入（VPE）

## 🚀 使用方式

1. 启动 VINS-Fusion  
2. 启动本节点进行转发  
3. PX4 将自动从 MAVROS 接收视觉定位数据
