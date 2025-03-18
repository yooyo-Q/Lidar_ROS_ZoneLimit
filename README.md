# Lidar_ROS_ZoneLimit

这个项目实现了基于激光雷达数据的区域限制功能，使用 ROS（机器人操作系统）框架开发。

## 环境要求

- 操作系统：Ubuntu 16.04 LTS
- ROS 版本：ROS Noetic

## 依赖

在运行此项目之前，请确保您的系统中已安装以下依赖：

- ROS Noetic（完整安装）

## 安装
确保您的系统满足上述环境要求。


## 运行
1. 启动 ROS 主节点：
   ```bash
   roscore
   ```
2. 在另一个终端中，启动激光雷达节点：
   2.1 catkin_make编译激光雷达驱动
   ```
   source devel/setup.bash
   catkin_make
   roslaunch lidar_driver lidar_driver.launch   
   ```
   or
   通过bag包播放record
   rosbag play record/2023-09-14-13-46-30.bag

   
3. 在另一个终端中，启动区域限制节点：
   ```bash
   python markRegion.py
   ```
4. 在另一个终端中，启动障碍物检测节点：
   ```bash
   python obstacleDetection.py
   ```
