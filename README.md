# Lidar_ROS_ZoneLimit

这个项目实现了基于激光雷达数据的区域限制功能，使用 ROS（机器人操作系统）框架开发。

## 环境要求

- 操作系统：Ubuntu 16.04 LTS
- ROS 版本：ROS Noetic

## 依赖

在运行此项目之前，请确保您的系统中已安装以下依赖：

- ROS Noetic（完整安装）

## 安装

1. 确保您的系统满足上述环境要求。

2. 克隆此仓库到您的 ROS 工作空间的 src 目录：

   ```bash
   cd Lidar_ROS_ZoneLimit
   catkin_make
   ```
## 运行
1. 启动 ROS 主节点：
   ```bash
   roscore
   ```
2. 在另一个终端中，启动激光雷达节点：
   or
   通过rqt_bag播放record文件夹
3. 在另一个终端中，启动区域限制节点：
   ```bash
   pythoh3 markRegion.py
   ```
4. 在另一个终端中，启动障碍物检测节点：
   ```bash
   pythoh3 obstacleDetection.py
   ```
