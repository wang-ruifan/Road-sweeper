# 路面清扫车项目

## 项目简介
这是一个大型路面清扫车的项目，基于Ubuntu 18.04，ROS Melodic，Autoware1.14。
使用的控制板为NVIDIA Jetson Xavier NX，底盘为线控底盘，使用CAN进行通信。
本仓库包含了修改后的Autoware代码、Autoware的底盘控制指令与CAN进行对接的包，激光雷达和IMU/GNSS驱动包，地图文件等。

## 目录
- [项目简介](#项目简介)
- [功能描述](#功能描述)
- [安装指南](#安装指南)
- [使用说明](#使用说明)
- [使用Autoware源代码遇到的问题](#使用autoware源代码遇到的问题)
- [Autoware 源代码修改](#Autoware-源代码修改)


## 功能描述
该项目的主要功能包括：
- 基于Autoware实现路面清扫车的自动驾驶控制
- 通过CAN接口使ROS与底盘进行通信
- 在ROS中读取3D激光雷达与IMU/GNSS的数据

## 安装指南
请按照以下步骤安装和配置项目：


## 使用说明

## 使用Autoware源代码遇到的问题

1. 

## Autoware 源代码修改
为了解决上述问题，我对Autoware的源代码进行了以下修改：



### GNSS信息转换节点：修改所在地经纬度，增加转换矩阵
修改了[GNSS信息转换为pose](src/autoware/common/gnss/src/geo_pos_conv.cpp)的set_plane方法，在当传入的num为7时，设置纬度和经度为建图时录制的bag的/gps/fix话题的第一帧数据转换为度分秒后得到的度和分。同时在conv_llh2xyz经纬度转换为位姿方法的最后添加使用旋转矩阵和平移向量进行计算。

### NDT定位节点：修改使用GNSS辅助定位时的位姿重置逻辑
修改了[NDT-Matching定位](src/autoware/core_perception/lidar_localizer/nodes/ndt_matching/ndt_matching.cpp)的gnss_callback回调函数的业务逻辑，直接使用当前的gnss位姿信息，发布初始位姿。同时修改调用gnss重置位姿的阈值为1，并添加一个初始位姿发布者initialpose_pub并设置发布话题名为/initialpose。

### 点云滤波跟踪节点：修改订阅话题
修改了[点云卡尔曼滤波跟踪](src/autoware/core_perception/lidar_kf_contour_track/nodes/lidar_kf_contour_track/lidar_kf_contour_track_core.cpp)的sub_cloud_clusters订阅者订阅的话题名，从/cloud_clusters修改为/detection/lidar_detector/cloud_clusters。

### 点云欧式聚类节点：增加发布信息
修改了[雷达点云欧式聚类](src/autoware/core_perception/lidar_euclidean_cluster_detect/nodes/lidar_euclidean_cluster_detect/lidar_euclidean_cluster_detect.cpp)的publishCloudClusters方法内的点云聚类发布消息的内容，增加了对聚类转换的平均点、状态和id的赋值。


