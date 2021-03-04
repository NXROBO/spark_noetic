# NXROBO Spark
<img src="http://wiki.ros.org/Robots/Spark?action=AttachFile&do=get&target=spark1.png" width="300">

## 说明 Description
- This is a tutorial for beginners, and Detailed version is [here](https://github.com/NXROBO/spark/blob/master/README_Detailed.md) . 
- 本说明为初学者体验版，[这里](https://github.com/NXROBO/spark_noetic/blob/master/README_Detailed.md)有详细说明的版本。

## 列表 Table of Contents

* [功能包说明packages-overview](#功能包说明packages-overview)
* [使用usage](#使用usage)
* [视频展示Video](#视频展示Video)

## 功能包说明packages-overview

* ***src*** : Spark的源代码，包括底层配置，硬件驱动，和各个应用功能包等。
* ***doc*** : 软硬件依赖包。

## 使用usage

### 系统要求 Prequirement

* System:	Ubuntu 20.04+
* ROS Version:	noetic(Desktop-Full Install) 

### 下载安装 Download and install

* 下载工作空间 Download the workspace:
```yaml
git clone https://github.com/NXROBO/spark_noetic.git
```
* 安装依赖库 Install libraries and dependencies:
```yaml
cd spark_noetic
```

### 编译运行 compile and run
```yaml
catkin_make
```
* 如果编译一切正常，可根据提示运行相关例程。If everything goes fine, test the examples as follow:
```yaml
./onekey.sh
```

## 视频展示Video

1.Spark跟随 Spark-Follower

<a href="https://www.youtube.com/embed/UrD2AEQ3VkI" target="_blank"><img src="http://img.youtube.com/vi/UrD2AEQ3VkI/0.jpg" 
alt="follow-person" width="240" height="180" border="10" /></a>
```yaml
cd spark
source devel/setup.bash
roslaunch spark_follower bringup.launch
```

2.Spark建图 Spark-SLAM-Mapping

<a href="https://www.youtube.com/embed/Yt9Sld-EX0s" target="_blank"><img src="http://img.youtube.com/vi/Yt9Sld-EX0s/0.jpg" 
alt="follow-person" width="240" height="180" border="10" /></a>
```yaml
cd spark
source devel/setup.bash
roslaunch spark_slam 2d_slam_teleop.launch slam_methods_tel:=gmapping
```

3.Spark导航 Spark-Navigation

<a href="https://www.youtube.com/embed/3RP11sZKfJg" target="_blank"><img src="http://img.youtube.com/vi/3RP11sZKfJg/0.jpg" 
alt="follow-person" width="240" height="180" border="10" /></a>
```yaml
cd spark
source devel/setup.bash
roslaunch spark_navigation amcl_demo_lidar_rviz.launch
```

4.Spark-RtabMap建图 Spark-RtabMap-Mapping

<a href="https://www.youtube.com/embed/K5wvlWb-2uQ" target="_blank"><img src="http://img.youtube.com/vi/K5wvlWb-2uQ/0.jpg" 
alt="follow-person" width="240" height="180" border="10" /></a>
```yaml
cd spark
source devel/setup.bash
roslaunch spark_rtabmap spark_rtabmap_teleop.launch 
```

5.Spark机械臂视觉抓取 Spark-Carry_Object

<a href="https://www.youtube.com/embed/aNPy6GYcdu0" target="_blank"><img src="http://img.youtube.com/vi/aNPy6GYcdu0/0.jpg" 
alt="follow-person" width="240" height="180" border="10" /></a>
```yaml
cd spark
source devel/setup.bash
roslaunch spark_carry_object spark_carry_object_only_cv3.launch 
```


