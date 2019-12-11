# Overview

- software frame

![image](http://files.amovauto.com:8088/group1/default/20191208/14/41/1/sofe_frame.png)

- Dir Tree

  ![image](http://files.amovauto.com:8088/group1/default/20191208/14/43/1/some.png)
# Simulation

此simulation 包含2D、3D激光雷达模型、深度相机模型、双目相机模型、realsense相机模型、IRlock相机模型。

![image](http://files.amovauto.com:8088/group1/default/20191206/18/07/1/gazebo_world.jpg)

![image](http://files.amovauto.com:8088/group1/default/20191209/15/50/1/sensor_rviz.png)

- 配置PX4以及ros环境

- 下载gazebo模型包

- 编译工作空间，运行launch文件

下载源码：

```
git clone https://gitee.com/bingobinlw/some
```

## 配置PX4以及ros环境

在ubuntu 16.04或18.04已测试通过

建议安装gazebo9

这里给出ubuntu16.04安装步骤

### ROS

#### for ubuntu16.04 kinetic

1. Add ROS to sources.list.

   ```bash
   echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
   sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
   sudo apt update
   ```

1. Install gazebo with ROS.

   ```bash
   sudo apt-get install ros-kinetic-desktop
   
   # Source ROS
   source /opt/ros/kinetic/setup.bash
   ```

  Full installation of ROS Kinetic comes with Gazebo 7.

  If you are using different version of Gazebo,

  please make sure install ros-gazebo related packages

  For Gazebo 8,

  ```
  sudo apt install ros-kinetic-gazebo8-*
  ```

  For Gazebo 9,

  ```
  sudo apt install ros-kinetic-gazebo9-*
  ```

1. Initialize rosdep.

   ```bash
   rosdep init
   rosdep update
   ```

1. Install catkin and create your catkin workspace directory.

   ```bash
   sudo apt install python-catkin-tools
   mkdir -p ~/catkin_ws/src
   ```

1. Install mavros version 0.29.0 or above. Instructions to install it from sources can be found here: https://dev.px4.io/en/ros/mavros_installation.html. If you want to install using apt, be sure to check that the version is 0.29.0 or greater.

   ```bash
   sudo apt install ros-kinetic-mavros ros-kinetic-mavros-extras
   ```

1. Install the geographiclib dataset

   ```bash
   wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
   chmod +x install_geographiclib_datasets.sh
   sudo ./install_geographiclib_datasets.sh
   ```


### 下载编译px4 Firmware

在此目录下下载px4源码并切换v1.9.2的固件

```
cd ~/some
git clone https://github.com/PX4/Firmware
```

或下载码云中的px4源码

```
cd ~/some
git clone https://gitee.com/bingobinlw/Firmware
```

然后更新submhttp://files.amovauto.com:8088/group1/default/20191211/14/11/1/gmapping_slam.pngodule 切换固件并编译

```
cd Firmware
git submodule update --init --recursive
git checkout v1.9.2
make distclean
make px4_sitl_default gazebo
```

编译成功后运行`source_environment.sh`添加Firmware环境变量,以及gazebo模型路经

```
source source_enviroment.sh
```



## 下载gazebo模型包

  在home目录下创建**gazebo_models**文件夹

```
youname@ubuntu:~$ mkdir gazebo_models
```

下载gazebo模型包 https://bitbucket.org/osrf/gazebo_models/downloads/

把gazebo模型包解压出来的所有模型文件剪切至**gazebo_models**文件夹

添加**gazebo_models**文件夹路经

```
echo "export GAZEBO_MODEL_PATH=:~/gazebo_models" >> ~/.bashrc
source ~/.bashrc
```

然后在任意终端中打开gazebo

```
gazebo
```

在gazebo 界面insert选项中确保**gazebo_models**模型包被跟踪

## 编译工作空间，运行launch文件

```
cd ~/some
catkin_make
添加bash路经
echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

运行model demo launch文件

```
roslaunch simulation models_demo_test_px4.launch
```

# slam

运行slam-Demo之前请先安装必要的功能包，具体请看

```
roscd ros_slam
查看README.md
```

## gmapping_slam

运行

```
roslaunch simulation gmapping_demo_px4.launch
```

![image](http://files.amovauto.com:8088/group1/default/20191211/14/11/1/gmapping_slam.png)

同时会出现一飞机控制界面，要想使用此脚本请先查看下面路经的README.md

```
dir:some/src/simulation/scripts/README.md
```



![image](http://files.amovauto.com:8088/group1/default/20191211/14/17/1/keyboard_control.png)