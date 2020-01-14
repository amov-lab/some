- Overview

- Simulation

- Px4_command

- Slam

- image_process

- planning

- P200

- AmovCar

  此项目运行环境可以手动安装，或下载已安装好了的iso镜像进行虚拟机安装或实体机安装，链接如下：

  链接：https://pan.baidu.com/s/1yMvZXezQSPDPkYmSdbEZLA
  提取码：y3rr

  运行各demo之前，请先更新一下仓库:

  ```
  git pull
  ```
  **注：有任何疑问都可在issues提问:)**

# Overview

- software frame

![image](http://files.amovauto.com:8088/group1/default/20191208/14/41/1/sofe_frame.png)

# Simulation

此simulation 包含2D、3D激光雷达模型、深度相机模型、双目相机模型、realsense相机模型、IRlock相机模型。

![image](http://files.amovauto.com:8088/group1/default/20191206/18/07/1/gazebo_world.jpg)

![image](http://files.amovauto.com:8088/group1/default/20191209/15/50/1/sensor_rviz.png)

- 配置PX4以及ros环境

- 下载gazebo模型包

- 编译工作空间，运行launch文件



## 配置PX4以及ros环境

在18.04已测试通过

建议安装Ubuntu18.04 ,gazebo9

这里给出ubuntu18.04安装步骤

### ROS

#### for ubuntu18.04 melodic

1. Add ROS to sources.list.

   ```bash
   echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
   sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
   sudo apt update
   ```

2. Install gazebo with ROS.

   ```bash
   sudo apt-get install ros-melodic-desktop
   
   # Source ROS
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
   ```

  please make sure install ros-gazebo related packages


  For Gazebo 9,

  ```
  sudo apt install ros-melodic-gazebo9*
  ```

3. Initialize rosdep.

   ```bash
   rosdep init
   rosdep update
   ```

4. Install catkin.

   ```bash
   sudo apt install python-catkin-tools
   ```

4. Install mavros version 0.29.0 or above. Instructions to install it from sources can be found here: https://dev.px4.io/en/ros/mavros_installation.html. If you want to install using apt, be sure to check that the version is 0.29.0 or greater.

   ```bash
   sudo apt install ros-melodic-mavros ros-melodic-mavros-extras
   ```

5. Install the geographiclib dataset

   ```bash
   wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
   chmod +x install_geographiclib_datasets.sh
   sudo ./install_geographiclib_datasets.sh
   ```

参考于：http://wiki.ros.org/melodic/Installation/Ubuntu

### 下载编译px4 Firmware

- 安装必要的工具链

- 下载编译firmware

#### 安装必要的工具链

下载**ubuntu.sh,requirements.txt**

```
wget https://raw.githubusercontent.com/PX4/Firmware/master/Tools/setup/ubuntu.sh

```

```
wget https://raw.githubusercontent.com/PX4/Firmware/master/Tools/setup/requirements.txt
```

然后运行：
```
source ubuntu.sh
```

参考于http://dev.px4.io/master/en/setup/dev_env_linux_ubuntu.html

下载源码：

```
git clone https://gitee.com/bingobinlw/some
```
#### 下载编译firmware

在此目录下下载px4源码并切换v1.9.2的固件

```
cd some
git clone https://github.com/PX4/Firmware
```

或下载码云中的px4源码

```
cd some
git clone https://gitee.com/bingobinlw/Firmware
```

然后更新submodule切换固件并编译

```
cd Firmware
git submodule update --init --recursive
git checkout v1.9.2
make distclean
make px4_sitl_default gazebo
```




## 下载gazebo模型包

  在home目录下创建**gazebo_models**文件夹

```
youname@ubuntu:~$ mkdir gazebo_models
```

下载gazebo模型包 https://bitbucket.org/osrf/gazebo_models/downloads/

把gazebo模型包解压出来的所有模型文件剪切至**gazebo_models**文件夹

## 编译工作空间，运行launch文件
编译之前，请先下载必要的slam包，具体请到ros_slam包中查看readme.md

```
dir:some/src/mid/slam/ros_slam
查看README.md
```

运行demo之前请先下载3Dlidar仿真相关的插件包


for ubuntu 18.04

```
sudo apt-get install ros-melodic-velodyne-gazebo-plugins
```

```
cd some
catkin_make

```


编译成功后运行`source_environment.sh`添加Firmware环境变量,some gazebo模型路经,gazebo_modles模型路经

```
source source_enviroment.sh
```


运行model demo launch文件

```
roslaunch simulation models_demo_test_px4.launch
```
# Px4_command

px4控制以及上层应用模块

## offboard 模式下走圆形轨迹

运行

```
roslaunch simulation circular_px4.launch
```
同时会出现一飞机控制界面，要想使用此脚本请先查看下面路经的README.md

```
dir:some/src/simulation/scripts/README.md
```



![image](http://files.amovauto.com:8088/group1/default/20191211/14/46/1/keyboard_control.png)
在键盘控制终端中，输入**0**解锁，然后输入**2**切**offboard**,飞机随后会按照你给定的半径与高度飞行，完成一圈后会自动降落。

## offboard 模式下进行二维VFH避障

运行demo之前，请先在QGC中添加航点，然后上传，VFH节点将读取航点信息，当作目标点。

然后重新运行

```
roslaunch simulation obstacle_avoidance_2Dlaser_vfh_px4.launch 
```

![image](http://files.amovauto.com:8088/group1/default/20200101/15/18/1/2dvfh.png)

中间终端为GCG中各航点的平面信息，读取航点成功后，在最后一个控制终端中输入**2**然后回车，VFH节点将开始工作。

# Slam

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

![image](http://files.amovauto.com:8088/group1/default/20191215/21/46/1/gmapping_map.png)


## cartographer
cartographer在2019年10月份已经支持以ros包形式安装。若想运行此demo请先安装必要cartogra包。具体请看ros_slam包中的**README.md**

### 2Dlidar location

运行demo之前请先在QGC参数表中配置参数，选择EKF位置来源来自板载计算机

```
EKF2_AID_MASK = 24
```

cartogra节点将接收2d激光雷达以及无人机的imu话题。

```
roslaunch simulation cartographer2Dlidar_location_demo_px4.launch
```

在定位之前请在键盘控制界面用键盘的**'g'**键调整uav的允许速度为1570，降低uav的运动时的倾斜角度以及速度，以达到更好的定位效果。

结果

![image](http://files.amovauto.com:8088/group1/default/20191215/22/03/1/carto_use_imu.png)

### 2Dlidar mapping

如果你想建立更加准确的地图，而且你的robot已经拥有里程计。那么cartogra能够生成准切而稳定的map，不会存在location模式中地图会飘的情况。

运行demo之前请先在QGC参数表中配置参数，选择EKF位置来源来自gps

```
EKF2_AID_MASK = 1
```

cartogra节点将接收2d激光雷达以及无人机的里程计话题

```
roslaunch simulation cartographer2Dlidar_mapping_demo_px4.launch
```

### 3Dlidar location

使用运行demo之前请先确保以安装3D雷达相关插件

for ubuntu 18.04

```
sudo apt-get install ros-melodic-velodyne-gazebo-plugins
```

使用一个16线的激光雷达，以及一个imu数据，激光雷达水平安装在飞机的顶部。就其定位效果来看，没有发现2Dlidar定位时会飘的情况，而且无人机速度倾斜角度都可以大幅提高。

![image](http://files.amovauto.com:8088/group1/default/20191217/15/31/1/carto_3D_rviz.png)

运行

```
roslaunch simulation cartographer3Dlidar_demo_px4.launch
```
## rtabmap slam
使用深度相机以及室内里程计，为了达到更好的建图效果，其中室内里程计选择用3Dcartogra.
运行demo之前请先安装必要的rtabmap ros包

```
sudo apt-get install ros-melodic-rtabmap-ros
```

请先在QGC参数表中配置参数，选择EKF位置来源来自板载计算机

```
EKF2_AID_MASK = 24
```

运行demo

```
roslaunch simulation rtabmap_depthCam_mapping_demo_px4.launch
```

参考于：http://wiki.ros.org/rtabmap_ros

建图效果：

![image](http://files.amovauto.com:8088/group1/default/20191230/22/37/1/rtabmap1.png)

![image](http://files.amovauto.com:8088/group1/default/20191230/22/38/1/rtabmap3.png)

# P200
待定

  **注：有任何疑问都可在issues提问:)**
