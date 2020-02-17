- Overview

- Simulation

- Px4_command

- Slam

- map

- image_process

- planning

- P200

- AmovCar

项目介绍：该项目给PX4无人机仿真开发提供了一套完整的环境和程序接口，ROS下的Gazebo仿真一键起飞，Offboard模式自动飞行，基于激光雷达的VFH避障，激光SLAM/视觉SLAM等。
课程和项目介绍详情地址：在阿木实验室微信公众号上

 此项目源码在这里，在编译之前需要安装好PX4开发环境，ROS。Gazebo9，MAVROS。是一套完整的框架，可以实现无人机仿真系统下Gazebo的模拟飞行，激光雷达VHF避障等算法。
 运行这个仿真环境的硬件要求：
 - 台式电脑I5以上，固态硬盘128G，内存8G以上。
 - 笔记本电脑四核8线程CUP，推荐I7，固态硬盘128G，内存8G以上。

 同时我们推荐一台仅仅350克的机载计算机，手掌大小，可以同时完成Gazebo仿真开发，包含了PX4开发环境，ROS环境等，可以直接运行本例程所有仿真和代码。同时可以安装在无人机/无人车设备上作为机载计算机，X86构架，八代i7 CPU(8565)，固态硬盘128G，内存8G。

-  详情和链接购买地址：https://item.taobao.com/item.htm?id=611800776364

 软件要求：Ubuntu18.04环境

 要确保PX4开发环境，ROS，Gazebo9，MAVROS可以运行，如果安装环境有困难，或者是PX4无人机开发的初学者，请去铂贝学院购买视频课程和完整的ISO系统镜像工具包。
 地址：https://bbs.amovlab.com/plugin.php?id=zhanmishu_video:video&mod=video&cid=18

  运行各demo之前，请先更新一下仓库:

  ```
  git pull
  ```
  