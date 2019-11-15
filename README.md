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

然后更新submodule 切换固件并编译

```
cd Firmware
git submodule update --init --recursive
git checkout v1.9.2
make distclean
make px4_sitl_defaule gazebo
```

编译成功后运行`source_environment.sh`添加环境变量

```
source source_enviroment.sh
```

