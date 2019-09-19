# seu-unirobot-unofficial-guidance
东南大学RoboCup Kidsize 校队代码非官方使用指引

This project contains a controller, some debug tools and some scripts for seu-robocup kidszie team. The controller and debug tools are written in c++. The scripts are written in python.

## Build tools
* cmake >= 3.12 [install turtorial](https://www.linuxidc.com/Linux/2018-09/154165.htm)
* gcc
* g++
* nvcc

## Dependencies
(in the braces are recommended installation methods)

### c++ libraries
* cuda >= 9.0 (PLZ use [JetPack](https://docs.nvidia.com/jetson/archives/jetpack-archived/jetpack-33/index.html#jetpack/3.3/install.htm%3FTocPath%3D_____3) to install and skip the Jetson TX2 setup)
* cudnn >= 7.0 [install turtorial](https://blog.csdn.net/lucifer_zzq/article/details/76675239)
* opencv >= 3.3.1
* libeigen3-dev (`sudo apt-get install libeigen3-dev`)
* libboost-all-dev (`sudo apt-get install libboost-all-dev`)
* freeglut3-dev
* libv4l-dev (`sudo apt-get install libv4l-dev`)
* MVSDK (library for our camera `sudo ./MVSDK/install.sh #at the project dir, and PLZ restart after installation`)
* qt5-default (`sudo apt-get install qt5-default`)
* astyle (`sudo apt-get install astyle`)

You can use Jetpack-3.3 to install cuda, opencv and cross compiler tools.

### python3 libraries
* python3-paramiko
* transitions

## Compile & Run

### Compile for x86_64
```Bash
cd path/to/project
chmod +x x86_64-build.py
./x86_64-build.py
```
* Then you find the executable files in bin/x86_64
* You can run with ./exe_name -h to get infomation about how to use

### Cross compile for aarch64
```Bash
cd path/to/project
./aarch64-build.py
```
* Then you find the executable files in bin/aarch64
* If you want to run program, you should connect with robot, then use the script start_robot.py in bin/

## Recommend OS
* ubuntu 16.04 64bit

## Recommend IDE
* Visual Studio Code(with c++ and python plugin)
* Clion(maybe better with a cmake built-in)

## Binocular Vision
[小觅双目相机SDK文档](https://buildmedia.readthedocs.org/media/pdf/mynt-eye-s-sdk-docs-zh-cn/latest/mynt-eye-s-sdk-docs-zh-cn.pdf)

