# 基于Jetson与Autoware的自动驾驶清扫车安装指南

**此文档仅为在Jetson Xavier环境下的安装指南，关于项目的详细说明等请前往[Docs分支](https://github.com/wang-ruifan/Road-sweeper/blob/docs/README.md)查看。**

**For English version, please [click me!](#road-sweeper-based-on-jetson-and-autoware-installation-guide)**

## 使用的Jetson环境

![Jetson环境](https://github.com/wang-ruifan/Road-sweeper/tree/docs/images/jetson-environment.png)  

- 注意：opencv一定不能是通过jetpack安装的4.x版本的，必须是安装ros Melodic时自带的opencv3.2，否则编译时会报一堆cv库错误。需要卸载jetpack安装的opencv或者不安装jetpack，手动安装CUDA，cuDnn，TensorRT。

## 1. Jetson环境配置

### 1.1. Jetson 刷机安装Ubuntu 18.04

请参考[CSDN博客](https://blog.csdn.net/weixin_47606814/article/details/127841948?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522568955dfc76993359b335c02fcf6e43f%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=568955dfc76993359b335c02fcf6e43f&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-127841948-null-null.142^v100^pc_search_result_base8&utm_term=jetson%20xavier%20nx%E7%83%A7%E5%BD%95%E7%B3%BB%E7%BB%9F&spm=1018.2226.3001.4187)  
注意：选择jetpack版本为4.x.x才是Ubuntu 18.04，同时只安装Jetson OS，并设置安装位置为nvme。

### 1.2. 安装jtop

```shell
sudo apt-get install python3-pip
pip3 install jetson-stats
```

之后在终端输入jtop即可查看CPU，GPU等信息。  

### 1.3. 安装CUDA

```shell
sudo apt-get install cuda-toolkit-10-2
```

安装完成后，打开.bashrc文件  

```shell
sudo gedit ~/.bashrc
```

在文件末尾添加  

```shell
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-10.2/lib64
export PATH=$PATH:/usr/local/cuda-10.2/bin
export CUDA_HOME=$CUDA_HOME:/usr/local/cuda-10.2
```

### 1.4. 安装cuDNN

首先查看仓库提供的cuDNN有哪些版本

```shell  
sudo apt-cache policy libcudnn8
```

我的Jetson在JetPack 4.6.1和cuda 10.2环境下，显示提供的libcudnn8只有一个版本，故可以直接安装  

```shell
sudo apt-get install libcudnn8
```

若有要选择版本的话

```shell
sudo apt-get install libcudnn8=（*.*.*.**+cuda*** ）
```

### 1.5 验证CUDA和cuDNN安装

使用jtop，在info中查看CUDA和cuDNN的版本，若显示正确则安装成功。

```shell
jtop
```

还可以运行python代码验证

```python
import torch
print(torch.__version__)
print('CUDA available: ' + str(torch.cuda.is_available()))
print('cuDNN version: ' + str(torch.backends.cudnn.version()))
a = torch.cuda.FloatTensor(2).zero_()
print('Tensor a = ' + str(a))
b = torch.randn(2).cuda()
print('Tensor b = ' + str(b))
c = a + b
print('Tensor c = ' + str(c))
```

**注：以上1.2~1.5步骤的内容参考了[CSDN博客](https://blog.csdn.net/a111222zw/article/details/120632906)并经过我自己的验证可以完成安装配置。**

### 1.6 安装TensorRT

首先查看仓库提供的TensorRT有哪些版本

```shell
sudo apt-cache policy tensorrt
```

**!!!注意!!!：TensorRT版本必须为7.x.x，否则会导致编译Autoware时出现错误。**  
选择版本为7.x.x，安装  

```shell
sudo apt-get install tensorrt=（7.*.*.**+cuda10-2 ）
```

**注：以上1.6步骤的内容参考了[CSDN博客](https://blog.csdn.net/weixin_43541510/article/details/130796360)中的单独安装TensorRT部分的内容并经过我自己的验证可以完成安装配置。**

### 1.7 安装Eigen3.3.7

安装系统后可能会自动安装Eigen，但是版本可能不是3.3.7，所以需要手动安装3.3.7版本。  
首先查看已安装版本  

```shell
gedit /usr/include/eigen3/Eigen/src/Core/util/Macros.h
```

或  

```shell
gedit /usr/local/include/eigen3/Eigen/src/Core/util/Macros.h
```

查看版本号，若显示内容为如下，则已安装3.3.7版本

```h
#define EIGEN_WORLD_VERSION 3
#define EIGEN_MAJOR_VERSION 3
#define EIGEN_MINOR_VERSION 7
```

若不是3.3.7则需要手动安装。  
首先搜索得到当前安装的Eigen库的位置  

```shell
sudo updatedb
locate eigen3
```

得到当前Eigen库路径后，删除所有Eigen库文件  

```shell
sudo rm -rf /usr/include/eigen3 /usr/lib/cmake/eigen3 /usr/share/doc/libeigen3-dev /usr/share/pkgconfig/eigen3.pc /var/lib/dpkg/info/libeigen3-dev.list /var/lib/dpkg/info/libeigen3-dev.md5sums
```

然后到[网站](https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz)下载Eigen3.3.7的安装包  
解压安装包  

```shell
sudo tar -xzvf eigen-3.3.7.tar.gz 
```

然后编译安装  

```shell
cd eigen-3.3.7
mkdir build
cd build
sudo cmake ..
sudo make install
```

安装完成后，头文件安装在/usr/local/include/eigen3/  
但是由于很多程序中include时经常使用#include <Eigen/Dense>，故需要复制头文件文件夹到include目录下，否则有可能系统无法默认搜索到，build时会找不到。  

```shell
sudo cp -r /usr/local/include/eigen3/Eigen /usr/local/include 
```

**注：以上1.7步骤的内容参考了[CSDN博客](https://blog.csdn.net/reasonyuanrobot/article/details/114372363)并经过我自己的验证可以完成安装配置。**

## 2. ROS 环境配置

### 2.1 安装ROS Melodic

使用鱼香ROS一键安装  

```shell
wget http://fishros.com/install -O fishros && . fishros
```

选择1一键安装ROS，选择更换系统源，选择版本为Melodic(桌面版)

### 2.2 安装Autoware编译系统依赖项  

```shell
sudo apt update
sudo apt install python3-pip
sudo apt install -y python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin
sudo apt install -y python3-pip python3-colcon-common-extensions python3-setuptools python3-vcstool
pip3 install -U setuptools
```

### 2.3 安装rosdepc

由于众所周知的原因，rosdep的源可能无法访问，所以我使用rosdepc替代rosdep  
使用鱼香ROS一键安装rosdepc  

```shell
wget http://fishros.com/install -O fishros && . fishros
```

选择3一键安装rosdepc，然后

```shell
rosdepc update
```

## 3. 工作空间创建与编译安装

### 3.1 创建工作空间

```shell
mkdir -p road-sweeper.ws
cd road-sweeper.ws
```

### 3.2 克隆源代码

```shell
mkdir src
cd src
git clone -b source https://github.com/wang-ruifan/Road-sweeper.git
```

### 3.3 安装依赖项

```shell
cd ..
rosdepc install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```
rosdep安装依赖项时，会报错找不到系统依赖项，需要手动安装。如下面的报错

```shell
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
road_occupancy_processor: Cannot locate rosdep definition for [grid_map_ros]
xsens_driver: Cannot locate rosdep definition for [gps_common]
nmea_navsat: Cannot locate rosdep definition for [nmea_navsat_driver]
autowarecmd_to_can: Cannot locate rosdep definition for [socketcan_bridge]
```

则需要手动安装这些依赖项

```shell
sudo apt-get install ros-melodic-grid-map-ros
sudo apt-get install ros-melodic-gps-common
sudo apt-get install ros-melodic-nmea-navsat-driver
sudo apt-get install ros-melodic-socketcan-bridge
```

### 3.4 编译安装

若正常安装了CUDA，cuDNN，TensorRT，则为

```shell
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

若没有安装CUDA，则为

```shell
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

选择特定包编译时，为

```shell
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select xxx
```

# Road-sweeper based on Jetson and Autoware Installation Guide

## Jetson Environment

![Jetson Environment](https://github.com/wang-ruifan/Road-sweeper/tree/docs/images/jetson-environment.png)  

- Note: The OpenCV version must not be 4.x installed via JetPack. It must be OpenCV 3.2 that comes with ROS Melodic. Otherwise, there will be many cv library errors during compilation. You need to uninstall the OpenCV installed by JetPack or not install JetPack at all, and manually install CUDA, cuDnn, and TensorRT.

## 1. Jetson Environment Setup

### 1.1. Flash Jetson with Ubuntu 18.04

Refer to [CSDN Blog](https://blog.csdn.net/weixin_47606814/article/details/127841948?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522568955dfc76993359b335c02fcf6e43f%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=568955dfc76993359b335c02fcf6e43f&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-127841948-null-null.142^v100^pc_search_result_base8&utm_term=jetson%20xavier%20nx%E7%83%A7%E5%BD%95%E7%B3%BB%E7%BB%9F&spm=1018.2226.3001.4187)  
Note: Choose JetPack version 4.x.x for Ubuntu 18.04, and only install Jetson OS, setting the installation location to NVMe.

### 1.2. Install jtop

```shell
sudo apt-get install python3-pip
pip3 install jetson-stats
```

Then, type `jtop` in the terminal to view CPU, GPU, and other information.  

### 1.3. Install CUDA

```shell
sudo apt-get install cuda-toolkit-10-2
```

After installation, open the .bashrc file  

```shell
sudo gedit ~/.bashrc
```

Add the following at the end of the file  

```shell
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-10.2/lib64
export PATH=$PATH:/usr/local/cuda-10.2/bin
export CUDA_HOME=$CUDA_HOME:/usr/local/cuda-10.2
```

### 1.4. Install cuDNN

First, check the available versions of cuDNN in the repository

```shell  
sudo apt-cache policy libcudnn8
```

For my Jetson with JetPack 4.6.1 and CUDA 10.2, only one version of libcudnn8 is available, so it can be installed directly  

```shell
sudo apt-get install libcudnn8
```

If you need to choose a version

```shell
sudo apt-get install libcudnn8=(*.*.*.**+cuda***)
```

### 1.5 Verify CUDA and cuDNN Installation

Use jtop to check the CUDA and cuDNN versions in the info section. If they are correct, the installation is successful.

```shell
jtop
```

You can also run the following Python code to verify

```python
import torch
print(torch.__version__)
print('CUDA available: ' + str(torch.cuda.is_available()))
print('cuDNN version: ' + str(torch.backends.cudnn.version()))
a = torch.cuda.FloatTensor(2).zero_()
print('Tensor a = ' + str(a))
b = torch.randn(2).cuda()
print('Tensor b = ' + str(b))
c = a + b
print('Tensor c = ' + str(c))
```

**Note: The content of steps 1.2 to 1.5 is referenced from [CSDN Blog](https://blog.csdn.net/a111222zw/article/details/120632906) and has been verified by me to complete the installation and configuration.**

### 1.6 Install TensorRT

First, check the available versions of TensorRT in the repository

```shell
sudo apt-cache policy tensorrt
```

**!!!Note!!!: The TensorRT version must be 7.x.x, otherwise there will be errors when compiling Autoware.**  
Choose version 7.x.x and install  

```shell
sudo apt-get install tensorrt=(7.*.*.**+cuda10-2)
```

**Note: The content of step 1.6 is referenced from [CSDN Blog](https://blog.csdn.net/weixin_43541510/article/details/130796360) and has been verified by me to complete the installation and configuration.**

### 1.7 Install Eigen 3.3.7

The system may automatically install Eigen after installation, but the version may not be 3.3.7, so you need to manually install version 3.3.7.  
First, check the installed version  

```shell
gedit /usr/include/eigen3/Eigen/src/Core/util/Macros.h
```

or  

```shell
gedit /usr/local/include/eigen3/Eigen/src/Core/util/Macros.h
```

Check the version number. If the content is as follows, version 3.3.7 is installed

```h
#define EIGEN_WORLD_VERSION 3
#define EIGEN_MAJOR_VERSION 3
#define EIGEN_MINOR_VERSION 7
```

If it is not version 3.3.7, you need to manually install it.  
First, search for the current location of the installed Eigen library  

```shell
sudo updatedb
locate eigen3
```

After getting the current Eigen library path, delete all Eigen library files  

```shell
sudo rm -rf /usr/include/eigen3 /usr/lib/cmake/eigen3 /usr/share/doc/libeigen3-dev /usr/share/pkgconfig/eigen3.pc /var/lib/dpkg/info/libeigen3-dev.list /var/lib/dpkg/info/libeigen3-dev.md5sums
```

Then download the Eigen 3.3.7 installation package from the [website](https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz)  
Extract the installation package  

```shell
sudo tar -xzvf eigen-3.3.7.tar.gz 
```

Then compile and install  

```shell
cd eigen-3.3.7
mkdir build
cd build
sudo cmake ..
sudo make install
```

After installation, the header files are installed in /usr/local/include/eigen3/  
However, since many programs often use `#include <Eigen/Dense>` when including, you need to copy the header file folder to the include directory, otherwise the system may not find it by default during the build.

```shell
sudo cp -r /usr/local/include/eigen3/Eigen /usr/local/include 
```

**Note: The content of step 1.7 is referenced from [CSDN Blog](https://blog.csdn.net/reasonyuanrobot/article/details/114372363) and has been verified by me to complete the installation and configuration.**

## 2. ROS Environment Setup

### 2.1 Install ROS Melodic

Use Fishros ROS one-click installation  

```shell
wget http://fishros.com/install -O fishros && . fishros
```

Choose option 1 for one-click ROS installation, change the system source, and select the Melodic (desktop) version.

### 2.2 Install Autoware Build System Dependencies  

```shell
sudo apt update
sudo apt install python3-pip
sudo apt install -y python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin
sudo apt install -y python3-pip python3-colcon-common-extensions python3-setuptools python3-vcstool
pip3 install -U setuptools
```

### 2.3 Install rosdepc

Due to well-known reasons, the rosdep source may not be accessible, so I use rosdepc as a replacement for rosdep  
Use Fishros ROS one-click installation to install rosdepc  

```shell
wget http://fishros.com/install -O fishros && . fishros
```

Choose option 3 for one-click rosdepc installation, then

```shell
rosdepc update
```

## 3. Workspace Creation and Build Installation

### 3.1 Create Workspace

```shell
mkdir -p road-sweeper.ws
cd road-sweeper.ws
```

### 3.2 Clone Source Code

```shell
mkdir src
cd src
git clone -b source https://github.com/wang-ruifan/Road-sweeper.git
```

### 3.3 Install Dependencies

```shell
cd ..
rosdepc install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

### 3.4 Build Installation

If CUDA, cuDNN, and TensorRT are installed correctly, use

```shell
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

If CUDA is not installed, use

```shell
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

To build specific packages, use

```shell
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select xxx
```
