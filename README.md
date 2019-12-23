# The University of Auckland 
## Faculty of Engineering, Department of Electrical, Computer Systems, and Software Engineering
### Undergraduate Summer Research Program. Orchard Robot Testing, 2018. 
### NOTE: THIS REPOSITORY IS NO LONGER MAINTAINED
#### Contributors
#### * Oliver Kim (@jkim838, jkim838@aucklanduni.ac.nz)
#### * James Flood (@floodski, jflo015@aucklanduni.ac.nz)
#### * Jiahui Wang (@jiahui747, jwan747@aucklanduni.ac.nz)

## Overview

### Introduction

The aim of this project is to develop a controller software for indoor prototype for automatic kiwifruit pollinating robot (a.k.a. orchard robot). The robot has three components - PC controller, Pioneer Robot and a laser panel to simulate pollinating fluid on a canopy of fake kiwifruits. 

The robot will maneuver under the canopy of kiwifruits and detects the exact location of targeted flowers using an Intel RealSense Camera and TensorFlow object detection API. Upon detection, the controller software will activate the corresponding laser panel on the target.

Enclosed in this document is an overview on development/testing environment and a complete user manual to using this software.

### Testing Environment

  * OS: Ubuntu 16.04 LTS
  * ROS: Kinetic
  * OpenCV: 3.3.1-dev (ROS Kinetic Install default)
  
  **Controller PC**
  * CPU: Intel(R) Core(TM) i7-6700HQ CPU @ 2.60 GHz
  * GPU: nVidia GTX1070 8GB GP104M (Mobile)
  * nVidia Graphics Driver Version: 418.67
  * CUDA Driver Version: 10.1
  * Memory: 16GB
  
  **Camera**
  * Intel RealSense D435i
  
  **Platform**
  * Pioneer P3-DX

## Dependencies

### 1. [ROS TensorFlow API](http://github.com/osrf/tensorflow_object_detector)

### 2. [Intel RealSense Camera Driver](https://github.com/IntelRealSense/librealsense)

### 3. [ROS Intel RealSense Wrapper](https://github.com/IntelRealSense/realsense-ros)

### 4. ROS Kinetic

### 5. [ROSAria](https://github.com/amor-ros-pkg/rosaria), [ARIACoda](https://github.com/reedhedges/AriaCoda)

## Installation

### 1. Setting Up ROS Kinetic

For more detail, refer to official installation guide [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)...

#### **1.1. Obtaining ROS**

On Ubuntu terminal,

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
#### **1.2. Downloading ROS**

Update softwares to current date with following command,
```
sudo apt-get update
```
Install ROS Kinetic with following command, 
```
sudo apt-get install ros-kinetic-desktop-full
```
#### **1.3. Initializing Rosdep**

To automatically install system dependencies for ROS, install Rosdep with following command,
```
sudo rosdep init
rosdep update
```
#### **1.4. Setup Environment**
Automatically add ROS dependencies upon Shell launch,
```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
#### **1.5. Install dependencies to building packages**
```
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### 2. Setting up ROS Environment

#### **2.1. Installing ROS TensorFlow Object Detection API**

On Ubuntu terminal, create *catkin_ws* folder with the following command,

```
cd ~
mkdir -p catkin catkin_ws/src
```
Install camera dependencies with the following command,

```
sudo apt-get install ros-kinetic-usb_cam ros-kinetic-openni2-launch
```
Clone standard vision message repository and object detection package to *catkin_ws/src* with the following command,

```
cd ~/catkin_ws/src
git clone https://github.com/Kukanani/vision_msgs.git
```

Copy the contents of *ROS Object Detect* in this repository to *~/catkin_ws/src*

Build *tensorflow_object_detector* and *vision_msgs* with the following command,

```
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
```

Once the build is complete, modify *usb_cam_detector.launch* file located at *~/catkin_ws/src/tensorflow_object_detector/launch*

from line

```
<remap from="image" to="/usb_cam_node/image_raw"/>
```
to

```
<remap from="image" to="/camera/color/image_raw"/>
```

Extract object detection model, and place them in *data/models* and the label file to *data/labels*

In *detect_ros.py* file, edit *MODEL_NAME* and *LABEL_NAME* to a corresponding file name. (By default, the entries are *ssd_mobilenet_v1_coco_11_06_2017* and *mscoco_label_map.pbtxt*)

#### **2.2. Setup RealSense Driver**

For additional information, refer to the official Intel RealSense driver [Installation Guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)

In a new instance of Ubuntu terminal, register the server public key with the following command, 

```
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
```
(If the server key cannot be retrieved, refer to the official guide)

Add the sever to the list of repositories with the following command,

```
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
```

Install the libraries and developer packages with the following command,
```
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```

Once the installation is complete, connect the RealSense camera and verify installation with the following command,

```
realsense-viewer
```

#### **2.3. Installing *librealsense* from the source code**

For additional information, refer to the official [guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

On Ubuntu terminal, update Ubuntu distribution and kernel with the following command,

```
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
```
Clone *librealsense* to *catkin_ws/src* with the following command,

```
cd ~/catkin_ws/src
git clone https://github.com/IntelRealSense/librealsense.git
```

Disconnect any RealSense camera, and install *librealsense* with the following command,

```
cd ~/catkin_ws/src/librealsense
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev
```

Run Intel RealSense permissions script with the following command,

```
./scripts/setup_udev_rules.sh
```

Build and apply patched kernel module with the following command,

```
./scripts/patch-realsense-ubuntu-lts.sh
```

Build *librealsense2* SDK with the following command,

```
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-5 g++-5
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5
sudo update-alternatives --set gcc "/usr/bin/gcc-5"
```

Navigate back to *librealsense* root directory and build SDK with the following command,

```
cd ~/catkin_ws/src/librealsense
mkdir build && cd build
cmake ../
```
Recompile and install *librealsense* binaries with the following command,

```
sudo make uninstall && make clean && make && sudo make install
```

### **3. Setup ROSAria**

For additional information, refer to [How to use ROSAria](http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA)

On Ubuntu terminal, navigate to *catkin_ws/src* and clone ROSAria code with the following command,

```
cd ~/catkin_ws/src
git clone https://github.com/amor-ros-pkg/rosaria.git
```
Download [ROSAria package](https://github.com/reedhedges/AriaCoda). Extract the content and *make* with the following command,

```
cd ~/$PATH_TO_DOWNLOADED_FILE/AriaCoda-master
make
```

Once *make* is complete, install ARIA with the following command,

```
sudo make install
```

After a successful install, there will be a message,

```
add "/usr/local/Aria/lib" to /etc/ld.so.conf or add it to the LD_LIBRARY_PATH environment variable
```

If so, navigate to *etc* and open *ld.so.conf* file with the following command,

```
cd /etc
sudo nano ld.so.conf
```

Once the file is open, on the bottom of the file, add

```
/usr/local/Aria/lib
```

Press **ctrl+o** to save and **ctrl+x** to exit.

Once ROSAria is installed, navigate back to *catkin_ws* and compile ROSAria with the following command,

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

To connect to the Pioneer Robot, open a new instance of Ubuntu terminal and type,

```
roscore
```

On the previous instance of Ubuntu terminal, launch ROSAria node with the following command,

```
rosrun rosaira rosaria
```

#### Troubleshooting ROSAria

For message *[FATAL]: ROSAria: ROS node setup failed...*, manually configure parameters to establish connection.

On Ubuntu terminal, try the following command,

```
sudo chmod 777 /dev/ttyUSB0
rosrun rosaria rosaria _port:=/dev/ttyUSB0
```

### **4. Laser Control**

Terminate any instance of *ROSCORE* with **ctrl+c**

On Ubuntu terminal, configure IP for Ethernet interface between ROS and the laser module with the following command,

```
sudo ifconfig enp0s25 192.168.21.64
```

#### **4.1. Troubleshooting Laser Control**

By default, laser module package is already included in this repository. If no executable is found, refer to following steps.

Download the contents of *laser_control* folder and paste it to *catkin_ws/src*

On Ubuntu terminal, navigate to *catkin_ws* folder and *make* with the following command,

```
cd catkin_ws
catkin_make
source devel/setup.bash
```

On a new instance of Ubuntu terminal, launch *ROSCORE* with the following command,

```
roscore
```

On the previous instance of Ubuntu terminal, create ROS package with the following command,

```
cd ~/catkin_ws/src
catkin_create_pkg laser_control std_msgs rospy roscpp
cd ..
catkin_make
~/devel/setup.bash
```

In *~catkin_ws/src/laser_control*, edit *CMakeLists.txt* at **line 121**

```
add_executable( shoot_laser_node src/shoot_laser.cpp)
target_link_libraries(shoot_laser_node ${catkin_LIBRARIES})
add_dependencies(shoot_laser_node ${catkin_EXPORTED_TARGETS})
```

Close and save the file. Perform *catkin_make* again to start building with the modified CMAKE file.


### **5. Launching the Platform**

Add robot maneuvering feature to the system by copyng the contents of *pioneer_robot* from the repository to *catkin_ws/src*

Add fire-control monitor feature (FCMonitor) to the system by copying the contents of *FCMonitor* from the repository to *~/catkin_ws/src*

On Ubuntu terminal, to launch the robot with all of its feature enabled, type

```
roscore
```
On a new instance of Ubuntu terminal, use the following command,

```
sudo ifconfig enp0s25 192.168.64.21
sudo chmod 777 /dev/ttyUSB0
cd ~/catkin_ws
catkin_make
source devel/setup.bash
cd ~/catkin_ws/src
roslaunch FCMonitor FC.launch
rosrun pioneer_robot pioneer_robot
```
