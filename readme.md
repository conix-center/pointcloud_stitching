#Pointcloud Stitching for ARENA

##Overview
Scalable, multicamera distributed system for realtime pointcloud stitching in the ARENA (Augmented Reality Environment ~~in something~~ Area/Arena). This program is currently designed to use the **D400 Series Intel RealSense** depth cameras. Using the [Librealsense 2.0 SDK](https://github.com/IntelRealSense/librealsense), depth frames are grabbed and pointclouds are computed on the edge, before sending the raw XYZRGB values to a central computer over a TCP socket. The central program stitches the pointclouds together and displays it a viewer using [PCL](http://pointclouds.org/) libraries.

##Installation
Different steps of installation are required for installing the realsense camera servers versus the central computing system.
####Camera servers on the edge
- Go to [Librealsense Github](https://github.com/IntelRealSense/librealsense) and follow the instructions to install the Librealsense 2.0 SDK
- Install [Paho-MQTT](https://github.com/eclipse/paho.mqtt.cpp) for c++
- Install openssh-server to make it easier for access
`apt-get install openssh-server`
- Ensure that your cmake version is 3.1 or later. If not, download and install a newer version from the [CMake website](https://cmake.org/download/)
- Navigate to this *pointcloud_stitching* repository
`mkdir build && cd build`
`cmake ..`
`make && sudo make install`

####Central computing system
- Follow the instructions to download and install PCL libraries from their [website](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_posix.php).
- Install [Paho-MQTT](https://github.com/eclipse/paho.mqtt.cpp) for c++
- Install openssh-client to run commands on the edge computers from the central computer
`apt-get install openssh-client`
- Install python3 to run the camera-registration script
`sudo apt-get update`
`sudo apt-get install python3.6`
`sudo apt-get install python3-pip`
`python3 -m pip install --user scipy`
`python3 -m pip install --user numpy`
`python3 -m pip install --user pandas`
- Navigate to this *pointcloud_stitching* repository
`mkdir build && cd build`
`cmake .. -DBUILD_CLIENT=true`
`make && sudo make install`
