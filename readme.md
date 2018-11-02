# Pointcloud Stitching for ARENA [![Build Status](https://travis-ci.com/ABalanuta/pointcloud_stitching.svg?branch=master)](https://travis-ci.com/ABalanuta/pointcloud_stitching)

## Overview
Scalable, multicamera distributed system for realtime pointcloud stitching in the ARENA (Augmented Reality Environment ~~in something~~ Area/Arena). This program is currently designed to use the **D400 Series Intel RealSense** depth cameras. Using the [Librealsense 2.0 SDK](https://github.com/IntelRealSense/librealsense), depth frames are grabbed and pointclouds are computed on the edge, before sending the raw XYZRGB values to a central computer over a TCP socket. The central program stitches the pointclouds together and displays it a viewer using [PCL](http://pointclouds.org/) libraries.

This system has been designed in mind to allow 10-20 cameras to be connected simultaneously. Currently, our set up involves each RealSense depth camera connected to its own Intel i7 NUC computer. Each NUC is connected to a local network via ethernet, as well as the central computer that will be doing the bulk of the computing. Our current camera calibration method is to use a Theodolite, a survey precision instrument, to obtain real world coordinates of each camera (this will be updated soon I hope). 

## Installation
Different steps of installation are required for installing the realsense camera servers versus the central computing system.
#### Camera servers on the edge
- Go to [Librealsense Github](https://github.com/IntelRealSense/librealsense) and follow the instructions to install the Librealsense 2.0 SDK
- Install [Paho-MQTT](https://github.com/eclipse/paho.mqtt.cpp) for c++
- Install openssh-server to make it easier for access<br />
`apt-get install openssh-server`
- Ensure that your cmake version is 3.1 or later. If not, download and install a newer version from the [CMake website](https://cmake.org/download/)
- Navigate to this *pointcloud_stitching* repository<br />
`mkdir build && cd build`<br />
`cmake ..`<br />
`make && sudo make install`

#### Central computing system
- Follow the instructions to download and install PCL libraries from their [website](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_posix.php).
- Install [Paho-MQTT](https://github.com/eclipse/paho.mqtt.cpp) for c++
- Install openssh-client to run commands on the edge computers from the central computer<br />
`apt-get install openssh-client`
- Install python3 and other packages (Needed to run the camera-registration script)<br />
`sudo apt-get update`<br />
`sudo apt-get install python3.6`<br />
`sudo apt-get install python3-pip`<br />
`python3 -m pip install --user scipy`<br />
`python3 -m pip install --user numpy`<br />
`python3 -m pip install --user pandas`
- Navigate to this *pointcloud_stitching* repository<br />
`mkdir build && cd build`<br />
`cmake .. -DBUILD_CLIENT=true`<br />
`make && sudo make install`

## Usage
Each realsense is connected to an Intel i7 NUC, which are all accessible through ssh from the ALAN (central) computer. To start running, go through each ssh connection and run pcs-camera-server. If the servers are setup correctly, each one should say "Waiting for client...". Then on the ALAN computer, run "pcs-multicamera-client -v" to begin the pointcloud stitching (-v for visualizing the pointcloud). For more available options, run "pcs-multicamera-client -h" for help and an explanation of each option.


# Docker

## Usage
Build image
```
sudo docker build . -t pcl
```

Run image
```
sudo docker run -it --rm pcl
```

`mkdir build && cd build`<br />
`cmake .. -DBUILD_CLIENT=true`<br />
`make && make install`
