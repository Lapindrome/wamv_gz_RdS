# WAM-V simulation using ROS 2 Jazzy + Gazebo Harmonic

**Master 2 Mecatronic - Promo 2024-2025**

**Robotic of Service - MOLLARD Yoan**

**Project members - TURPAULT Maxence & VILLEGER Arthur**

---

[Preview WAM-V](https://github.com/user-attachments/assets/71d9622f-a003-4d59-bfaf-0f32e410608c)

---

- [WAM-V simulation using ROS 2 Jazzy + Gazebo Harmonic](#wam-v-simulation-using-ros-2-jazzy--gazebo-harmonic)
  - [1. Introduction](#1-introduction)
  - [2. Installation](#2-installation)
    - [2.1. Download ROS 2 Jazzy](#21-download-ros-2-jazzy)
    - [2.2. Download Gazebo Harmonic](#22-download-gazebo-harmonic)
    - [2.3. Download ros\_gz](#23-download-ros_gz)
    - [2.4. Create a ROS 2 workspace and clone this repository into the `/src` directory](#24-create-a-ros-2-workspace-and-clone-this-repository-into-the-src-directory)
    - [2.5. Permissions](#25-permissions)
    - [2.6. Dependencies installation](#26-dependencies-installation)
  - [3. ROS 2 package detail](#3-ros-2-package-detail)
    - [3.1. Launchfiles](#31-launchfiles)
    - [3.2. Nodes](#32-nodes)
    - [3.3. Topics](#33-topics)
  - [4. Usage](#4-usage)
    - [4.1. Launch Gazebo simulation only](#41-launch-gazebo-simulation-only)
    - [4.2. Launch Gazebo simulation + PID node + YOLO node](#42-launch-gazebo-simulation--pid-node--yolo-node)
    - [4.3. Change Gazebo world](#43-change-gazebo-world)
    - [4.4. Launching individual nodes](#44-launching-individual-nodes)
      - [4.4.1 YOLO node](#441-yolo-node)
      - [4.4.2 PID node](#442-pid-node)
    - [4.5. Spawning a vessel](#45-spawning-a-vessel)
    - [ROS 2 utility usage](#ros-2-utility-usage)


---


## 1. Introduction

The wamv_gz_rds package provides a realistic simulation of the WAM-V (Wave Adaptive Modular Vessel) in a Gazebo environment, using ROS 2 Jazzy and Gazebo Harmonic.

Available features:
- WAM-V GOTO with PID control of the thrusters.
- Boat detection using the YOLO neural network.

![simu_gz_wamv](https://github.com/Lapindrome/wamv_gz_rds/blob/main/pictures/simu_gz_wamv.png)
![yolo_detection.png](https://github.com/Lapindrome/wamv_gz_rds/blob/main/pictures/yolo_detection.png)


---


## 2. Installation

### 2.1. Download [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)


### 2.2. Download [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/)


### 2.3. Download ros_gz
    
`sudo apt-get install ros-jazzy-ros-gz`


### 2.4. Create a ROS 2 workspace and clone this repository into the `/src` directory


### 2.5. Permissions

Give permissions to execute `/scripts` and `/src` from `wamv_gz_rds` package with :

```bash
chmod +x ./wamv_gz_rds/scipts/* ./wamv_gz_rds/src/*
```


### 2.6. Dependencies installation


If you use a Docker container, please run `startup.sh` script, using :

```bash
./wamv_gz_rds/scipts/startup.sh   # A virtual environment will be created at /home
```

Else :
```bash
source ~/.bashrc
sudo apt-get update
sudo apt-get upgrade

pip install ultralytics # Required to run YOLO detection

cd ..   # Position yourself in the ROS 2 workspace ros2_ws
colcon build
source install/setup.bash
```

For Usage, see section [4. Usage](#4-usage)

---


## 3. ROS 2 package detail

### 3.1. Launchfiles

Launchfile for WAM-V Gazebo sim :
   - [wamv_launch.py](https://github.com/Lapindrome/wamv_gz_rds/blob/main/launch/wamv_launch.py)

Launchfile for WAM-V Gazebo sim + PID node + YOLO node :
   - [wamv_launch_all_nodes.py](https://github.com/Lapindrome/wamv_gz_rds/blob/main/launch/wamv_launch_all_nodes.py)


### 3.2. Nodes

YOLO detection node `/camera_node` :
   - [wamv_camera_YOLO.py](https://github.com/Lapindrome/wamv_gz_rds/blob/main/src/wamv_camera_YOLO.py)

PID control node `/PID_node` :
   - [wamv_pid_control.py](https://github.com/Lapindrome/wamv_gz_rds/blob/main/src/wamv_pid_control.py)


### 3.3. Topics

- `/wamv/ground_truth/odometry`
- `/wamv/ground_truth/pose`
- `/wamv/sensors/camera/front/camera_info`
- `/wamv/sensors/camera/front/image`
- `/wamv/sensors/front_lidar/points`
- `/wamv/sensors/front_lidar/scan`
- `/wamv/sensors/gps/gps/fix`
- `/wamv/sensors/imu/imu/data`
- `/wamv/thrusters/left/thrust`
- `/wamv/thrusters/right/thrust`

---


## 4. Usage

Setup the ROS 2 environment, using :
```bash
cd ros2_ws

source ~/.bashrc

colcon build
source install/setup.bash

# source ~/venv/bin/activate    # If you use Docker, execute this line to activate the virtual environment
```



### 4.1. Launch Gazebo simulation only

```bash
ros2 launch wamv_gz_rds wamv_launch.py
```


### 4.2. Launch Gazebo simulation + PID node + YOLO node

```bash
ros2 launch wamv_gz_rds wamv_launch_all_nodes.py
```

When launched, Gazebo simulation and YOLO window will open

YOLO window only work when Gazebo simulation is started/resumed using world control


### 4.3. Change Gazebo world

```bash
ros2 launch wamv_gz_rds wamv_launch.py world:=src/wamv_gz_rds/worlds/wamv_world_modified.sdf
```

```bash
ros2 launch wamv_gz_rds wamv_launch_all_nodes.py world:=src/wamv_gz_rds/worlds/wamv_world_modified.sdf
```


### 4.4. Launching individual nodes

You may want to launch nodes individually, for exemple when testing either nodes alone with the Gazebo simulation

As so, use command to launch `wamv_launch.py`, and following commands in another terminal


#### 4.4.1 YOLO node

```bash
ros2 run wamv_gz_rds wamv_camera_YOLO.py
```


#### 4.4.2 PID node

```bash
ros2 run wamv_gz_rds wamv_pid_control.py
```


### 4.5. Spawning a vessel

You have access to a downloaded asset in `models/`, named vessel_E

When Gazebo is running, you can use the Resource Spawner to access it :

- Use upper right context menu, then search `Resource Spawner`
- You might have to minimize open tabs, as so, right click on title bar, then `Settings`, and check `Show title bar`
- In the `Resource Spawner`, select `Local Resources`, then click the `vessel_E`
- You can then add it to the scene


### ROS 2 utility usage

---

Get topic list with `ros2 topic list`

Get topic output with `ros2 topic echo <topic_name>`

Get topic informations with `ros2 topic info <topic_name>`

---

Get node list with `ros2 node list`

Get node informations with `ros2 node info <node_name>`

---
