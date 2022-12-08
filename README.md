# ICRA 2023 quadruped competition simulation map

- Version 1.0.0

---

<div style="display:flex;">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

**Package summary**

A repository for quadruped robot competition. Used for build, test, and deployment.

- Maintainer status: maintained
- Maintainers
  - Jeongmin Jeon (nicky707@g.skku.edu)
  - Jinjae Shin (jinjae92@g.skku.edu)
  - Hyungpil Moon (hyungpil@g.skku.edu)
- Author
  - Jeongmin Jeon (nicky707@skku.edu)


</div>
<div style="flex:40%; padding-left:10px;">

**Table of Contents**
- [Overview](#overview)
- [Installation methods](#installation-methods)
    - [1. ROS](#1-ros)
    - [2. Dependencies](#2-dependencies)
    - [3. Gazebo](#3-gazebo)
    - [4. Tutorial](#4-tutorial)

</div>
</div>

---

## Overview

[<img src="https://www.ros.org/imgs/logo-white.png" width="250"/>](http://www.ros.org/)

<img src="doc/gazebo_map.png" width=""/>

- A repository for quadruped robot competition. Used for build, test, and deployment.
- This repository provides a ROS urdf package of robot competition map environment for simulations.

---

## Installation methods

ROS must be installed.

#### 1. ROS

Tested on ros-melodic and ros-noetic versions. [ROS Install](http://wiki.ros.org/melodic/Installation/Ubuntu)

#### 2. Dependencies

```bash
sudo apt-get install ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-urdf ros-$ROS_DISTRO-urdf-tutorial
sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher
```

#### 3. Gazebo

```bash
sudo apt-get install ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-gazebo-ros-control
```


#### 4. Tutorial
clone the URDF package 
```bash
cd $ROS_WORKSPACE
mkdir src
cd src
git clone https://github.com/rise-lab-skku/ICRA2023_Quadruped_Competition
```

build & install
```bash
cd ..
mkdir build
catkin_make
```

visualization with Rviz
```bash
roslaunch ICRA2023_Quadruped_Competition rviz.launch 
```

import urdf environment from gazebo
```bash
roslaunch ICRA2023_Quadruped_Competition competition_world.launch 
```


In this version, we only provide a method to import the urdf environment from gazebo simulation. If you want to use another simulator, import `urdf/urdf.map` directly