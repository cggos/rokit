# ROS & Gazebo

* [ROS.org](http://www.ros.org/)
* [ROS Discourse](https://discourse.ros.org/)
* [ROS机器人俱乐部](http://www.rosclub.cn/)
* [eros](http://wiki.ros.org/eros)
* [ros2](https://index.ros.org/doc/ros2/)
* [ROS-Industrial](https://rosindustrial.org/)

------

[TOC]

## ROS Tutorials

* [the Construct](http://www.theconstructsim.com/): Learn to Develop for Robots with ROS
* [ros_tutorials (ROS Wiki)](http://wiki.ros.org/ros_tutorials)
* [ROS Tutorials (ClearPath)](http://www.clearpathrobotics.com/assets/guides/ros/index.html)
* [机器人操作系统入门](http://www.icourse163.org/course/ISCAS-1002580008)
* [古月居(CSDN)](https://blog.csdn.net/hcx25909)
* [ROS与navigation教程（创客智造）](https://www.ncnynl.com/category/ros-navigation/)


### Book & Code

* [Learning_ROS_for_Robotics_Programming_2nd_edition](https://github.com/AaronMR/Learning_ROS_for_Robotics_Programming_2nd_edition)

* [ROS by Example](http://wiki.ros.org/Books/ROSbyExample)
  - install rbx1
    ```sh
    git clone https://github.com/pirobot/rbx1.git
    git clone https://github.com/vanadiumlabs/arbotix_ros.git
    ```


## ROS Tips

### rosdep
* Install dependency of all packages in the workspace
  ```sh
  rosdep install --from-paths src --ignore-src -r -y
  rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
  ```

### rospack

```sh
rospack plugins --attrib=plugin costmap_2d
```

## Gazebo and ROS

* [【ROS-Gazebo】SDF的建模与使用](https://zhuanlan.zhihu.com/p/129660662)

### Files

<p align="center">
  <img src="./imgs/gazebo_model.png" style="width:50%"/>
</p>

* mesh files: .stl, .dae
* urdf files: .sdf, .urdf, .xacro
* world files: .world

#### URDF SDF XACRO

- sdf文件、urdf文件和xacro文件都是模型文件。
- xacro文件是urdf文件的改进版，urdf文件只能在rviz等中显示，不能在仿真器中显示出来。
- xacro文件可以在gazebo仿真器中显示出来，相对urdf文件，xacro文件增加了更多的属性设置标签。
- sdf文件和urdf文件、xacro文件都可以加载dae等三维模型文件。
- sdf和urdf、xacro的区别是：
    - ros和gazebob 不能 使得sdf模型动起来、sdf模型有自己的行为
    - ros和gazebob 能 使得urdf、xacro模型动起来、urdf、xacro模型有自己的行为
- 当前，sdf文件和urdf、xacro文件之间还不能自动转换，只能根据实体手动转换。

### gazebo_ros

```sh
roslaunch gazebo_ros empty_world.launch
roslaunch gazebo_ros willowgarage_world.launch
roslaunch gazebo_ros mud_world.launch
```

turtlebot3

```sh
roslaunch turtlebot3_gazebo turtlebot3_world.launch

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
# or
roslaunch teleop_twist_joy teleop.launch
```

### Simulations

* run gmapping with turtlebot on Gazebo
  ```sh
  roslaunch turtlebot_gazebo turtlebot_world.launch
  roslaunch turtlebot_gazebo gmapping_demo.launch
  roslaunch turtlebot_rviz_launchers view_navigation.launch

  roslaunch turtlebot_teleop keyboard_teleop.launch --screen
  ```

## Build Tools

* [Catkin](http://docs.ros.org/api/catkin/html/) is a collection of CMake macros and associated code used to build packages used in **ROS**

* catkin_make

* catkin_make_isolated

* catkin_tools
  - [catkin_tools](https://catkin-tools.readthedocs.io): command line tools for working with the catkin meta-buildsystem and catkin workspaces, these tools are separate from the Catkin CMake macros used in Catkin source packages.
  - [catkin_tools (1) - Linux Man Pages](https://www.systutorials.com/docs/linux/man/1-catkin_tools/)

* ament_tools

* [colcon](https://colcon.readthedocs.io/) is a command line tool to improve the workflow of building, testing and using multiple software packages. It automates the process, handles the ordering and sets up the environment to use the packages.

  ```sh
  sudo apt install python3-colcon-common-extensions

  colcon build 
  
  source install/setup.bash
  ```

* [Bloom](http://wiki.ros.org/bloom) is a **release automation tool**, designed to make generating platform specific release artifacts from source projects easier. Bloom is designed to work best with catkin projects, but can also accommodate other types of projects.


## ROS IDE

* CLion
  - [ROS Setup Tutorial](https://www.jetbrains.com/help/clion/ros-setup-tutorial.html)

* Qt Creator
  - [ROS Qt Creator Plug-in](https://ros-qtc-plugin.readthedocs.io)
