# Gazebo

---

## Files

<p align="center">
  <img src="../img/gazebo_model.png" style="width:50%"/>
</p>

* mesh files: .stl, .dae
* urdf files: .sdf, .urdf, .xacro
* world files: .world

### URDF SDF XACRO

- sdf文件、urdf文件和xacro文件都是模型文件。
- xacro文件是urdf文件的改进版，urdf文件只能在rviz等中显示，不能在仿真器中显示出来。
- xacro文件可以在gazebo仿真器中显示出来，相对urdf文件，xacro文件增加了更多的属性设置标签。
- sdf文件和urdf文件、xacro文件都可以加载dae等三维模型文件。
- sdf和urdf、xacro的区别是：
    - ros和gazebob 不能 使得sdf模型动起来、sdf模型有自己的行为
    - ros和gazebob 能 使得urdf、xacro模型动起来、urdf、xacro模型有自己的行为
- 当前，sdf文件和urdf、xacro文件之间还不能自动转换，只能根据实体手动转换。


## Gazebo with ROS

* [【ROS-Gazebo】SDF的建模与使用](https://zhuanlan.zhihu.com/p/129660662)

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
