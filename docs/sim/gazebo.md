# Gazebo

---

## Overview

Gazebo是一款优秀的开源物理仿真环境，它具备如下特点：

* 动力学仿真：支持多种高性能的物理引擎，例如ODE、Bullet、SimBody、DART等。

* 三维可视化环境：支持显示逼真的三维环境，包括光线、纹理、影子。

* 传感器仿真：支持传感器数据的仿真，同时可以仿真传感器噪声。

* 可扩展插件：用户可以定制化开发插件，扩展gazebo的功能，满足个性化的需求。

* 多种机器人模型：官方提供PR2、Pioneer2 DX、TurtleBot等机器人模型，当然也可以使用自己创建的机器人模型。

* TCP/IP传输：gazebo可以实现远程仿真，后台仿真和前台显示通过网络通信。

* 云仿真：gazebo仿真可以在Amazon、Softlayer等云端运行，也可以在自己搭建的云服务器上运行。

* 终端工具：用户可以使用gazebo提供的命令行工具在终端实现仿真控制。


## Config

```yaml title="~/.ignition/fuel/config.yaml"
servers:
  -
    name: osrf
    url: https://fuel.ignitionrobotics.org
```

## Files

```mermaid
graph LR;
    A(RViz)   --> C(.launch);
    B(Gazebo) --> C;
    C --> D(Model);
    C --> E(.world);
    D --> F(.urdf);
    D --> K(.sdf);
    D --> G(.xacro);
    F --> H(Mesh);
    G --> I(.gazebo);
    H --> J(.dae);
```

* mesh files: .obj, .stl, .dae
* urdf files: .sdf, .urdf, .xacro
* world files: .world

URDF vs SDF vs XACRO

- sdf、urdf、xacro 都是模型文件
- sdf、urdf、xacro 都可以加载dae等三维模型文件
- sdf、urdf、xacro 之间还不能自动转换，只能根据实体手动转换

- sdf、urdf、xacro的区别是：
    - ros和gazebob 不能 使得sdf模型动起来，sdf模型有自己的行为
    - ros和gazebob 能 使得urdf、xacro模型动起来，urdf、xacro模型有自己的行为

- xacro文件是urdf文件的改进版，urdf文件只能在rviz等中显示，不能在仿真器中显示出来。
- xacro文件可以在gazebo仿真器中显示出来，相对urdf文件，xacro文件增加了更多的属性设置标签。

### URDF

```sh
sudo apt install liburdfdom-tools
```

```sh
check_urdf xxx.urdf

urdf_to_graphiz xxx.urdf
```

### load model

```sh
export GAZEBO_MODEL_PATH=/home/pan/.gazebo/models

rosrun gazebo_ros spawn_model -sdf -model model.sdf

rosrun gazebo_ros spawn_model \
  -file `echo $GAZEBO_MODEL_PATH`/xxx/model.sdf -sdf -model modelname

# online model
rosrun gazebo_ros spawn_model \
  -database coke_can -sdf -model coke_can1 -y 2.2 -x -0.3
```

### model state

```sh
rostopic pub -r 20 /gazebo/set_model_state gazebo_msgs/ModelState \
  '{model_name: coke_can1, \
  pose: { position: { x: 1, y: 0, z: 2 }, \
  orientation: {x: 0, y: 0.491983115673, z: 0, w: 0.870604813099 } }, 
  twist: { linear: { x: 0, y: 0, z: 0 }, \
  angular: { x: 0, y: 0, z: 0}  }, \
  reference_frame: world }'

rostopic echo -n 1 /gazebo/model_states
rostopic echo -n 1 /gazebo/link_states
```

### delete model

```sh
rosservice call gazebo/delete_model '{model_name: xxx}'
```


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
