# ROS

---

## Build

```sh
cd rokit
mkdir build
cd build
rosdep install --from-paths ../ros/ --ignore-src --rosdistro $ROS_DISTRO -y
catkin_make --source ../ros/
```

## Run

### robot1_description

```sh
roslaunch robot1_description state_xacro.launch model:="`rospack find robot1_description`/urdf/robot1.xacro"

# 模型并没有任何纹理渲染
roslaunch robot1_description gazebo.launch model:="`rospack find robot1_description`/urdf/robot1_base_01.xacro"

# 在 gazebo 中添加可见的纹理，创建并使用robot.gazebo文件
roslaunch robot1_description gazebo.launch model:="`rospack find robot1_description`/urdf/robot1_base_02.xacro"

# 为机器人添加 Hokuyo 激光雷达3D模型
# 在 robot.gazebo 文件里，我们将添加 libgazebo_ros_laser 插件，这样就可以模拟 Hokuyo 激光测距雷达的行为
roslaunch robot1_description gazebo.launch model:="`rospack find robot1_description`/urdf/robot1_base_03.xacro"

# 增加另一个传感器（一个摄像头）
roslaunch robot1_description gazebo.launch model:="`rospack find robot1_description`/urdf/robot1_base_04.xacro"

# 在 gazebo 中加载和使用地图，并移动机器人
roslaunch robot1_description gazebo_wg.launch
```
