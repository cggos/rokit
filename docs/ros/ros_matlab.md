# ROS with Matlab

---

## Overview

* [ROS Toolbox](https://ww2.mathworks.cn/help/ros/index.html): Design, simulate, and deploy ROS-based applications

* [ROS与Matlab入门教程](https://www.ncnynl.com/category/ros-matlab/)

<p align="center">
  <img src="https://ww2.mathworks.cn/help/ros/ros_proddesc_image.png" style="width:100%"/>
</p>

## ExampleHelperRobotSimulator

```matlab
% 启动ROS
rosinit

% 创建模拟器
robotsim = ExampleHelperRobotSimulator;
    
% 启用ROS接口
enableROSInterface(robotsim, true);
    
% 启用激光传感器
enableLaser(robotsim, true);
   
% 绘制机器人轨迹
showTrajectory(robotsim, true);
    
% 关闭模拟器
delete(robotsim)
    
% 关闭ROS
rosshutdown
```
