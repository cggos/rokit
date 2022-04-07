# Arduino

* [Arduino](https://www.arduino.cc/)
* [Arduino中文社区](https://www.arduino.cn/)

-----

# Arduino IDE Install

```sh
wget https://downloads.arduino.cc/arduino-1.6.9-linux64.tar.xz
tar xvf arduino-1.6.9-linux64.tar.xz
sudo ./installl.sh
```

## Permission Problem

```sh
sudo usermod -aG dialout $USER
sudo chmod a+rw /dev/ttyACM0
```

# Arduino and Sensors

## Ultrasonic Sensor

![](./imgs/Ultrasonic-Mega2560.png)

* [Ultrasonic Sensor HC-SR04 and Arduino Tutorial](https://howtomechatronics.com/tutorials/arduino/ultrasonic-sensor-hc-sr04/)  

* [HOW TO CONTROL A SIMPLE PROCESSING GAME WITH ARDUINO](http://www.instructables.com/id/How-to-control-a-simple-Processing-game-with-Ardui/)

### ROS程序读取HC-SR04数据

1.按上图接线

2.烧录 **ultrasonic_ros.ino** 程序到 Arduino

3.分别启动如下脚本

```sh
roscore
rosrun rosserial_python serial_node.py /dev/ttyACM0
rostopic echo /range_ultrasonic
```

