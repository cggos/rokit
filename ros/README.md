# ROS

---

## Build

```sh
cd rokit
mkdir build
cd build
rosdep install --from-paths ../ros/ --ignore-src --rosdistro $ROS_DISTRO -y
catkin_make --source ../build/
```
