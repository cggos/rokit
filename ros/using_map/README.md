
## Draw Map

```sh
roslaunch using_map draw_map.launch
```

## Read Map

```sh
roslaunch turtlebot_stage turtlebot_in_stage.launch

rosrun using_map sub_map.py
rosrun using_map read_map
```

## Local Path Planning

```sh
roslaunch turtlebot_stage turtlebot_in_stage.launch

rosrun using_map read_map_turtlebot_stage
```