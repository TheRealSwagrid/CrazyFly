# Readme

## with real crazyflies

### Setup

**_!!! Consider weather you using zsh or bash => Therefor use .zsh or .bash endings of the scripts !!!_**

* install ros noetic (ubuntu20): http://wiki.ros.org/noetic/Installation/Ubuntu
* install crazyswarm: https://wiki.isse.de/index.php/Crazyflie2.1
* install https://wiki.isse.de/index.php/Crazyflie2.1 (ganz unten ist wichtig)

* export PYTHONPATH=$PYTHONPATH:~/Documents/combo/crazyswarm/ros_ws/src/crazyswarm/scripts/

### Build

```
 cd combo-implementation/ros1/ros_ws
 source path/to/crazyswarm/ros_ws/devel/setup.bash
 catkin_make
```

### Run

 ```
cd combo-implementation/ros1/ros_ws
 export PYTHONPATH=$PYTHONPATH:~/Documents/combo/crazyswarm/ros_ws/src/crazyswarm/scripts/
 source devel/setup.bash
 roslaunch isse_crazy java_crazyflie_service.launch
```

## just a ros java test thingy

### Setup

* install ros noetic (ubuntu20): http://wiki.ros.org/noetic/Installation/Ubuntu

### Build

```
 source /opt/ros/noetic/setup.bash
 cd combo-implementation/ros1/ros_ws
 catkin_make
```

### Run

```
 cd combo-implementation/ros1/ros_ws
 source devel/setup.bash
 roslaunch isse_crazy java_sample.launch
```

## tips for ros development

* do not forget to make new python scripts executable:
  ` chmod +x python_node.py`
