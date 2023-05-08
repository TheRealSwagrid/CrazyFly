# Ubuntu

   ```bash
   lsb_release -a
   ```

   ```
   Distributor ID: Ubuntu
   Description: Ubuntu 20.04.5 LTS
   Release: 20.04
   Codename: focal
   ``` 

# Packages to install

   ```bash
   sudo apt update
   ```

### Git

   ```bash
   sudo apt-get install git
   ```

### Ros Noetic (ubuntu20)<br/>

http://wiki.ros.org/noetic/Installation/Ubuntu (1.1-1.5)

### Minizinc

   ```bash
   sudo apt-get install minizinc
   ```

   ```bash
   minizinc --version
   ```

   ```bash
   MiniZinc to FlatZinc converter, version 2.4.2
   Copyright (C) 2014-2020 Monash University, NICTA, Data61
   ```

### Crazyswarm (Crazyflies)

Steps from: https://crazyswarm.readthedocs.io/en/latest/installation.html

1. Ros noetic installed and source ros
    1. Setup sources.list
       ```bash
       sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
       ```
    2. Setup keys
       ```bash
       sudo apt install curl # if you haven't already installed curl
       curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
       ```
    3. Installation
      ```bash
      sudo apt update
      ```
      ```bash
      sudo apt install ros-noetic-desktop-full
      ```
      ```bash
      source /opt/ros/noetic/setup.bash
      ```

2. Install packages
   ```bash
   export CSW_PYTHON=python3
   sudo apt install -y ros-noetic-tf ros-noetic-tf-conversions ros-noetic-joy
   sudo apt install -y libpcl-dev libusb-1.0-0-dev
   sudo apt install -y swig lib${CSW_PYTHON}-dev ${CSW_PYTHON}-pip
   ${CSW_PYTHON} -m pip install pytest numpy PyYAML scipy
   ```
3. Clone Crazyswarm Repo<br/>
   Check current terminal path for git clone!
   ```bash
   git clone https://github.com/USC-ACTLab/crazyswarm.git
   cd crazyswarm
   source /opt/ros/noetic/setup.bash
   ./build.sh
   ```
4. Run tests
   ```bash
   cd ros_ws/src/crazyswarm/scripts
   source ../../../devel/setup.bash
   $CSW_PYTHON -m pytest
   ```   
   Output: 30 passed, 1 skipped
5. Set permissions
   ```bash
   cd crazyswarm 
   bash pc_permissions.sh 
   ```

### VRPN Client for Vicon (needed!)

   ```bash
   sudo apt-get install ros-noetic-vrpn-client-ros
   ```

## Build and run

**Falls notwendig**</br>
`chmod +x crazyswarm/ros_ws/devel/setup.bash`

```bash
 source path/to/crazyswarm/ros_ws/devel/setup.bash
 cd combo-implementation/ros1/ros_ws
 catkin_make
```

Add following to `combo-implementation/ros1/ros_ws/devel/setup.bash`

   ```bash
   nano combo-implementation/ros1/ros_ws/devel/setup.bash
   export PYTHONPATH=$PYTHONPATH:path/to/crazyswarm/ros_ws/src/crazyswarm/scripts
   ```

for example: `export PYTHONPATH=$PYTHONPATH:~/crazyswarm/ros_ws/src/crazyswarm/scripts`
for example: `export PYTHONPATH=$PYTHONPATH:~/Documents/combo/crazyswarm/ros_ws/src/crazyswarm/scripts`

```bash
 cd combo-implementation/ros1/ros_ws
 source devel/setup.bash
 roslaunch isse_crazy demo.launch
```  

# General

1. source noetic => catkin_make crazyswarm
2. source crazyswarm => catkin_make ros1/ros_ws
3. source ros1/ros_ws/devel/setup.bash
4. export python path 
