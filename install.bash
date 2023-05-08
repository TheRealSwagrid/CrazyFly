COMBO_DIR=$(realpath "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)/../")

# --- ROS ---
# setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# setup keys
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# installation
sudo apt update
sudo apt install ros-noetic-desktop-full

# setup environment
source /opt/ros/noetic/setup.bash

# --- CRAZYSWARM ---
# set environment variable of python interpreter
export CSW_PYTHON=python3

# install the dependencies
sudo apt install -y ros-noetic-tf ros-noetic-tf-conversions ros-noetic-joy
sudo apt install -y libpcl-dev libusb-1.0-0-dev
sudo apt install -y swig lib${CSW_PYTHON}-dev ${CSW_PYTHON}-pip
${CSW_PYTHON} -m pip install pytest numpy=="1.24.0" PyYAML scipy

# build
cd $COMBO_DIR/crazyswarm
./build.sh

# verify installation by running the unit tests
cd ros_ws/src/crazyswarm/scripts
source ../../../devel/setup.bash
$CSW_PYTHON -m pytest

# set permissions
cd $COMBO_DIR/crazyswarm
bash pc_permissions.sh

# vrpn client for vicon
sudo apt-get install ros-noetic-vrpn-client-ros

# --- BUILD ---
chmod +x $COMBO_DIR/crazyswarm/ros_ws/devel/setup.bash
source $COMBO_DIR/crazyswarm/ros_ws/devel/setup.bash
cd $COMBO_DIR/ros1/ros_ws
catkin_make

# since the following scripts need to be sourced in every bash terminal where ros should be used we add
# this to the .bashrc which automatically sources the scripts when a new shell is launched

STRING="export PYTHONPATH=\$PYTHONPATH:${COMBO_DIR}/crazyswarm/ros_ws/src/crazyswarm/scripts"
grep -qxF "${STRING}" $COMBO_DIR/ros1/ros_ws/devel/setup.bash || echo "${STRING}"  >> $COMBO_DIR/ros1/ros_ws/devel/setup.bash

STRING="source ${COMBO_DIR}/crazyswarm/ros_ws/devel/setup.bash"
grep -qxF "${STRING}"  ~/.bashrc || echo "${STRING}"  >> ~/.bashrc

STRING="source ${COMBO_DIR}/ros1/ros_ws/devel/setup.bash"
grep -qxF "${STRING}"  ~/.bashrc || echo "${STRING}"  >> ~/.bashrc
