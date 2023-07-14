# Virtual RobotX (VRX)
This repository is a fork of the gazeb_classic VRX repo. This repository will be used to develop software for our autonomous surface vessel.
The software should be ran in a docker container, and instructions for setting this up is listed below. The VRX team made these instructions originally, which are just modified for our specific fork.

## System setup
#### Install all prerequisites

First, we recommend upgrading the packages installed on your system:
```
sudo apt update
sudo apt full-upgrade
```

Install [ros noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) (desktop-full):

Install the dependencies:
```
sudo apt install -y build-essential cmake cppcheck curl git gnupg libeigen3-dev libgles2-mesa-dev lsb-release pkg-config protobuf-compiler qtbase5-dev python3-dbg python3-pip python3-venv ruby software-properties-common wget 
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
DIST=noetic
GAZ=gazebo11
sudo apt install ${GAZ} lib${GAZ}-dev ros-${DIST}-gazebo-plugins ros-${DIST}-gazebo-ros ros-${DIST}-hector-gazebo-plugins ros-${DIST}-joy ros-${DIST}-joy-teleop ros-${DIST}-key-teleop ros-${DIST}-robot-localization ros-${DIST}-robot-state-publisher ros-${DIST}-joint-state-publisher ros-${DIST}-rviz ros-${DIST}-ros-base ros-${DIST}-teleop-tools ros-${DIST}-teleop-twist-keyboard ros-${DIST}-velodyne-simulator ros-${DIST}-xacro ros-${DIST}-rqt ros-${DIST}-rqt-common-plugins
```
Create your workspace and get the code:

```
mkdir -p ~/vrx_ws/src
cd ~/vrx_ws/src
```
Clone the repository:
```
git clone https://github.com/NAVIER-USN/vrx_navier.git
```
Use rosdep to install additional catkin dependencies:
```
cd ~/vrx_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```



## Build and run the simulation:
Now that the environment is set up, either with docker or natively, we can build the workspace and start the simulation.
First, source the ROS setup.bash file:
```
source /opt/ros/noetic/setup.bash
```
Use catkin_make to build the software:
```
cd ~/vrx_ws
catkin_make
```
```
source  ~/vrx_ws/devel/setup.bash
```

### launch simulation and rviz
Launch the simulation and visualizations with the navier wamv configuration:
Start basic nodes and rviz:
```
roslaunch navier_bringup bringup.launch
```

### GNC (guidance, navigation, control)
To start the localization node, which publishes the odom -> base_link transform directly from imu and gps data:
```
roslaunch navier_bringup localization.launch
```
The navigation file starts the thrust allocation node and a control/guidance server (LOS and DP control):
```
roslaunch navier_bringup navigation.launch
```

### object detection
make sure you have the YOLO weights downloaded and placed as weights/weights.pt in the navier_object_detection directory.
```
roslaunch navier_bringup perception.launch
```






# from the VRX team:

## Reference

If you use the VRX simulation in your work, please cite our summary publication, [Toward Maritime Robotic Simulation in Gazebo](https://wiki.nps.edu/display/BB/Publications?preview=/1173263776/1173263778/PID6131719.pdf): 

```
@InProceedings{bingham19toward,
  Title                    = {Toward Maritime Robotic Simulation in Gazebo},
  Author                   = {Brian Bingham and Carlos Aguero and Michael McCarrin and Joseph Klamo and Joshua Malia and Kevin Allen and Tyler Lum and Marshall Rawson and Rumman Waqar},
  Booktitle                = {Proceedings of MTS/IEEE OCEANS Conference},
  Year                     = {2019},
  Address                  = {Seattle, WA},
  Month                    = {October}
}
```
