# Wheelchair ROS Simulation
Wheelchair simulation in ROS using Gazebo for a research project.

## Getting Started
Clone the packages using `git clone https://github.com/agil98/Wheelchair-Simulation` and move to your `catkin_ws/src` folder.

### Prerequisites

The packages are compatible with [ROS Melodic](https://wiki.ros.org/melodic/Installation). To display the model, the latest [Gazebo](http://gazebosim.org/tutorials?cat=install) and [Rviz](http://wiki.ros.org/rviz) versions are needed. To download the softwares, please visit the official ROS website.

If you downloaded a full installation of ROS, the following packages will be already present.
```
sudo pip3 install rospkg catkin_pkg
sudo apt-get install python-catkin-tools python3-dev
```
and
```
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
```

The library joint-state-publisher-gui might be missing from the installation, it can easily be downloaded using 
```
sudo apt install ros-melodic-joint-state-publisher-gui
```

## Running the Simulation

Run the `display.launch` file inside the wheelchair package to open the simulation in Gazebo. The file can be changed to also display the model in Rviz.
```
roslaunch wheelchair display.launch
```
To drive the model inside Gazebo, you can publish commands to `cmd_vel`, for instance `rostopic pub -1 cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 0.5}}'` or connect it t o a keyboard driver, i.e. `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`.

If you want to use a GUI to publish commands, running the `diffdriver.launch` file launches the rqt robot steering wheel package.

If you encounter any errors, try the following command in the top level folder inside your 	catkin workspace.
```
source devel/setup.bash
```
