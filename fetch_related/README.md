## 1. Running
Please refer to Sections 2,3,4 for installation guidleines

> **1.1**  Launch house environment in the gazebo
```
roslaunch fetch_gazebo housee.launch
```
Tuck the arm during navigation
```
cd ./src/fetch_gazebo/fetch_gazebo/scripts
python tuck_arm.py
```
> **1.2**  Launch move base for navigation functionalities and mapping module.

  If you wish to include depth image alongside LaserScan for mapping, 
  ```
  roslaunch fetch_gazebo mapping.launch
  ```
  If you wish to map using LaserScan data alone,
  ```
  roslaunch fetch_gazebo mapping2.launch
  ```
> **1.3** Navigate the robot using keyboard
  ```
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py
  ```
> **1.4** To run exploration
```
roslaunch explore_lite explore_n_save.launch
```
> **1.5** Save the data
The following scripts saves rgb, depth, odometry, camera pose, occupancy map
```
cd ./src/fetch_gazebo/fetch_gazebo/scripts
python save_data.py <time_interval>
```
> **1.6** Save the final map
The following script saves the map along with the .yaml file containing map parameters
```
cd ./src/fetch_gazebo/fetch_gazebo/scripts
savemap.sh <name>
```
> **1.7**
This scripts clears the data folders saved in the current folder.
```
cd ./src/fetch_gazebo/fetch_gazebo/scripts
clear_data.sh
```
> **1.8**
To send a point goal to the robot, run the following script
```
cd ./src/fetch_gazebo/fetch_gazebo/scripts
python navigate.py
```



## 2.  Install ROS
This code is tested on ros noetic version. Detailed installation instructions are found [here](http://wiki.ros.org/noetic/Installation/Ubuntu)
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install -y curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install -y ros-noetic-desktop-full

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

source ~/.bashrc
```

## 3. Install Gazebo
The compatible version for ros noetic is gazebo 11. Detailed installation instructions are found [here](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install#Defaultinstallation:one-liner)

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-get update

sudo apt-get install -y gazebo11
```
## 4. **Install Dependencies**

> ### General Dependencies
```
sudo apt install ros-noetic-tf2-sensor-msgs ros-noetic-gazebo-ros-pkgs python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-rosdep
```
```
pip install transforms3d empy==3.3.4 pyyaml defusedxml catkin_pkg rospkg lxml pyassimp==3.3
```


> ### Navigation Dependencies
```
sudo apt install ros-noetic-navigation ros-noetic-move-base ros-noetic-slam-gmapping ros-noetic-grid-map-costmap-2d ros-noetic-teleop-twist-joy ros-noetic-pointcloud-to-laserscan ros-noetic-rosbridge-server
```

> ### Control system Dependencies
```
sudo apt install ros-noetic-robot-controllers ros-noetic-gazebo-ros-control
``` 

> ### Perception Dependencies
```
sudo apt install ros-noetic-rgbd-launch
```

> ### Manipulation Dependencies
```
sudo apt install ros-noetic-moveit ros-noetic-trac-ik ros-noetic-moveit-python
```

> ## **Install Missing Dependencies**
```
sudo rosdep init && rosdep update && rosdep install --from-paths src --ignore-src -r -y
```

> ## **If you want to test the data visualisation**
```
pip install opencv-python numpy Pillow tk anytree
```

## 5. Compiling the workspace
```
cd fetch_ws
catkin_make
source devel/setup.bash
```
> If the build doesn't conisder python3 by default, build with the following command
```
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## 6. **Known Issues:**
AttributeError: module 'em' has no attribute 'RAW_OPT': [solution](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1519#issuecomment-2124599093)

