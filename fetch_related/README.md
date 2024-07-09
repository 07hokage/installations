## 1. Running
Please refer to Sections 2,3,4 for installation guidleines

**1.1**  Launch house environment in the gazebo
```
roslaunch fetch_gazebo houseee.launch
```
**1.2**  Launch move base for navigation functionalities and mapping module.

  If you wish to include depth image alongside LaserScan for mapping, 
  ```
  roslaunch fetch_gazebo mapping.launch
  ```
  If you wish to map using LaserScan data alone,
  ```
  roslaunch fetch_gazebo mapping2.launch
  ```
**1.3** Navigate the robot using keyboard
  ```
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py
  ```




## 2.  Install ROS
This code is tested on ros noetic version. Detailed installation instructions are found [here](http://wiki.ros.org/noetic/Installation/Ubuntu)
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install ros-noetic-desktop-full

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

source ~/.bashrc
```

## 3. Install Gazebo
The compatible version for ros noetic is gazebo 11. Detailed installation instructions are found [here](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install#Defaultinstallation:one-liner)

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-get update

sudo apt-get install gazebo11
```
## 4. **Install Dependencies**

### General Dependencies
```
sudo apt install ros-noetic-tf2-sensor-msgs ros-noetic-gazebo-ros-pkgs python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-rosdep
```
```
pip install transforms3d empy pyyaml defusedxml catkin_pkg rospkg
```


### Navigation Dependencies
```
sudo apt install ros-noetic-navigation ros-noetic-move-base ros-noetic-slam-gmapping ros-noetic-grid-map-costmap-2d ros-noetic-teleop-twist-joy
```

### Control system Dependencies
```
sudo apt install ros-noetic-robot-controllers ros-noetic-gazebo-ros-control
``` 

### Perception Dependencies
```
sudo apt install ros-noetic-rgbd-launch
```

### Manipulation Dependencies
```
sudo apt install ros-noetic-moveit ros-noetic-trac-ik
```

## **Install Missing Dependencies**
```
sudo rosdep init && rosdep update && rosdep install --from-paths src --ignore-src -r -y
```

