# mars_lite_tutorials


### Step 0

```bash
sudo apt-get install synaptic
sudo apt-get install -y chrony ntpdate build-essential
sudo ntpdate -q ntp.ubuntu.com
sudo apt-get install openssh-server
sudo apt-get install -y ros-kinetic-rqt-*
sudo apt-get install -y ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-rviz-launchers ros-kinetic-turtlebot-teleop ros-kinetic-turtlebot-interactive-markers ros-kinetic-husky-desktop ros-kinetic-husky-simulator ros-kinetic-fetch-gazebo ros-kinetic-fetch-gazebo-demo ros-kinetic-husky-navigation
```
### Step 1 安裝ROS

### Step 2 創建ROS工作空間
```bash
export CATKIN_WS=~/catkin_ws
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin_make
```
### Step 3 創建你的ROS Package
```bash
export CATKIN_WS=~/catkin_ws
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin_make
```
### Step 3.1 git clon ROS Package
```bash
cd $CATKIN_WS/src
git clone https://github.com/mars-robotics/mars_lite_tutorials
cd ..
```
### Step 3.2 複製 parts 裡的 Package 到 /srce，刪除 parts

```bash
catkin_make
```
### Step 4 Real Robot Mars-lite 使用鍵盤遙控 Mars-lite
```bash
roslaunch mars_lite_bringup mars_lite_bringup.launch
rosrun turtlebot_teleop turtlebot_teleop_key turtlebot_teleop/cmd_vel:=/mob_plat/cmd_vel
```


