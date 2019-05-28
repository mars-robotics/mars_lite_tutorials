# mars_lite_tutorials

### Step 



### Step 

```bash
sudo apt-get install synaptic
sudo apt-get install -y chrony ntpdate build-essential
sudo ntpdate -q ntp.ubuntu.com
sudo apt-get install openssh-server
sudo apt-get install -y ros-kinetic-rqt-*
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-rviz-launchers ros-kinetic-turtlebot-teleop ros-kinetic-turtlebot-interactive-markers ros-kinetic-husky-desktop ros-kinetic-husky-simulator
```


### Step 

```bash
roslaunch turtlebot_gazebo turtlebot_world.launch
roslaunch turtlebot_teleop keyboard_teleop.launch
roslaunch husky_gazebo husky_playpen.launch
rosrun turtlebot_teleop turtlebot_teleop_key turtlebot_teleop/cmd_vel:=/cmd_vel
```
