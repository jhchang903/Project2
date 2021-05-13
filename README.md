# Project2
Design a robot chasing a white ball and simulate in ROS.


## Clone directory
```
git clone https://github.com/jhchang903/Project2.git
```


## Create a catkin workspace and initialize it
```
mkdir catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
```
and then **move the two directories (_ball_chaser_ and _my_robot_) from Project2 to catkin_ws/src**


## Compile the packages
```
cd catkin_ws
catkin_make
```

## Launch my_robot
```
cd catkin_ws
source devel/setup.bash
roslauch my_robot world.launch
```

##  Launch ball_chaser 
**Open a new terminal**
```
cd catkin_ws
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch
```
