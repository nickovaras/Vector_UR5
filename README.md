# Vector + UR5 Python Node Example

An example of programatic control of ROS+MoveIt with a Python node. This node is an addition to the simulation of a Vector Mobile Robot + UR5 robot arm described here:

https://github.com/nickovaras/vector

## A note on SRDF
Unlike the c++ MoveGroup class, the Python MoveGroupCommander class seems to lack a method to directly read <group states> (named robot positions) saved on the robot's SRDF. Consider using a ROS package like srdfdom (http://wiki.ros.org/srdfdom) to parse the robots SRDF.


# Setup Instructions:

First, it is essential that you overlay the workspaces, so if you built the repository of the link above, start a new terminal and do:

```
cd ~/vector_ws
source devel/setup.bash
```

And ON THE SAME TERMINAL do:

```
mkdir ~/mobmanip_ws
cd ~/mobmanip_ws
mkdir src
cd src
catkin_init_workspace
git clone https://github.com/nickovaras/Vector_UR5.git
cd..
catkin_make
source devel/setup.bash
```
It is essential that you make the Python script executable. Go to the directory where the file is located and do: 
```
chmod +x mobile_manipulation.py
```

To use, after bringing all other components described in https://github.com/nickovaras/vector , start node with
```
rosrun mobile_manipulation mobile_manipulation.py

