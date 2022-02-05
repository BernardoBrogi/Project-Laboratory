# Collaborative task

## Quick start
After a successful compiling of the package, run the package with the following:
First, source the package
```
. ./your_catkin_ws/devel/setup.sh
```
Then, launch the ros launch file with the command:
```
roslaunch lwr_simple_example Stick.launch
```
This should create the gazebo simulation environment with a stick fixed to the arm. Then open another terminal to launch the client:
```
roslaunch lwr_simple_example client.launch
```
and another terminal for the console:
```
roslaunch lwr_simple_example console.launch
```
Run the following command in the console:
```
go_home
```
You should see the arm move, wait until it reach the target, then run the following command in the console:
```
Collaborate
```
Try to select the stick in gazebo and right click it to apply forces, at x=0  y= 1 z=0, apply a force around 50 - 100 N and keep pressing the enter key, you should see it move somehow accordingly.

## Current progress
It is implemented by sensing the force vector F with the force sensor between lwr_6_link and lwr_7_link(2 link with a fixed joint will be convert to one link, you still get the force vector because of fixed link). The program will get the current position p first and set the target position p*, with the relationship:
$p*=p+k\cdot F$
It is not perfectly moving exactly according to F vector but rotate a little at every time, this could because it is very offen that p* has been updated, even if it does not reach the target, It might can be solved by using PI control law. But in real human will eliminate this error automatically(human will adjust so the little error with the direction does not affect it)

##Structure of the project
roslaunch file Stick.launch organize the whole project.
In roslaunch file Stick.launch, path to the model, gazebo setup, controller to load and limit tto set has been defined there by values or path to some file
In folder "simple_actions" you can define new actions by yourself, Here is where your action class is defined, after the implementation, register your action in client_action_node.cpp and client_console_node.cpp.
The modification of the stick model is in kuka_lwr.urdf.xacro, by creating an additional link and fixed joint to the kuka_lwr
##Issues
###Gravity compensation
it is currently done externally, that is in the modeling by **set the mass of the stick very low**(but the Moment of inertia is the same so the dynamic should be same).
Object Compatibility Issues: In Collaborate the math are done by tf::Vector3 but the topic use geometry_msgs::Vector3 have not find a way to convert them.
Issue with reference frame: No way to get how the reference frame is defined(no document and not a good idea to do Reverse Engineering for a huge C++ project), therefore no idea about how should rotate the gravity vector.
## Failure attempt but worth to try again:
### sparse&joining the model: Currently in .urdf with the arm model
Sparse a (stick) model in .sdf file i.e. simple_environment.world and join it with the arm (cannot join two model)
Sparse a (stick) model in another .urdf file
Sparse a (stick) model in the arm's .urdf file but apart from the arm itself
Try to imitate a gripper: Try not to fix it add 2 DOF to the stick, failed with an additional virture link and two revolute joint in .urdf( do not know the frame of reference after). 
In .urdf and revolute2 inside <gazebo> label: nothing happened

### gazebo plugin
Apply force: can not set point of force
## Notes

