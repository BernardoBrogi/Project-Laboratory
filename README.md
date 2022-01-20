# Project Laboratory Human Centered Robotics

## Introduction


This package is an improved version of the [KUKA LWR EPFL package robot lasa](https://github.com/epfl-lasa/kuka-lwr-ros).
All the bugs contained in that package are solved, and we also added other funtionalities to the package.

The actual funtionalities we implemented in our package are the following:


* Move the robot with an Haptic Device
* Record the trajectory of the end effector
* Replay the trajectory
* Using DMPs to Learn from Demonstration and replay the shape of the trajectory recorded changing initial position and goal of the trajectory recorded 


## How to use (simulation)

This package runs perfectly in ROS melodic (full desktop version) and GAZEBO 9.

In order to launch the robot in simulation, run the following launch files in three different terminals:

```
$ roslaunch lwr_simple_example sim.launch
```

which will setup the simulation environment;

```
$ roslaunch lwr_simple_example client.launch
```
which will run the client;

```
$ roslaunch lwr_simple_example console.launch
```

This is the console from which we can command the robot.
All the functionalities of the EPFL package work.

For istance writing the command

```
$ Cmd> go_home
```

in the console will make the robot move in the home position.
It's always recommended to run this command at the beginning and after every other command in order to avoid singularities and problems related to the Cartesian controller.

### Move the robot with the Haptic Device and record the trajectory

#### Haptic Device setup

The haptic device we worked with is the Omega3. You may need to adjust the path to the libraries of the haptic device in the CMakelist.txt.
You may also need to give the permission to the port where the haptic device is inserted. To do this type in a terminal

```
$ ls -l /dev/bus/usb/00*
```
Which will give the list of devices connected to the port. Check which is the port connected to the haptic device.

Suppose this corresponds to your device
```
/dev/bus/usb/003
crw-rw-r-- 1 root root 189, 263  1월 10 15:42 008
```
Then type this command to give permisssion
```
sudo chmod o+w /dev/bus/usb/003/008
```
#### Implementation

Start the three files of the simulation and go in the position "go_home".

Now if you execute the command

```
$ Cmd> Record
```

you will be able for 10 seconds to control the robot with the haptic device and record the trajectory of the end effector. The data will be saved in the file data.txt.
The end effector positions are taken 100 times per second.

Now if you go_home again (you may need to execute the command twice) and execute the command

```
$ Cmd> Replay
```

the robot will replay the trajectory recorded.

### Exploit the trajectory recorded to LfD and execute another trajectory with the same shape

Supposing there is a trajectory recorded in the file data.txt, to plan and execute a new trajectory run in another terminal

```
$ roslaunch dmp dmp.launch
```
 
In the file LfD.py you can set the initial position of the trajectory (default is the initial position of go_home) and the goal position.

Running in another terminal

```
$ rosrun dmp LfD.py
```

will create a new trajectory with the goal given. It will be saved it in the file plan.txt.

if you go_home and then write in the console

```
$ Cmd> LfD
```

the robot will execute the new trajectory.

## How to use (real robot)

TO DO



