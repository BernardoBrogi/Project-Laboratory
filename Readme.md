## Introduction


This package is an improved version of the KUKA LWR EPFL package robot lasa (https://github.com/epfl-lasa/kuka-lwr-ros).
All the bugs contained in that package are solved, and we also added other funtionalities to the package.

In simulation, the robot can be moved with a haptic device, recording the trajectory executed by the robot and then replaying it.


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

For istance writing the command "go_home" in the console will make the robot move in the home position.

Hereafter are presented the commands added in order also to exploit the haptic device.
The haptic device we worked with is the Omega3. You may need to adjust the path to the libraries of the haptic device.

Running the command

```
$ LfD
```

on the console will allow to use the haptic device to move the robot using a Cartesian Controller and recordin the trajectory in the file data_ee.txt.
After 10 seconds the program will stop. In order to avoid singularities and be sure that the Cartesian Controller works properly is better to go_home before using this command.

Now if you go_home again and run the command

```
$ LfD_replay
```

the robot will execute the trajectory previously recorded.

## How to use (real robot)

TO DO




