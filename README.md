# tiago_dual_cartesio_config
----------------------------
CartesI/O configuration files for Tiago Dual (omnibase) robot developed by PAL Robotics. 
 

# Features
[![Tiago Dual](https://img.youtube.com/vi/tNu_9wIXg-Q/0.jpg)](https://youtu.be/tNu_9wIXg-Q "Tiago Dual")


The stack is depicted in the ```tiago_dual_cartesio_config/stack/tiago_dual.stack``` file.

How to run
----------
Simply:

```reset && mon launch tiago_dual_cartesio_config/launch/cartesio.launch```

It is possible to check the collision model by enabling ```Collision Enabled``` in the ```RobotModel``` display in RVIZ. Notice that different collision models can be used.

How to run on the robot
-----------------------
To run the controller on the robot first run the ```ros_control_bridge.py```:

```rosrun tiago_dual_cartesio_config ros_control_bridge.py```

this will forward the solution from the controller to the ```JointTrajectory``` controllers running on the robot. 

Then run the controller using:

```reset && mon launch tiago_dual_cartesio_config/launch/cartesio.launch```

How to move the robot
-------------------------------
First, perform the steps in **How to run on the robot**. Then run the launch file:

```reset && mon launch tiago_dual_cartesio_config teleop.launch```

This will start the teleoperation node that allows to control the arms and the gripper for the robot. Then to move the base of the robot run the launch file:

```reset && mon launch tiago_dual_cartesio_config joystick_controller.launch```


