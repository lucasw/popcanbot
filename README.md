# popcanbot

ros robot with two-wheels plus caster steering and a gripper on an arm for competing in the SRS popcan challenge:

http://www.robothon.org/robothon/popcan.php

This shows the latest with the simulation https://www.youtube.com/watch?v=9LyC6aFFdv4

The sim grippers aren't great at holding onto the can.

# Dimensions

"The US standard can is 4.83 inches (0.1227 m) high, 2.13 inches in diameter at the lid, and 2.60 inches (0.0660 m) in diameter at the widest point of the body."

https://en.wikipedia.org/wiki/Beverage_can

The gazebo coke_can.dae measures 120 mm (?), maybe it isn't oversized.

# Dependencies

Currently in jade and kinetic it is necessary to

    git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git

because it is not available as a deb.

If using kinetic, get the kinetic branch:

     cd gazebo_ros_pkgs
     git checkout remotes/origin/kinetic-devel

# gazebo simulation

In ros kinetic:

    roslaunch popcanbot_gazebo_control popcanbot_gazebo_control.launch

Pre gazebo 7 (ros jade and perhaps earlier, haven't tested in anything earlier than jade):

    roslaunch popcanbot_gazebo_control popcanbot_gazebo_control.launch world_name:=worlds/empty.world

If it is the first time gazebo has launched, this may take a long time before you see

    [Spawn status: SpawnModel: Successfully spawned model]

and the model appears in rviz and gazebo.

Gazebo frequently doesn't exit cleanly after pressing ctrl-c, you may have to kill the roscore and restart it to run again.

If it does exit cleanly it is a good idea to:

    rosparam delete /

Or at least set sim_time to false.

# commanding motion

Any generator of cmd_vel messages can work here, though teleop twist keyboard and teleop joy are scaled inappropriately by default so need adjustment.

Stop the car:

    rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}" -1

Drive straight forward:

    rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"

Turn to left:

    rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.02}}"

Turn to right:

    rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: -0.02}}"

Trying to go to fast glitches the gazebo simulation- it's possible toroidal/capsule shaped wheels would do better.
(The advice given in ode/bullte simulation tutorials for high speed car simulations don't model the spinning wheels physically interacting with the ground, it is handled with rays- but it's not clear if gazebo can handle that approach.)

# linear.y vs. angular.z

It's impossible to command a car/bicycle/ackerman vehicle to drive only sideways (linear.y) without also driving forward or backward (linear.x) and combining rotation (angular.z),
so it is a matter of convention to have to always command some linear.x and then angular.z or linear.y which may or may not be achieveable depending on the distance of the steering wheels to the fixed wheels, and the joint limits of the steering wheels.

It appears that commanding linear.x in combination with angular.z is the preferred method (and linear.y results even if desired to be zero).
