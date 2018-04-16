#Course Project Tutorial
Author: HUANG, Kan
Date: 2018.4.8
##Enviroment
The newest version of regarding softwares is recommended.
[ROS: Kinetic](http://wiki.ros.org/kinetic)
[V-REP: 3.5.0 PRO EDU](http://coppeliarobotics.com/files/V-REP_PRO_EDU_V3_5_0_Linux.tar.gz)
Plugins(Please git clone them from the latest master):
[vrep_ros_bridge](https://github.com/lagadic/vrep_ros_bridge)
[vrep_ros_interface](https://github.com/CoppeliaRobotics/v_repExtRosInterface)
Platform: Ubuntu 16.04 LTS

For vrep_ros_bridge, if you choose V-REP 3.5.0 PRO EDU, please notice:

>If you are using the last version of V-REP (V3_4_0_Linux) and you are under Ubuntu 14.04 you have to download an older version of V-REP (V-REP_PRO_V3_3_2_64_Linux for example) and copy paste the file compiledRosPlugins/libv_repExtRos.so from here. If you are using Ubuntu 16.04 and ROS Kinetic you will need to compile the plugin by yourself: Copy the folders vrep_plugin and vrep_common from home/user/Desktop/V-REP_PRO_V3_3_2_64_Linux/programming/ros_packages in your catkin_ws/src and do a catkin_make. You will find in devel the file libv_repExtRos.so. You will need to copy it in the root of the new V-REP Folder.


The above environment is test by TA. If you have any problems, please contact khuangak@connect.ust.hk

##Tutorial
Here are some useful links for you.
The hyperlinks above, which containing the installation tutorials, and
[Ros Bridge Installation test](http://wiki.ros.org/vrep_ros_bridge#Installation_test)
Be notified, since installation of vrep_ros_bridge uses catkin_make, and ROS Interface plugin for V-REP uses
catkin build, you can create different catkin workspaces for them, refer to this article [Create a temporary catkin workspace for ROS Interface](http://analuciacruz.me/articles/RosInterface_kinetic/).

Remember, we only need a temporary catkin workspace to compile the source files of vrep_ros_bridge and vrep_ros_interface, because we just need the library files(*.so), which should be copied into root path of vrep for vrep to load.
##Demo
After configing the environment, run
```
roscore
```
first.
And in another terminal, run(cd to VREP_ROOT)
```
./vrep.sh
```
You should see the loading details of plugins, including ROS Bridge and ROS Interface.
In V-REP, load the scene file *env.ttt*.
You can use
```
rostopic list
```
to see the Topics of vrep.
Like
```
rostopic echo /vrep/cmd_vel
```
For controling the Pioneer p3dx robot, type the following in terminal
```
rostopic pub -r 10 /vrep/cmd_vel geometry_msgs/Twist  '{linear:  {x: 1.0, y: 1.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```
where x stand for the velocity of left wheel, and y for the left.
A [demo video](https://drive.google.com/file/d/1JxcH519VuLYpr4ukUh8mx_vIoU89ZSsR/view) is attached for your information.

Hints: when you don't know the syntax of command, hit *tab* key in the console.
Passing the command using ROS package: you should work by yourself, its part of the project.
