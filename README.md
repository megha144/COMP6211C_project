#ELEC6910R, COMP6211C Course Project
Teaching Assitant: HUANG, Kan
Members: Chen Yau Pun, Lei Zhao, Yang Shaohui

##Enviroment
The newest version of regarding softwares is recommended.

- [ROS: Kinetic](http://wiki.ros.org/kinetic)

- [V-REP: 3.5.0 PRO EDU](http://coppeliarobotics.com/files/V-REP_PRO_EDU_V3_5_0_Linux.tar.gz)

- [vrep_ros_bridge](https://github.com/lagadic/vrep_ros_bridge)(already inside vrep_dep)

- [vrep_ros_interface](https://github.com/CoppeliaRobotics/v_repExtRosInterface)(already inside vrep_dep)

- Platform: Ubuntu 16.04 LTS

###Configuration 
1. Download vrep 3.5.0 

2. Add the line `export VREP_ROOT_DIR=/ChangeWithyourPathToVrep/` to ~/.bashrc

3. Add the line `source /path_to_catkin_ws/catkin_ws/devel/setup.bash` to ~/.bashrc

4. `catkin_make`

5. Go to the location of vrep and run `ln -s /YOUR_CATKIN_WS_PATH/devel/lib/libv_repExtRosBridge.so`

###Demo

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

