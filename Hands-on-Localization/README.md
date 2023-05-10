# Hands-on-Localization
Aruco based slam

1. Put this files in your turtlebot_simulation package.  
2. To run the simulation, run the following commands:  
2.1 roslaunch turtlebot_simulation hol_kobuki_basic.launch   
2.2 rosrun turtlebot_simulation hol_Aruco_based_SLAM.py   
2.3 rosrun turtlebot_simulation hol_image_sub.py   
2.4 rosrun turtlebot_simulation vel_command.py (for moving the robot)  


3. If you want to change right or left wheel velocities, then do the following   
3.1 rosparam set /left_wheel 0.5   
3.2 rosparam set /right_wheel 0.5    
