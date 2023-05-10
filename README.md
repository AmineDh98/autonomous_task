

## RUNNING

You will find many launch files in the turtlebot_simulation package:
Launch any of those to run either the mobile base alone, the manipulator alone, or the whole robot.
Some maps has Aruco and some not 

To execute the exploration code you run the following commands:

roslaunch turtlebot_simulation turtlebot_hoi_circuit1_copy.launch 

rosrun turtlebot_simulation hol_Aruco_based_SLAM.py

rosrun turtlebot_simulation hol_image_sub.py (optional if you want to use Aruco detection)

rosrun turtlebot_simulation turtlebot_online_path_planning_node.py

rosrun turtlebot_simulation exploration.py


```
