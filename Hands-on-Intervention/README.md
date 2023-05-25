For Base Orientation:   

1. roslaunch turtlebot_simulation my_turtlebot_hoi.launch    
2. roslaunch turtlebot_simulation inter.launch      
 

For Mazhar_with_mobile_base_working folder     
Place all the files except launch and rviz file in the "turtlebot_simulation" package and put the launch file in launch folder of "turtlebot_simulation" package and turtlebot.rviz file in the turtlebot_description in Rviz folder, and catkin_make, then follow below steps 

1. roslaunch turtlebot_simulation my_turtlebot_hoi.launch    
2. rosrun turtlebot_simulation ground_truth.py   
3. rosrun turtlebot_simulation mob_hands_on_func_call_3.py   
------------------------------------------------------------------------


roslaunch turtlebot_simulation swiftpro_basic.launch

rosrun intervention help.py

For Mazhar_Intervention folder    
Place all the files in the "turtlebot_simulation" package and catkin_make, then follow below steps     
1. roslaunch turtlebot_simulation swiftpro_basic.launch    
2. rosrun turtlebot_simulation hands_on_func_call_3.py 
