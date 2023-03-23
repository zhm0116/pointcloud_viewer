///------------------Start slam nodes-----------------
roslaunch livox_ros_driver livox_lidar_msg.launch 

roslaunch fast_lio mapping_avia.launch

rosrun roswww map_generator.py

///--------------start server-------------------------
roslaunch rosbridge_server rosbridge_websocket.launch

rosrun tf2_web_republisher tf2_web_republisher 

roslaunch roswww roswww.launch

