#!/usr/bin

echo 'running IQ tutorial sim'

gnome-terminal --title Gazebo --working-directory=/home/suas/Desktop/Obstacle_Avoidance_2024-2025/ROS-IQ-Tutorials -e "bash -c 'source ./devel/setup.bash; roslaunch iq_sim lidar.launch'"

gnome-terminal --title MavProxy -e "bash -c 'cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console --out 127.0.0.1:14551'"

sleep 10

gnome-terminal --title MavROS --working-directory=/home/suas/Desktop/Obstacle_Avoidance_2024-2025/ROS-IQ-Tutorials -e "bash -c 'source ./devel/setup.bash; roslaunch iq_gnc apm.launch'"

gnome-terminal --title IQAvoidance --working-directory=/home/suas/Desktop/Obstacle_Avoidance_2024-2025/ROS-IQ-Tutorials -e "bash -c 'source ./devel/setup.bash; rosrun iq_gnc avoidance_sol'"