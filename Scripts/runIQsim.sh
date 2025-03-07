#!/usr/bin

echo 'running IQ tutorial sim'

gnome-terminal --title mavros -e 'roslaunch mavros apm.launch fcu_url:=udp://:14550@'

gnome-terminal --title UnitreeLidar --working-directory=/home/suas/Desktop/Obstacle_Avoidance_2024-2025/Boring-2D/unilidar_sdk/unitree_lidar_ros -e "bash -c 'source ./devel/setup.bash; roslaunch unitree_lidar_ros run.launch'"

gnome-terminal --title RosToMav  --working-directory=/home/suas/Desktop/Obstacle_Avoidance_2024-2025/Homebrew-OA/catkin_ws -e "bash -c 'source ./devel/setup.bash; rosrun mavros_package 3DAvoid.py'"

gnome-terminal --title rqt_graph -e 'rqt_graph'