gnome-terminal --title Gazebo --working-directory=/home/suas/Desktop/Obstacle_Avoidance_2024-2025/ROS-IQ-Tutorials -e "bash -c 'gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world'"

gnome-terminal --title Gazebo --working-directory=/home/suas/Desktop/Obstacle_Avoidance_2024-2025/ROS-IQ-Tutorials -e "bash -c 'cd ~/ardupilot/ArduCopter/ && ../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --out=udp:127.0.0.1:14551'"

