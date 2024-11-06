#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import open3d as o3d
import numpy as np
import math
def main():
    # Initialize the ROS node
    rospy.init_node('chatter', anonymous=True)

    # Publisher for the PointCloud2 topic
    lidar_scan_pub = rospy.Publisher('pointcloud', PointCloud2, queue_size=10)

    # Set loop rate to 10Hz
    rate = rospy.Rate(10)
    
    cloud_o3d = o3d.io.read_point_cloud("/home/ethan/Projects/Obstacle_Avoidance_2024-2025/Homebrew-OA/cloudPublisher/src/my_pcl_tutorial/src/sampleData/office1.pcd")
    points = np.asarray(cloud_o3d.points)
    # Create a list of tuples in (x, y, z) format
    ros_points = [(p[0], p[1], p[2]) for p in points]

    cloud_msg = pc2.create_cloud_xyz32(rospy.Header(frame_id="map"), ros_points)

    while not rospy.is_shutdown():

        # Publish the point cloud
        lidar_scan_pub.publish(cloud_msg)
        
        # Sleep to maintain loop rate
        rate.sleep()
    
    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass