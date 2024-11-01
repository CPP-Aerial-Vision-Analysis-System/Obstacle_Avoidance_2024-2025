#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import random

def create_random_pointcloud():
    # Create an empty list to hold points
    points = []
    for _ in range(1000):
        # Generate random x, y, z coordinates
        x = random.uniform(0, 100.0)
        y = random.uniform(0, 100.0)
        z = random.uniform(0, 100.0)
        points.append([x, y, z])
    
    # Define fields for PointCloud2
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]
    
    # Create a PointCloud2 message
    cloud_msg = pc2.create_cloud(rospy.Header(frame_id="lidar_frame"), fields, points)
    return cloud_msg

def talker():
    # Initialize the ROS node
    rospy.init_node('talker', anonymous=True)

    # Publisher for the PointCloud2 topic
    lidar_scan_pub = rospy.Publisher('chatter', PointCloud2, queue_size=10)

    # Set loop rate to 10Hz
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        rospy.loginfo("Inside")
        # Create random point cloud
        cloud_msg = create_random_pointcloud()
        
        # Publish the point cloud
        lidar_scan_pub.publish(cloud_msg)
        
        # Sleep to maintain loop rate
        rate.sleep()

    rospy.loginfo("Done")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
