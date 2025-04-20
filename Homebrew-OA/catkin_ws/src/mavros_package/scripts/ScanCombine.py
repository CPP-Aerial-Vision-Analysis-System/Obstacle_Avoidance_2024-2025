#!/usr/bin/env python3

import struct
import rospy
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import math
from pymavlink import mavutil
import time

rospy.init_node('ScanCombine', anonymous=True)

time.sleep(1)

pub = rospy.Publisher('/ScanCombine', PointCloud2, queue_size=10)
num_scan = 0
scan_combine = []

def lidar_callback(data):
    """
    Callback function for processing PointCloud2 data and publishing MAVLink messages.
    """

    global pub, num_scan

    # Convert PointCloud2 to a list of points
    points = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))

    if num_scan < 8:
        scan_combine.extend(points)
        num_scan += 1
    
    else:
        num_scan = 0
        header = std_msgs.msg.Header()

        header.stamp = rospy.Time.now()
        header.frame_id = 'unilidar_lidar'

        scaled_polygon_pcl = pc2.create_cloud_xyz32(header, scan_combine)

        scan_combine.clear()

        # Publish the message
        pub.publish(scaled_polygon_pcl)


def main():
    
    

    # Initialize the ROS node
    
    

    # Subscriber for unitree
    rospy.Subscriber('/unilidar/cloud', PointCloud2, lidar_callback)
    rospy.loginfo("Scan Combine Node started.")
    rospy.spin()
    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass