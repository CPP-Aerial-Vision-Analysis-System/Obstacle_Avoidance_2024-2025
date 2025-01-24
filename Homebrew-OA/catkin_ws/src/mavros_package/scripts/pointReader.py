#!/usr/bin/env python3                                                                                           /usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from mavros_msgs.msg import Mavlink
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
import math
import time

# Function to convert the point cloud into MAVLink obstacle distance messages
def create_obstacle_distance_msg(point):
    """
    Create a MAVLink obstacle distance 3D message from point cloud data.
    """

    # Create the dimensions of the point
    x, y, z = point

    # Fill MAVLink OBSTACLE_DISTANCE_3D (11037) fields
    mav_msg = mavlink2.MAVLink_obstacle_distance_3d_message(
        time_boot_ms=int(time.time() * 1000),  # Current time in ms
        x=float(x),
        y=float(y),
        z=float(z),
        min_distance=0.2,  # Example min distance in meters
        max_distance=10.0,  # Example max distance in meters
        obstacle_id=1,  # Example obstacle ID
        sensor_type=mavlink2.MAV_DISTANCE_SENSOR_LASER,  # Lidar sensor type
        frame=mavlink2.MAV_FRAME_BODY_NED,  # Example frame
    )
    return mav_msg

def pointcloud_callback(data, mav_pub):
    """
    Callback function for processing PointCloud2 data and publishing MAVLink messages.
    """
    # Convert PointCloud2 to a list of points
    points = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))

    # Log the number of points
    rospy.loginfo(f"Number of points in the point cloud: {len(points)}")
    rospy.loginfo(f"First point in the cloud: {points[0]}")


def main():
    """
    Main function to initialize ROS node, subscribers, and publishers.
    """
    rospy.init_node('unilidar_to_mavlink', anonymous=True)

    # Publisher for MAVLink messages
    mav_pub = rospy.Publisher('/mavros/mavlink/to', Mavlink, queue_size=10)

    # Subscriber for UniLidar point cloud data
    rospy.Subscriber('/unilidar/cloud', PointCloud2, pointcloud_callback, callback_args=mav_pub)

    rospy.loginfo("UniLidar subscriber and MAVLink publisher node started.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
