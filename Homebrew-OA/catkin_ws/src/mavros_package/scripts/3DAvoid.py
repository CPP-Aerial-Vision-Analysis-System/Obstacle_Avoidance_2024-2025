#!/usr/bin/env python3

import typing

import rospy
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import open3d as o3d
import numpy as np
import math
from mavros_msgs.msg import Mavlink
from mavros.mavlink import convert_to_rosmsg
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from pymavlink import mavutil
import time

def convertX(r, theta, phi):
    x = r * math.sin(theta) * math.cos(phi)
    return x

def convertY(r, theta, phi):
    y = r * math.sin(theta) * math.sin(phi)
    return y

def convertZ(r, theta, phi):
    z = r * math.cos(theta)
    return z

def lidar_callback(data, pub):
    """
    Callback function for processing PointCloud2 data and publishing MAVLink messages.
    """

    rate = rospy.Rate(10)

    # Convert PointCloud2 to a list of points
    points = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))

    # <19.7499
    top = []
    # 19.7499 - 53.966
    mid = []
    # >53.966
    bot = []
    for point in points:
        x = point[0]
        y = point[1]
        z = point[2]
        if x == 0 and y == 0 or x == 0 and y == 0 and z == 0:
            continue

        r = math.sqrt(x**2 + y**2 + z**2)
        theta = math.acos(z / math.sqrt(x**2 + y**2 + z**2))
        phi = math.asin(y / math.sqrt(x**2 + y**2))

        if phi <= 19.7499:
            top.append((r, theta, phi))
        elif phi > 19.7499 and phi <= 53.966:
            mid.append((r, theta, phi))
        else:
            bot.append((r, theta, phi))

    top.sort()
    mid.sort()
    bot.sort()

    finalList = []
    finalList.append(top[0])
    for i in range(6):
        if i < len(mid):
            finalList.append(mid[i])
    for i in range(10):
        if i < len(bot):
            finalList.append(bot[i])

    for point in finalList:
        obstacle_x = point[0]
        obstacle_y = point[1]
        obstacle_z = point[2]

        # Sensor and frame configuration
        sensor_type = 0  # Laser
        obstacle_id = 1
        frame = mavlink.MAV_FRAME_LOCAL_NED

        # Create the raw MAVLink message
        mav_msg = mavlink.MAVLink_obstacle_distance_3d_message(
            time_boot_ms = int(100),  # Current time in microseconds
            sensor_type = int(sensor_type),
            obstacle_id=int(obstacle_id),
            x=float(obstacle_x),
            y=float(obstacle_y),
            z=float(obstacle_z),
            frame=int(frame),
            min_distance=float(0.5),
            max_distance=float(20.0)
        )

        # Pack the MAVLink message
        raw_msg = mav_msg.pack(mavutil.mavlink_connection('udpin:localhost:14540'))

        # Convert to ROS Mavlink message
        ros_msg = Mavlink()
        ros_msg.sysid = 1  # System ID (adjust as needed)
        ros_msg.compid = 1  # Component ID (adjust as needed)
        ros_msg.msgid = mavlink.MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D
        ros_msg.payload64 = list(raw_msg)

        # Publish the message
        pub.publish(convert_to_rosmsg(mav_msg))
        rate.sleep()

def main():
    pub = rospy.Publisher('/mavlink/to', Mavlink, queue_size=10)

    # Initialize the ROS node
    rospy.init_node('send_obstacle_3D', anonymous=True)

    # Subscriber for unitree
    rospy.Subscriber('/unilidar/cloud', PointCloud2, lidar_callback, callback_args=pub)
    rospy.loginfo("UniLidar subscriber and MAVLink publisher node started.")
    rospy.spin()

    

    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass