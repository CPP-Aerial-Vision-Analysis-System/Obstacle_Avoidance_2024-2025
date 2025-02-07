#!/usr/bin/env python3

import struct
import rospy
from mavros.mavros import mavlink
from sensor_msgs.msg import PointCloud2
from mavros_msgs.msg import Mavlink
from pymavlink.dialects.v20 import ardupilotmega as o3d
from pymavlink import mavutil
import time


def lidar_callback(data, pub):
    """
    Callback function for processing PointCloud2 data and publishing MAVLink messages.
    """

    rate = rospy.Rate(10)

    # Sensor and frame configuration
    sensor_type = 0  # Laser
    obstacle_id = 1
    frame = o3d.MAV_FRAME_LOCAL_NED

    master=mavutil.mavlink_connection('udpin:127.0.0.1:14550')
    master.mav.obstacle_distance_3d_send(
        time_boot_ms = int(100),  # Current time in microseconds
        sensor_type = int(sensor_type),
        obstacle_id=int(obstacle_id),
        x=float(0),
        y=float(5),
        z=float(10),
        frame=int(frame),
        min_distance=float(0.5),
        max_distance=float(20.0)
    )

    # Create the raw MAVLink message
    raw_msg = o3d.MAVLink_obstacle_distance_3d_message(
        time_boot_ms = int(100),  # Current time in microseconds
        sensor_type = int(sensor_type),
        obstacle_id=int(obstacle_id),
        x=float(0),
        y=float(5),
        z=float(10),
        frame=int(frame),
        min_distance=float(0.5),
        max_distance=float(20.0)
    )

    payload_bytes = bytearray(raw_msg)
    payload_len = len(payload_bytes)
    payload_octets = int(payload_len / 8)
    if payload_len % 8 > 0:
        payload_octets += 1
        payload_bytes += b"\0" * (8 - payload_len % 8)

    payload_msg = struct.unpack(f"<{payload_octets}Q", payload_bytes)

    # Convert to ROS Mavlink message
    ros_msg = Mavlink()
    ros_msg.sysid = 1  # System ID (adjust as needed)
    ros_msg.compid = 1  # Component ID (adjust as needed)
    ros_msg.msgid = o3d.MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D
    ros_msg.payload64 = payload_msg

    # Publish the message
    pub.publish(raw_msg.convert_to_rosmsg)
    rate.sleep()

def main():
    pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=10)

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