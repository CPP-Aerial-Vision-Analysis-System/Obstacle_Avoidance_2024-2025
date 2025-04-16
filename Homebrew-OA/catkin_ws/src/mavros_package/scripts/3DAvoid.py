#!/usr/bin/env python3

import struct
import rospy
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import math
from pymavlink import mavutil
import time

rospy.init_node('send_obstacle_3D', anonymous=True)

time.sleep(1)

pub = rospy.Publisher('/send_obstacle_3D', PointCloud2, queue_size=10)

import sys
sys.path.append("/usr/local/lib/")

# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"
os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'

master= mavutil.mavlink_connection('tcp:localhost:5762', dialect='ardupilotmega')

start_time =  int(round(time.time() * 1000))
current_milli_time = lambda: int(round(time.time() * 1000) - start_time)
current_time_ms = current_milli_time()

rate = rospy.Rate(100)

def convertX(r, theta, phi):
    x = r * math.sin(theta) * math.cos(phi)
    return x

def convertY(r, theta, phi):
    y = r * math.sin(theta) * math.sin(phi)
    return y

def convertZ(r, theta, phi):
    z = r * math.cos(theta)
    return z

def lidar_callback(data):
    """
    Callback function for processing PointCloud2 data and publishing MAVLink messages.
    """

    global pub, rate

    # print("got callback")

    # Convert PointCloud2 to a list of points
    points = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))

    # <19.7499
    top = []
    # 19.7499 - 53.966
    mid = [[]]
    # >53.966
    bot = [[]]

    cloud_itr = 0
    
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
            top.append((r, theta, phi, cloud_itr))
        elif phi > 19.7499 and phi <= 53.966:
            mid[int(theta / 6)].append((r, theta, phi, cloud_itr))
        else:
            bot[theta / 10].append((r, theta, phi, cloud_itr))
    
        cloud_itr += 1
    
    def get_min(p1):
        if(len(p1) > 0):
            lowest = 0
            itr = 0
            lowest_itr = 0

            for i in p1:
                if i[0] < lowest:
                    lowest = i[0]
                    lowest_itr = itr
                itr +=1 

            return(itr)
        
        else: return(0)

    finalList = []

    # if (len(top) > 0):
    #     point = points[get_min(top)]
    #     if point:
    #         finalList.append(point)
    for i in mid : finalList.append(points[get_min(i)])
    for i in bot : finalList.append(points[get_min(i)])
    
    

    for point in finalList:
        obstacle_x = point[0]
        obstacle_y = point[1]
        obstacle_z = point[2]

        # Sensor and frame configuration
        sensor_type = 0  # Laser
        obstacle_id = 1
        frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

        # Create the raw MAVLink message
        raw_msg = master.mav.obstacle_distance_3d_send(
            time_boot_ms = current_time_ms * 1000,   # Current time in microseconds
            sensor_type = 0,
            frame= frame,
            obstacle_id= 65535,
            x=float(obstacle_x),
            y=float(obstacle_y),
            z=float(obstacle_z),
            
            min_distance=float(.01),
            max_distance=float(25)
        )

        time.sleep(.1)

    # Sensor and frame configuration
    # sensor_type = 0  # Laser
    # obstacle_id = 1
    # frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

    # # Create the raw MAVLink message
    # raw_msg = master.mav.obstacle_distance_3d_send(
    #     time_boot_ms = current_time_ms * 1000,   # Current time in microseconds
    #     sensor_type = 0,
    #     frame= frame,
    #     obstacle_id= 65535,
    #     x=float(1),
    #     y=float(0),
    #     z=float(0),
        
    #     min_distance=float(.01),
    #     max_distance=float(25)
    # )

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'your_frame'

    scaled_polygon_pcl = pc2.create_cloud_xyz32(header, finalList)

    # Publish the message
    pub.publish(scaled_polygon_pcl)
    

def main():
    
    

    # Initialize the ROS node
    
    

    # Subscriber for unitree
    rospy.Subscriber('/velodyne_points', PointCloud2, lidar_callback)
    rospy.loginfo("UniLidar subscriber and MAVLink publisher node started.")
    rate.sleep()
    rospy.spin()
    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass