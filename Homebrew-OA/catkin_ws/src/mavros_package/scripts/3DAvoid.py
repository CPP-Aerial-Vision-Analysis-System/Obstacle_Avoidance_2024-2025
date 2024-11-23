#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import open3d as o3d
import numpy as np
import math
from mavros_msgs.msg import State, Mavlink

def convertX(r, theta, phi):
    x = r * math.sin(theta) * math.cos(phi)
    return x

def convertY(r, theta, phi):
    y = r * math.sin(theta) * math.sin(phi)
    return y
def convertZ(r, theta, phi):
    z = r * math.cos(theta)
    return z
def main():
    
    # Initialize the ROS node
    rospy.init_node('send_obstacle_3D', anonymous=True)

    # Publisher for the Mavlink topic
    pub = rospy.Publisher('\mavros\mavlink\to', Mavlink, queue_size=10)
    rate = rospy.Rate(10)

    cloud_o3d = o3d.io.read_point_cloud("/home/ethan/Projects/Obstacle_Avoidance_2024-2025/Homebrew-OA/cloudPublisher/src/my_pcl_tutorial/src/sampleData/office1.pcd")
    points = np.asarray(cloud_o3d.points)

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
        finalList.append(mid[i])
    for i in range(10):
        finalList.append(bot[i])
    
    msg = Mavlink()
    msg.sysid = 1
    msg.compid = 1
    msg.msgid = 1
    msg.payload64 = [0,0,0,0,0,0,0]

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
        
    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass