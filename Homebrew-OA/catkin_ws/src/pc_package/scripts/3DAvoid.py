#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import open3d as o3d
import numpy as np
import math
def main():
    
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
        if z == 0 or x == 0:
            print("div by 0")
            continue

        r = math.sqrt(x**2 + y**2 + z**2)
        theta = math.atan(math.sqrt(x**2 + y**2) / z)
        phi = math.atan(y/x)

        if phi <= 19.7499:
            top.append((r, theta, phi))
        elif phi > 19.7499 and phi <= 53.966:
            mid.append((r, theta, phi))
        else:
            bot.append((r, theta, phi))

    top.sort()
    mid.sort()
    bot.sort()
    
    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass