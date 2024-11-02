#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace ros;

int main(int argc, char **argv){
	// Initialzie ROS, allows remapping through command line
	ros::init(argc, argv, "chatter");

	ros::NodeHandle n;

	// Returns a ros Publisher object
	ros::Publisher lidarScanPub = n.advertise<sensor_msgs::PointCloud2>("chatter", 10);

	// Create pointcloud object
	pcl::PointCloud<pcl::PointXYZ> cloud(new pcl::PointCloud<pcl::PointXYZ);

	// Load the file
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("office1.pcd", *cloud) == -1) {
		PCL_ERROR ("Couldn't read file\n");
		return(-1);
	}

	// Convert to sensor_msgs::PointCloud2 object
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(cloud, cloud_msg);

	cloud_msg.header.frame_id = "map";

	// Publish point cloud
	lidarScanPub.publish(cloud_msg);

	return 1;
}