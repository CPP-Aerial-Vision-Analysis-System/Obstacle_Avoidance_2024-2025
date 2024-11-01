#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
using namespace ros;

int main(int argc, char **argv){
	// Initialzie ROS, allows remapping through command line
	ros::init(argc, argv, "chatter");

	ros::NodeHandle n;

	// Rate object specifies frequency to loop at
	ros::Rate loopRate(1);

	// Returns a ros Publisher object
	ros::Publisher lidarScanPub = n.advertise<sensor_msgs::PointCloud2>("chatter", 10);

	srand(time(0));

	while(ros::ok()){

		// Create pointcloud object
		pcl::PointCloud<pcl::PointXYZ> cloud;
		
		// Fill cloud with random points
		for (int i = 0; i < 256; i++){
			pcl::PointXYZ newPoint;
			newPoint.x = ((float)rand()) / (float)RAND_MAX;
			newPoint.y = ((float)rand()) / (float)RAND_MAX;
			newPoint.z = ((float)rand()) / (float)RAND_MAX;
			cloud.points.push_back(newPoint);
		}

		// Convert to sensor_msgs::PointCloud2 object
		sensor_msgs::PointCloud2 cloud_msg;
		pcl::toROSMsg(cloud, cloud_msg);

		cloud_msg.header.frame_id = "map";

		// Publish point cloud
		lidarScanPub.publish(cloud_msg);

		loopRate.sleep();
	}

	return 1;
}