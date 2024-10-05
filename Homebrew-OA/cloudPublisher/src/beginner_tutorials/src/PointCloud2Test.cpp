#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pcl/conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
using namespace ros;

int main(int argc, char **argv){
	// Initialzie ROS, allows remapping through command line
	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

	// Rate object specifies frequency to loop at
	ros::Rate loopRate(10);

	// Returns a ros Publisher object
	ros::Publisher lidarScanPub = n.advertise<std_msgs::String>("chatter", 10);

	while(ros::ok()){

		// Create pointcloud object
		pcl::PointCloud<pcl::PointXYZ> cloud;
		
		// Fill cloud with random points
		for (int i = 0; i < 1000; i++){
			pcl::PointXYZ newPoint;
			newPoint.x = (rand() * 100.0) / RAND_MAX;
			newPoint.y = (rand() * 100.0) / RAND_MAX;
			newPoint.z = (rand() * 100.0) / RAND_MAX;
			cloud.points.push_back(newPoint);
		}

		// Convert to sensor_msgs::PointCloud2 object
		sensor_msgs::PointCloud2 cloud_msg;
		pcl::toROSMsg(cloud, cloud_msg);

		// Publish point cloud
		lidarScanPub.publish(cloud_msg);

		loopRate.sleep();
	}

	return 1;
}