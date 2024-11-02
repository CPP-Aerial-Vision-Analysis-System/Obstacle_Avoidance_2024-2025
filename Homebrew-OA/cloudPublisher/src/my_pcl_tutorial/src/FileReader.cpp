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

	ros::Rate loopRate(1);

	while(ros::ok()){

		// Create pointcloud object
		sensor_msgs::PointCloud2 cloud_blob;

		// Load the file
		if (pcl::io::loadPCDFile("/home/ethan/Projects/Obstacle_Avoidance_2024-2025/Homebrew-OA/cloudPublisher/src/my_pcl_tutorial/src/sampleData/office1.pcd", cloud_blob) == -1) {
			PCL_ERROR ("Couldn't read file\n");
			return(-1);
		}

		cloud_blob.header.frame_id = "map";

		// Publish point cloud
		lidarScanPub.publish(cloud_blob);

		loopRate.sleep();

	}
	return 1;
}