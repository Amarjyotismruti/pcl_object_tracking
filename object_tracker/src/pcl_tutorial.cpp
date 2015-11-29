#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
//openCV includes
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>



ros::Publisher pub;

void cloud_cb(const pcl::PCLPointCloud2ConstPtr& input)
{

pcl::PointCloud<pcl::PointXYZ> cloud_xyz;      // Convert PCL to XYZ data type
pcl::fromPCLPointCloud2(*input, cloud_xyz);
int size=cloud_xyz.points.size();
ROS_INFO("Size of Cloud: %d",size);

pcl::PointCloud<pcl::PointXYZ> mask_xyz;
mask_xyz.height=480;
mask_xyz.width=640;



pcl::PCLPointCloud2 cloud_filtered;           // Using Voxelgrid filtering to sample down point cloud                                              // point cloud
pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
sor.setInputCloud (input);
sor.setLeafSize (0.05, 0.05, 0.05);
sor.filter (cloud_filtered);
// Publish the dataSize
pub.publish (cloud_filtered);
}




int main (int argc, char** argv)
{
// Initialize ROS
ros::init (argc, argv, "my_pcl_tutorial");
ros::NodeHandle nh;
// Create ROS publisher for publishing the point cloud
// Create a ROS subscriber for the input point cloud
ros::Subscriber sub = nh.subscribe ("camera/depth/points", 1, cloud_cb);
// Create a ROS publisher for the output point cloud
pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
// Spin
ros::spin ();

return 0;
}