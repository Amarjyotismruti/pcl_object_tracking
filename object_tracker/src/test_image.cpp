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
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

void image_callback(const sensor_msgs::ImageConstPtr& img_ptr)
{
  cv::Mat cv_image;
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(img_ptr);
	cv_image=cv_ptr->image;
    
  cv::imshow("WINDOW",cv_image);
  cv::waitKey(5);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  ros::Subscriber sub_2;
  cv::namedWindow("WINDOW", CV_WINDOW_AUTOSIZE);
  sub_2=nh.subscribe<sensor_msgs::Image>("image_mask", 1,image_callback);
  
  ros::spin();
}