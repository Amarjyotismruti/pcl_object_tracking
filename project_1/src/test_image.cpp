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

class HSV_tracker
{
 public:
  int* HSV;
  ros::NodeHandle nh;
  ros::Subscriber sub_2;
 
 public:
  HSV_tracker(int* x)
  {
    HSV=x;
    sub_2=nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 1, &HSV_tracker::image_callback,this);
  }
 private:
  void image_callback(const sensor_msgs::ImageConstPtr& img_ptr);

};

void HSV_tracker::image_callback(const sensor_msgs::ImageConstPtr& img_ptr)
{
  cv::Mat cv_image;
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(img_ptr);
	cv_image=cv_ptr->image;
  cv::Mat imgHSV;

  cv::cvtColor(cv_image, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 
  cv::Mat imgThresholded;
 
 int iLowH=cvGetTrackbarPos("LowH","Control");
 int iHighH=cvGetTrackbarPos("HighH","Control");
 int iLowS=cvGetTrackbarPos("LowS","Control");
 int iHighS=cvGetTrackbarPos("HighS","Control");
 int iLowV=cvGetTrackbarPos("LowV","Control");
 int iHighV=cvGetTrackbarPos("HighV","Control");
  

   cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
  //morphological opening (remove small objects from the foreground)
  cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );
  cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) ); 

   //morphological closing (fill small holes in the foreground)
  cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20)) ); 
  cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20)) );

  
  cv::imshow("WINDOW",imgThresholded);
  cv::waitKey(5);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  
  cv::namedWindow("WINDOW", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("Control", CV_WINDOW_AUTOSIZE);
  int iLowH =0;
  int iHighH=20;

  int iLowS =0; 
  int iHighS=255;

  int iLowV =0;
  int iHighV=255;

  //Create trackbars in "Control" window
 cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
 cvCreateTrackbar("HighH", "Control", &iHighH, 179);

 cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
 cvCreateTrackbar("HighS", "Control", &iHighS, 255);

 cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
 cvCreateTrackbar("HighV", "Control", &iHighV, 255);

 int HSV[6];
 HSV[0]=iLowH;
 HSV[1]=iHighH;
 HSV[2]=iLowS;
 HSV[3]=iHighS;
 HSV[4]=iLowV;
 HSV[5]=iHighV;
 
 HSV_tracker tracker(HSV);


  
  ros::spin();
}