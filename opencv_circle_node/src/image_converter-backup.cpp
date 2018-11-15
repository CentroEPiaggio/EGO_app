#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

static const std::string OPENCV_WINDOW = "Image window";
int Arr[90];

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber sub;
  int i = 0;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("rgb/image_rect_color", 1,&ImageConverter::imageCb, this);
    sub = nh_.subscribe("/coordinates", 1, ImageConverter::coordinates_callback);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

  //  cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
  //  cv::destroyWindow(OPENCV_WINDOW);
  }

  void coordinates_callback(std_msgs::Float32MultiArray::ConstPtr& array)
   {
     int i = 0;
     // print all the remaining numbers
     for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
     {
       Arr[i] = *it;
       i++;
     }
     return;
   }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat cv_ptr_img;
    cv::Mat cv_scaled_img;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // std::cout << cv_ptr->image.rows << " :Rows" << std::endl;
    // std::cout << cv_ptr->image.cols << " :cols" << std::endl;
    // cv_ptr_img = cv_ptr->image;

    // if(cv_ptr_img.rows > 0 && cv_ptr_img.cols > 0) {
    //
    //   cv::resize(cv_ptr_img, cv_scaled_img, cv::Size(2560,1440));
    // }


    // Draw an example circle on the video stream
//273.3932189941406, 180.8743896484375, 587.7703857421875, 334.2678527832031s
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    cv::rectangle(cv_ptr->image,cv::Point(273, 180),cv::Point(587, 334),CV_RGB(255,0,0),7);

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
