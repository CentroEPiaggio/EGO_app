#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void coordinates_callback(const std_msgs::Float32MultiArray::ConstPtr& array);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);

int Arr[4];
image_transport::Subscriber image_sub;
image_transport::Publisher image_pub;
int main(int argc, char **argv)
{

	ros::init(argc, argv, "arraySubscriber");

	cv::startWindowThread();



	ros::NodeHandle n;
	image_transport::ImageTransport it(n);

	 ros::Subscriber sub = n.subscribe("/coordinates", 1, coordinates_callback);
	 image_sub = it.subscribe("rgb/image_rect_color", 1, imageCallback);
	 image_pub = it.advertise("/image_converter/output_video", 1);


	ros::spin();

	for(int j = 0; j < 4; j++)
	{
		printf("%d", Arr[j]);
	}

	printf("\n");
	return 0;
}

void coordinates_callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{

	int i = 0;
	// print all the remaining numbers
	for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		Arr[i] = *it;
		i++;
	}

	return;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
 	try {
 		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   }
	 catch (cv_bridge::Exception& e)
	 {
		 ROS_ERROR("cv_bridge exception: %s", e.what());
		 return;
	 }

	// Draw an example circle on the video stream
	if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
	cv::rectangle(cv_ptr->image,cv::Point(Arr[0], Arr[1]),cv::Point(Arr[2], Arr[3]),CV_RGB(255,0,0),7);

	// Update GUI Window
	cv::imshow("opencv", cv_ptr->image);

	cv::waitKey(3);

       // Output modified video stream
   image_pub.publish(cv_ptr->toImageMsg());
   }
