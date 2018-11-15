// ROS INCLUDES
#include <ros/ros.h>

// PCL INCLUDES
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

// OTHER INCLUDES
#include <cmath>
#include <string>

using namespace std;
int Arr[4];

// GLOBAL VARIABLES
ros::Publisher pub_down_points;                                 // publisher for downsampled point cloud
pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
ros::Time header_stamp;
std::string header_frame_id;
bool is_dense;
bool flag1, flag2;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input_cloud){

    pcl_conversions::toPCL(*input_cloud, *cloud);

    cout << "arr[0]" << Arr[0] << "\n";
    cout << "arr[1]" << Arr[1] << "\n";

    cout << "arr[2]" << Arr[2] << "\n";
    cout << "arr[3]" << Arr[3] << "\n";


    header_stamp = input_cloud->header.stamp;
    header_frame_id = input_cloud->header.frame_id;
    is_dense = input_cloud->is_dense;
    flag1 = true;
}

void coordinates_callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{

	int i = 0;
	// print all the remaining numbers
	for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		Arr[i] = floor(*it);
		i++;
	}
  flag2 = true;
	return;
}

// MAIN
int main(int argc, char** argv){
    double rateHz = 20;
    flag1 = false;
    flag2 = false;

    // Initializing ROS node
    ros::init(argc, argv, "pc_downsampler");

    ros::NodeHandle pc_nh;
    ros::Rate r(rateHz);

    // Creating a ROS subscriber for the input point cloud
    ros::Subscriber sub = pc_nh.subscribe("/point_cloud/cloud_registered", 1, cloud_cb);
    ros::Subscriber coord_sub = pc_nh.subscribe("/coordinates", 1, coordinates_callback);
    // Creating a ROS publisher for the output point cloud
    pub_down_points = pc_nh.advertise<sensor_msgs::PointCloud2>("output_topic", 1);

    while(ros::ok()) {
    if(flag1 && flag2 ) {
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB> down_cloud;
      pcl::fromPCLPointCloud2(*cloud,*temp_cloud);
      //if(Arr[0] < )
      down_cloud.clear();
      if(Arr[1] > Arr[3]) {
        int temp;
        temp = Arr[1];
        Arr[1] = Arr[3];
        Arr[3] = temp;
      }

      if(Arr[0] > Arr[2]) {
        int temp;
        temp = Arr[0];
        Arr[0] = Arr[2];
        Arr[2] = temp;
      }

      // First push all columns (x / width) in a row (y / height) then change row
      for(int j = Arr[1]; j < Arr[3]; j+=1){
          for(int i = Arr[0]; i < Arr[2]; i+=1){
              down_cloud.push_back(temp_cloud->at(i,j));
              // std::cout << "i & j " << i << " " << j << " : " << temp_cloud->at(i,j) << std::endl;
          }
      }
      std::cout << "temp_cloud just sampled has width " << temp_cloud->width << " and height " << temp_cloud->height << "!" << std::endl;
      std::cout << "Down cloud just sampled has width " << down_cloud.width << " and height " << down_cloud.height << "!" << std::endl;

      //
  //  Setting correctly width and height
    //  down_cloud.resize(down_cloud.width * down_cloud.height);                         // not necessary now
      down_cloud.width = abs(Arr[2] - Arr[0]) * abs(Arr[3] - Arr[1]);
      down_cloud.height = 1;
      //
        // Convert to ROS data type
      pcl::PCLPointCloud2 out_cloud;
      pcl::toPCLPointCloud2(down_cloud, out_cloud);
      sensor_msgs::PointCloud2 output_pc;
      pcl_conversions::fromPCL(out_cloud, output_pc);

      // // Write frame info to msg
      output_pc.width = down_cloud.width;
      output_pc.height = down_cloud.height;

      output_pc.header.stamp = header_stamp;
      output_pc.header.frame_id = header_frame_id;
      output_pc.is_dense = is_dense;

      pub_down_points.publish(output_pc);
}
      ros::spinOnce();
      r.sleep();
    }




}
