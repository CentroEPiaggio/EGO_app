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

#define DEBUG            0                                        // if DEBUG 1 prints additional couts

int Arr[4];

// GLOBAL VARIABLES
ros::Publisher pub_down_points;                                 // publisher for downsampled point cloud
int x_res = 320;
int y_res = 240;

// CALLBACK FUNCTION
void downsample_and_publish_cb(const sensor_msgs::PointCloud2ConstPtr& input_cloud){

      // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;

    // Convert to PCL data type
    pcl_conversions::toPCL(*input_cloud, *cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB> down_cloud;
    pcl::fromPCLPointCloud2(*cloud,*temp_cloud);

    // Checking if downsamplable
    if(temp_cloud->width < x_res || temp_cloud->height < y_res){
        ROS_ERROR_STREAM("Desired resolution is bigger than original pointcloud dimensions!");
    }

    // Filtering using iterator
    int x_skip = std::floor(temp_cloud->width / x_res);            // x skip while row downsampling
    int y_skip = std::floor(temp_cloud->height / y_res);        // y skip while column downsampling

    int x_margin = (temp_cloud->width - x_skip * x_res) / 2;    // number of points to skip from left and right
    int y_margin = (temp_cloud->height - y_skip * y_res) / 2;    // number of points to skip from up and down

    // First push all columns (x / width) in a row (y / height) then change row
    for(int j = Arr[0]; j < Arr[2]; j+=1){
        for(int i = Arr[1]; i < Arr[3]; i+=1){
            down_cloud.push_back(temp_cloud->at(i,j));
            // std::cout << "i & j " << i << " " << j << " : " << temp_cloud->at(i,j) << std::endl;
        }
    }

        std::cout << "Cloud just sampled has width " << down_cloud.width << " and height " << down_cloud.height << "!" << std::endl;

    //
    // Setting correctly width and height
    down_cloud.resize(down_cloud.width * down_cloud.height);                         // not necessary now
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
    // //
    // // if(DEBUG){
    // //     std::cout << "Cloud to be published has width " << down_cloud.width << " and height " << down_cloud.height << "!" << std::endl;
    // // }
    // //
    // Setting other message fields
    output_pc.header.stamp = input_cloud->header.stamp;
    output_pc.header.frame_id = input_cloud->header.frame_id;
    output_pc.is_dense = input_cloud->is_dense;
    // //
    // // // Publish the data
    pub_down_points.publish(output_pc);

    delete cloud;

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

// MAIN
int main(int argc, char** argv){

    // Initializing ROS node
    ros::init(argc, argv, "pc_downsampler");
    ros::NodeHandle pc_nh;

    // Creating a ROS subscriber for the input point cloud
    ros::Subscriber sub = pc_nh.subscribe("point_cloud/cloud_registered", 1, downsample_and_publish_cb);
    ros::Subscriber coord_sub = pc_nh.subscribe("/coordinates", 1, coordinates_callback);

    // Creating a ROS publisher for the output point cloud
    pub_down_points = pc_nh.advertise<sensor_msgs::PointCloud2>("output_topic", 1);

    // Success message
    std::cout << "Downscaled points (to " << x_res << "x" << y_res << ") are being published from input_topic to output_topic!" << std::endl;

    // Spin
    ros::spin ();

}
