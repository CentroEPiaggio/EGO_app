#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include <geometry_msgs/Point.h>

ros::Publisher pub;


void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  //PointCloud2
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  //PointCloudXYZ
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
  //Output PointCloud2
  pcl::PCLPointCloud2 cloud_filtered;


  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);
  pcl::fromPCLPointCloud2(*cloud,*cloudXYZ);

  // std::cout <<  "Width: " << cloud->width << std::endl;
  // std::cout << "Height: " << cloud->height << std::endl;


  //Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  int arrayPosition = 300*output.row_step + 300*output.point_step;
  std::cout << "Array Position: " << arrayPosition << "\n" ;

  // compute position in array where x,y,z data start
   int arrayPosX = arrayPosition + output.fields[0].offset; // X has an offset of 0
   int arrayPosY = arrayPosition + output.fields[1].offset; // Y has an offset of 4
   int arrayPosZ = arrayPosition + output.fields[2].offset; // Z has an offset of 8

   std::cout << "X: " << arrayPosX << "\n";
   std::cout << "Y: " << arrayPosY << "\n";
   std::cout << "Z: " << arrayPosZ << "\n";

//    float X = 0.0;
//     float Y = 0.0;
//     float Z = 0.0;
//
//     X = output.data[arrayPosX];
// Y = output.data[arrayPosY];
// Z = output.data[arrayPosZ];
//
// std::cout << "X: " << X << "\n";
// std::cout << "Y: " << X << "\n";
// std::cout << "Z: " << X << "\n";


  pcl_conversions::moveFromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/cloud_pcd", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
