// #include <ros/ros.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <boost/foreach.hpp>
// #include <pcl/filters/crop_box.h>
//
// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
// pcl::CropBox<pcl::PointXYZ> boxFilter;
// void callback(const PointCloud::ConstPtr& msg)
// {
//   printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
//
//   pcl::PointXYZ minPt = msg->at(0,0);
//   pcl::PointXYZ maxPtg = msg->at(300,300);
//
//
//   // int index = 100;
//   // float x = msg->points[index].x;
//   // float y = msg->points[index].y;
//   // float z = msg->points[index].z;
//   // std::cout << "Index: " << index << " X: " << x << " Y: " << y << " Z: " << z << std::endl;
//
//   // BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
//   //   printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "sub_pcl");
//   ros::NodeHandle nh;
//   ros::Subscriber sub = nh.subscribe<PointCloud>("/zed/point_cloud/cloud_registered", 1, callback);
//   ros::spin();
// }

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>



ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::fromPCLPointCloud2(*cloud,*temp_cloud);

  // pcl::PointXYZRGB minPt;
  // pcl::PointXYZRGB maxPt;
  // getMinMax3D<PointXYZ>(*temp_cloud,minPt,maxPt);

  if(temp_cloud->isOrganised()) {
    std:: cout << "isOrganised"
  } else {
    std::cout << "Not Organised"
  }
  // pcl::PointXYZRGB minPt = temp_cloud->at(10,10);
  // pcl::PointXYZRGB maxPt = temp_cloud->at(100,100);

  Eigen::Vector4f minV4f = minPt.getArray4fMap();
  Eigen::Vector4f maxV4f = maxPt.getArray4fMap();

  pcl::CropBox<pcl::PointXYZRGB> boxfilter;

  boxfilter.setMin(minV4f);
  boxfilter.setMax(maxV4f);
  boxfilter.setNegative(false);
  boxfilter.setInputCloud(temp_cloud);
  boxfilter.filter(*filtered_cloud);

  // Perform the actual filtering
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloudPtr);
  // sor.setLeafSize (0.05, 0.05, 0.05);
  // sor.filter (cloud_filtered);
  //
  // // Convert to ROS data type
  pcl::PCLPointCloud2 out_cloud;
  pcl::toPCLPointCloud2(*filtered_cloud, out_cloud);

  sensor_msgs::PointCloud2 output_pc;
  pcl_conversions::fromPCL(out_cloud, output_pc);

  //Publish the data
  pub.publish (output_pc);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
