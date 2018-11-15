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
#include <pcl/filters/passthrough.h>


ros::Publisher pub;
using namespace pcl;
using namespace std;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  //Converted Base PointXYZ Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr PointXYZ_Cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloud,*PointXYZ_Cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //Filters
  pcl::CropBox<pcl::PointXYZ> boxfilter;
  pcl::PassThrough<pcl::PointXYZ> pass;

  PointXYZ_Cloud->resize(PointXYZ_Cloud->width * PointXYZ_Cloud->height);
  // cout << "Cloud Width: " << PointXYZ_Cloud->width << endl;
  // cout << "Cloud height: " << PointXYZ_Cloud->height << endl;
  // cout << "Is the Cloud Organised: " << PointXYZ_Cloud->isOrganized() << endl;
  // cout << "Is the Cloud dense: " << PointXYZ_Cloud->is_dense << endl;

  PointXYZ minPt = PointXYZ_Cloud->points[100];
  PointXYZ maxPt = PointXYZ_Cloud->points[10000];

  if(isFinite(minPt) && isFinite(maxPt)) {

    cout << "MinPt: " << minPt << endl;
    cout << "MaxPt: " << maxPt << endl;

    Eigen::Vector4f minV4f = minPt.getArray4fMap();
    Eigen::Vector4f maxV4f = maxPt.getArray4fMap();


    boxfilter.setMin(minV4f);
    boxfilter.setMax(maxV4f);
    boxfilter.setNegative(true);
    boxfilter.setInputCloud(PointXYZ_Cloud);
    boxfilter.filter(*filtered_cloud);

    cout << "Cloud Width: " << filtered_cloud->width << endl;
    cout << "Cloud height: " << filtered_cloud->height << endl;
    cout << "Is the Cloud Organised: " << filtered_cloud->isOrganized() << endl;
    cout << "Is the Cloud dense: " << filtered_cloud->is_dense << endl;

    // pass.setInputCloud (PointXYZ_Cloud);
    // pass.setFilterFieldName ("z");
    // pass.setFilterLimits (0.0, 1.0);
    // pass.filter (*filtered_cloud);

    pcl::PCLPointCloud2 out_cloud;
    pcl::toPCLPointCloud2(*filtered_cloud, out_cloud);

    sensor_msgs::PointCloud2 output_pc;
    pcl_conversions::fromPCL(out_cloud, output_pc);

    //Publish the data
    pub.publish (output_pc);

  }

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/point_cloud/cloud_registered", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
