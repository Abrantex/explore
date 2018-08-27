#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//for test
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  std_msgs::String msg;
  std_msgs::Int8 free_pass_msg;
  std::stringstream ss;

  int free_pass = 1,i;

  float limites[3] = {(0.7/2),-0.92,0.7};

  //float minx= 1000,miny= 1000,minz = 1000;
  //float maxx = -1000,maxy = -1000,maxz = -1000;

  pcl::PointCloud<pcl::PointXYZ> cloud1;
  pcl::fromROSMsg (*input, cloud1);

  int cloud_size =  cloud1.size();

  for (size_t i = 0; i < cloud_size; ++i)
  {

    if(cloud1[i].z < limites[2] && (abs(cloud1[i].x) < limites[0])){
      free_pass = 0;
    }


    /*if(cloud1[i].x < minx){
      minx = cloud1[i].x;
    }
    if(cloud1[i].y < miny){
      miny = cloud1[i].y;
    }
    if(cloud1[i].z < minz){
      minz = cloud1[i].z;
    }

    if(cloud1[i].x > maxx){
      maxx = cloud1[i].x;
    }
    if(cloud1[i].y > maxy){
      maxy = cloud1[i].y;
    }if(cloud1[i].z > maxz){
      maxz = cloud1[i].z;
    }*/

  }

  ss << "FREE PASS: " <<free_pass;

  //ss << "min: "<<minx<<","<<miny<<","<<minz<<" Max: "<<maxx<<","<<maxy<<","<<maxz;

  msg.data = ss.str();

  

  // Publish the data.
  free_pass_msg.data = free_pass;
  pub.publish (free_pass_msg);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "free_pass_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<std_msgs::Int8> ("output", 1);

  // Spin
  ros::spin ();
}