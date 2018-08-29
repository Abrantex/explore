#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//for test
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "p2os_msgs/SonarArray.h"

ros::Publisher pub;
ros::Subscriber cloudSub;
ros::Subscriber sonarSub;
p2os_msgs::SonarArray sonar_array;
int sonar_mode;

int cloudFree,sonarFree,pathFree;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  //std_msgs::Int8 free_pass_msg;
  std::stringstream ss;

  int i;

  float limites[3] = {(0.7/2),-0.92,0.7};

  pcl::PointCloud<pcl::PointXYZ> cloud1;
  pcl::fromROSMsg (*input, cloud1);

  int cloud_size =  cloud1.size();


  cloudFree = 1;

  for (size_t i = 0; i < cloud_size; ++i)
  {

    if(cloud1[i].z < limites[2] && (abs(cloud1[i].x) < limites[0])){
      cloudFree = 0;
    }

  }

  ss << "FREE PASS: " <<cloudFree;



  

  // Publish the data.
  //free_pass_msg.data = cloudFree;
  //pub.publish (free_pass_msg);
}

int check_pass_sonar(void){
  
  int N = 8, count =0, sonarFree=1;

  float range,range2,threshold,threshold_minor = 0.25;
  threshold = 0.5;

  // do sonar 3 ao 6
  for(count = 2;count<6;count++){
    range = sonar_array.ranges[count];
    if (range < threshold){
    //se distancia[i] menor que limiar
      sonarFree = 0;
      return sonarFree;
    }
  }
  
  for(count = 0;count<2;count++){
    range = sonar_array.ranges[count];

    range2 = sonar_array.ranges[count + 6];

    if (range < threshold_minor || range2 < threshold_minor){
    //se distancia[i] menor que limiar
      sonarFree = 0;
      return sonarFree;
    }
  }

}


void sonar_cb(const p2os_msgs::SonarArray::ConstPtr& msg){

  sonar_array = *msg;

  sonarFree = 1;

  int N = 8, count =0;

  float range,range2,threshold,threshold_minor = 0.25;
  threshold = 0.5;

  // do sonar 3 ao 6
  for(count = 2;count<6;count++){
    range = sonar_array.ranges[count];
    if (range < threshold){
    //se distancia[i] menor que limiar
      sonarFree = 0;
      break;
    }
  }
  
  for(count = 0;count<2;count++){
    range = sonar_array.ranges[count];

    range2 = sonar_array.ranges[count + 6];

    if (range < threshold_minor || range2 < threshold_minor){
    //se distancia[i] menor que limiar
      sonarFree = 0;
      break;
    }
  }





  //sonarFree = check_pass_sonar();
  ROS_INFO("Sonar: %f,%d,%d",sonar_array.ranges[2],sonarFree,sonar_mode);


}


int main (int argc, char** argv){

  std_msgs::Int8 free_pass_msg;

  //int sonar_mode;


  // Initialize ROS
  ros::init (argc, argv, "free_pass_node");
  ros::NodeHandle nh("~");


  nh.param<int>("sonar_mode",sonar_mode,0);


  // Create a ROS subscriber for the input point cloud
  cloudSub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<std_msgs::Int8> ("output", 1);

  //subscriber para o sonar
  sonarSub = nh.subscribe ("sonar",1000,sonar_cb);


  sonarFree = 1;
  cloudFree= sonar_mode;



  ros::Rate loop_rate(5);
  while(ros::ok()){


    free_pass_msg.data = cloudFree;
    if (sonar_mode == 1){
      free_pass_msg.data = cloudFree*sonarFree;
    }


    pub.publish (free_pass_msg);
    
    loop_rate.sleep();
    ros::spinOnce();
  }


  // Spin
  //ros::spin ();
}