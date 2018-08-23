// *****************************
/*
    Publisher que publica mapa simples - usado para teste de mapas
    
    Criado por: Matheus Abrantes
    Criado em: 20/03/18
    Ultima manutencao: 20/03/18

*/
//********************************


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/OccupancyGrid.h>

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  
  nav_msgs::OccupancyGrid map_;


  ros::Publisher chatter_pub = n.advertise<nav_msgs::OccupancyGrid>("chatter", 1000);

  ros::Rate loop_rate(10);  //Hz

  std::string map_frame_id_;

  int height = 2;
  int width = 2;
  double resolution = 1.0;



  while(ros::ok()){
    //map_frame_id_ = ros::this_node::getName() + "/local_map";
    map_frame_id_ = "local_map";
    map_.header.frame_id = map_frame_id_;
    map_.info.width = width;
    map_.info.height = height;
    map_.info.resolution = resolution;
    map_.info.origin.position.x = -static_cast<double>(width) / 2 * resolution;
    map_.info.origin.position.y = -static_cast<double>(height) / 2 * resolution;
    map_.info.origin.orientation.w = 1.0;
    map_.data.assign(width * height, -1);  // Fill with "unknown" occupancy.

    chatter_pub.publish(map_);

    ros::spinOnce();

    loop_rate.sleep();
  }

 

  return 0;
}