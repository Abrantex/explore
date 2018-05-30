// *****************************
/*
    Teste criado para o PIBIC 18 - gerar pontos aleatorios e realizar 
    controle do turtlesim ate esses pontos
    
    Criado por: Matheus Abrantes
    Criado em: -------
    Ultima manutencao: /03/18

*/
//********************************

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float robot[2];


void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{


  float angle_min,range1,angle_max,angle_incr,angle,x,y,r_verifica;
  int N,conta,Naceito = 1;
  float p[2]; //[y,x]

  angle_min = msg->angle_min;
  angle_max = msg->angle_max;
  angle_incr = msg->angle_increment;

  // NUMERO DE AMOSTRAS
  N = floor((angle_max-angle_min)/angle_incr);
  srand(time(NULL));

  //ENQUANTO PONTO NAO FOR LIVRE
  while(Naceito){

      p[1] = (float(rand()) / float(RAND_MAX)) * (1-(0)) -0; //min x =0
      //p[0] = (float(rand()) / float(RAND_MAX)) * (1-(-1)) -1; // [-1,1]
      p[0] = (float(rand()) / float(RAND_MAX)) * (1-(0)) -0;

      //se o range for global
      //p[1]+= robot[1];
      //p[0]+= robot[0];

      ROS_INFO("p:[x],[y]: [%f],[%f]",p[1],p[0]);

    
    for(conta=0;conta<N;conta++){

      angle = angle_min+conta*angle_incr;

      range1 = msg->ranges[conta];


      if(!isnan(range1)){
        
        x = range1*cos(angle);
        y = range1*sin(angle);

        //ROS_INFO("[x],[y]: [%f],[%f]",x,y);

        r_verifica = (p[0]-y)*(p[0]-y);
        r_verifica = (p[1]-x)*(p[1]-x) + r_verifica;
        r_verifica = sqrt(r_verifica);

        Naceito = 0;
        if(r_verifica < 0.6){
          ROS_INFO("Nao Aceito!");
          Naceito = 1;
          break;
        }
      }

    }
  }
  ROS_INFO("FIM");


  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = p[1];  //default 1.0
  goal.target_pose.pose.position.y = p[0];  // default 0.0 para ir para frente.
  goal.target_pose.pose.orientation.w = 1.0;

  robot[0] += p[0];
  robot[1] += p[1];


  //goal.target_pose.pose.position.x = 0.5;
  //goal.target_pose.pose.position.y = -2;
  //goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.1);

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  ROS_INFO("Result");

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");



}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  robot[1] = 0;
  robot[0] = 0;

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("kinect_scan", 1000, chatterCallback);
  ros::spin();

  /*
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;


  //goal.target_pose.pose.position.x = 0.5;
  //goal.target_pose.pose.position.y = -2;
  //goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.1);

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");
  */
  return 0;
}