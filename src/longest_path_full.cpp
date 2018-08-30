/*
 * Author: Matheus Abrantes
 * Created: 03/26/18
 * Last Modification: 04/16/18
 *
 *
 * Based on robot cleanner in the course CS460 (/tuto_ws/src/teste_turlesim/robot_cleaner.cpp)
 * Random walk of robot. 
 *
 *  NOT SUPORT OF OBSTACLE AVOID, USE FOR SIMULATION ONLY!!! THE ROBOT MAY COLIDE.
 *  BACKUP 5- aqui ainda tem o código de movimentação linear a rotacional simultâneas
 * 
 *
 */


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/tf.h>

#include <fstream> //for file
#include "std_msgs/Int8.h"

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;
nav_msgs::Odometry robot_odom;


//Matheus- set goal:
turtlesim::Pose goal_pose;
nav_msgs::Odometry odom_goal;
ros::Subscriber scan_subscriber;
sensor_msgs::LaserScan the_scan;

int free_pass = 1;


const float x_min = -3.0;
const float y_min = -3.0;
const float x_max = 3.0;
const float y_max = 3.0;

const double PI = 3.14159265359;


double degrees2radians(double angle_in_degrees);
void poseCallback(const nav_msgs::Odometry::ConstPtr & pose_message);
void freeCallback(const std_msgs::Int8::ConstPtr& msg);


double getDistance(double x1, double y1, double x2, double y2);
void moveGoal(double distance_tolerance);

//alternativa para moveGoal.
void move_both(double distance_tolerance);


void rotate (float angular_tolerance);
void move(float error_tolerance);
void move_and_rotate(void);


void gera_ponto(int);


//pontos recentemente visitados. 
float past_x[10] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float past_y[10] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};


//Atualiza o passado do robo
void att_past(void);

void write_file(int,float, float);


int main(int argc, char **argv)
{
	// Initiate new ROS node named "random_walk"
	ros::init(argc, argv, "random_walk");
	ros::NodeHandle n;
	double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;

	float angulo_vinda  = 0.0;

	int Goal_Count = 1;
	
	scan_subscriber = n.subscribe("/free_topic", 10, freeCallback);
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/pose", 10, poseCallback);

	ros::Rate loop_rate(0.5);

   		
	goal_pose.x = 0;
	goal_pose.y = 0;

		
	robot_odom.pose.pose.position.x = 0;
	robot_odom.pose.pose.position.y = 0;
	robot_odom.pose.pose.orientation.z =0;

	srand(time(NULL));


    //**** Matheus: set goal: 
	while(ros::ok()){
		

		gera_ponto(1);

		ROS_INFO("GOAL: %d",Goal_Count);
		write_file(Goal_Count,goal_pose.x,goal_pose.y);
		Goal_Count++;
		
		moveGoal(0.1);
		//move_both(0.1);

		att_past();

		loop_rate.sleep();

		ros::spinOnce();
	}
   return 0;
}


void att_past(){

	int i;

	for(i=0;i<9;i++){
		past_x[9-i] = past_x[8-i];
		past_y[9-i] = past_y[8-i];	
	}

	past_x[0] = robot_odom.pose.pose.position.x;
	past_y[0] = robot_odom.pose.pose.position.y;

}


void write_file(int Goal_Count,float goalX , float goalY){

	ofstream myfile;
    myfile.open ("L_ExploreLOG.txt",ios::app);

    int n;
  	

	if (!myfile.is_open())
	{
		ROS_ERROR("Cannot open example.txt");
		exit(1);
	}

	
	myfile << "pose(:," << Goal_Count << ")=[" << robot_odom.pose.pose.position.x << ";" << robot_odom.pose.pose.position.y << "];";
	myfile << "goal(:," << Goal_Count << ")=[" << goalX << ";" << goalY << "];";


	myfile << "pastx(:," << Goal_Count << ")=[";
	
	for(n = 0;n<10;n++){
		myfile << past_x[n] <<";";
	}

	myfile<<"]; pasty(:," <<Goal_Count << ")=[";
	
	for(n = 0;n<10;n++){
		myfile << past_y[n] <<";";
	}
	myfile<<"];"<<endl;							
  
  	myfile.close();

}




void gera_ponto(int flag){

	float angulo, raio;

	ROS_INFO("pose: %f,%f",robot_odom.pose.pose.position.x,robot_odom.pose.pose.position.y);

	switch(flag){
		case(0):
		// pontos cartesiados
		
		goal_pose.x = robot_odom.pose.pose.position.x + (float(rand()) / float(RAND_MAX)) * (x_max-(x_min)) +x_min; // between [x_min,x_max]
	  	goal_pose.y = robot_odom.pose.pose.position.y +(float(rand()) / float(RAND_MAX)) *(y_max-(y_min))  +y_min; // between [y_min,y_max]
		goal_pose.theta = 0;
		
		break;

		case(1):
		//polar


		raio = (float(rand()) / float(RAND_MAX)) *  4.0;
		if(free_pass ==1){
			angulo = (float(rand()) / float(RAND_MAX)) *  PI/5 - PI/10; // [-18,18]
			angulo = 0;
		}else{
			do{
				angulo = (float(rand()) / float(RAND_MAX)) *  8*PI/6 + PI/3; //[60 a 300]
			}while( (9*PI/10)< angulo && angulo < (11*PI/10));
		}	
		//angulo = 0;
		goal_pose.x = robot_odom.pose.pose.position.x + raio*cos(angulo + robot_odom.pose.pose.orientation.z);
		goal_pose.y = robot_odom.pose.pose.position.y + raio*sin(angulo + robot_odom.pose.pose.orientation.z);

		break;

	}

	ROS_INFO("goal: %f , %f",goal_pose.x, goal_pose.y);
}



void move(float error_tolerance){

	geometry_msgs::Twist vel_msg;

	float x_error = 10000, x_1_error = 3000;

	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	//angular
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;



	ros::Rate loop_rate(10);
	do{

		//colision avoidance
		//check_pass();
		if(free_pass == 0){
			return;
		}


		x_1_error = x_error;

		velocity_publisher.publish(vel_msg);
		x_error = getDistance(robot_odom.pose.pose.position.x,robot_odom.pose.pose.position.y,goal_pose.x,goal_pose.y);
		
		vel_msg.linear.x = 0.2;

		

		ros::spinOnce(); //acho que nao precisa para publish, porem sem spinOnce a posição nao será atualizada
		loop_rate.sleep(); 

	//enquanto error nao aumentar
	}while(abs(x_1_error) > (abs(x_error) + error_tolerance));
	vel_msg.linear.x = 0;
	velocity_publisher.publish(vel_msg);
}


void rotate(float angular_tolerance){

	geometry_msgs::Twist vel_msg;

	float error_yaw = 100;

	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	//angular
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	ros::Rate loop_rate(10);
	do{
		velocity_publisher.publish(vel_msg);
		error_yaw = (atan2(goal_pose.y-robot_odom.pose.pose.position.y,goal_pose.x - robot_odom.pose.pose.position.x)-robot_odom.pose.pose.orientation.z);
		vel_msg.angular.z = 2.0*error_yaw;


		if(abs(vel_msg.angular.z) > 0.4){
			vel_msg.angular.z = (vel_msg.angular.z)/abs(vel_msg.angular.z)*0.4;
		}
		ros::spinOnce();
		loop_rate.sleep();

		
	}while(abs(error_yaw) > angular_tolerance);

	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);

}

//**move e rotaciona

void move_and_rotate(void){

	geometry_msgs::Twist vel_msg;


	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	//angular
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;


	float error_yaw = (atan2(goal_pose.y-robot_odom.pose.pose.position.y,goal_pose.x - robot_odom.pose.pose.position.x)-robot_odom.pose.pose.orientation.z);

	float  x_error = getDistance(robot_odom.pose.pose.position.x,robot_odom.pose.pose.position.y,goal_pose.x,goal_pose.y);

	vel_msg.angular.z = 2.0*error_yaw;

	vel_msg.linear.x = 0.6*x_error;

	if(abs(vel_msg.angular.z) > 0.4){
		vel_msg.angular.z = (vel_msg.angular.z)/abs(vel_msg.angular.z)*0.4;
	}

	if(abs(vel_msg.linear.x)>0.3){
		vel_msg.linear.x = 0.3*(vel_msg.linear.x)/abs(vel_msg.linear.x);
	}

	velocity_publisher.publish(vel_msg);

}


/**
 *  converts angles from degree to radians  
 */

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}

/**
 *  callback function to update the pose of the robot  
 */

void poseCallback(const nav_msgs::Odometry::ConstPtr & pose_message){
	robot_odom.pose.pose.position.x=pose_message->pose.pose.position.x;
	robot_odom.pose.pose.position.y=pose_message->pose.pose.position.y;
	robot_odom.pose.pose.orientation.z=tf::getYaw(pose_message->pose.pose.orientation);
}


/*
 * get the euclidian distance between two points 
 */
double getDistance(double x1, double y1, double x2, double y2){

	//ROS_INFO("error: %lf",sqrt(pow((x1-x2),2)+pow((y1-y2),2)));

	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

// ** implemented by Matheus Abrantes in 03/25/18 based on the youtube video
void moveGoal(double distance_tolerance){
	
	geometry_msgs::Twist vel_msg;

	int  angular = 1;

	vel_msg.angular.z = 1;


	float error_yaw = 100.0,error_x = 100.0, error_x_i = 1000.0;


	free_pass = 1;

	// valores iniciais
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	//angular
	vel_msg.angular.x =0;	
	vel_msg.angular.y =0;	
	vel_msg.angular.z =0;	

	do{

		
			rotate(0.01);
		
			//ROS_INFO("End of rotate");
			move(0.00000);


	}while(getDistance(robot_odom.pose.pose.position.x,robot_odom.pose.pose.position.y,goal_pose.x,goal_pose.y)>distance_tolerance   && free_pass == 1);
	
	//ROS_INFO("End of moving goal");
	
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);

}

// ** implemented by Matheus Abrantes in 05/30/18 
void move_both(double distance_tolerance){
	
	geometry_msgs::Twist vel_msg;


	float x_error, error_yaw;


	ros::Rate loop_rate(10);
	do{
		/******** Proportional Controller ********/
		//linear velocity in the x-axis
		x_error = getDistance(robot_odom.pose.pose.position.x,robot_odom.pose.pose.position.y,goal_pose.x,goal_pose.y);
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		//angular velocity int he z-axis
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		error_yaw = (atan2(goal_pose.y-robot_odom.pose.pose.position.y,goal_pose.x - robot_odom.pose.pose.position.x)-robot_odom.pose.pose.orientation.z);


		vel_msg.angular.z = 2.0*error_yaw;

		vel_msg.linear.x = 0.6*x_error;

		if(abs(vel_msg.angular.z) > 0.4){
			vel_msg.angular.z = (vel_msg.angular.z)/abs(vel_msg.angular.z)*0.4;
		}

		if(abs(vel_msg.linear.x)>0.3){
			vel_msg.linear.x = 0.3*(vel_msg.linear.x)/abs(vel_msg.linear.x);
		}


		//check_pass();


		velocity_publisher.publish(vel_msg);

		ros::spinOnce();
		loop_rate.sleep();

	}while(getDistance(robot_odom.pose.pose.position.x,robot_odom.pose.pose.position.y,goal_pose.x,goal_pose.y)>distance_tolerance && free_pass == 1);
	cout<<"end move goal"<<endl;
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);

}




void freeCallback(const std_msgs::Int8::ConstPtr& msg){

	free_pass = msg->data;
	if(free_pass == 0){
		ROS_INFO("FREE PASS = 0");
	}

}
