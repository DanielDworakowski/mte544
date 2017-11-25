//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 3
//
// Author: James Servos
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include "cpp/turtlebot_example/lab3Config.h"
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <PRM.hpp>

ros::Publisher marker_pub;
PRM * g_prm = NULL;

#define TAGID 0

//Callback function for the Position topic (LIVE)

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg) {
	//This function is called when a new position message is received
	double X = msg.pose.pose.position.x; // Robot X psotition
	double Y = msg.pose.pose.position.y; // Robot Y psotition
 	double Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

	std::cout << "X: " << X << ", Y: " << Y << ", Yaw: " << Yaw << std::endl ;
}

//Example of drawing a curve
void drawCurve(int k) {
  // Curves are drawn as a series of stright lines
  // Simply sample your curves into a series of points

  double x = 0;
  double y = 0;
  double steps = 50;

  visualization_msgs::Marker lines;
  lines.header.frame_id = "/map";
  lines.id = k; //each curve must have a unique id or you will overwrite an old ones
  lines.type = visualization_msgs::Marker::LINE_STRIP;
  lines.action = visualization_msgs::Marker::ADD;
  lines.ns = "curves";
  lines.scale.x = 0.1;
  lines.color.r = 1.0;
  lines.color.b = 0.2*k;
  lines.color.a = 1.0;

  //generate curve points
  for(int i = 0; i < steps; i++) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0; //not used
    lines.points.push_back(p);

    //curve model
    x = x+0.1;
    y = sin(0.1*i*k);
  }

  //publish new curve
  marker_pub.publish(lines);

}

void reconfigureCallback(turtlebot_example::lab3Config &config, uint32_t level)
{
  (void) level;
  std::cout << "Got prm callback!\n";
  if (g_prm) {
    g_prm->reconfigure(config, level);
  }
}

int main(int argc, char **argv) {
  //
  // Initialize the ROS framework
  ros::init(argc,argv,"main_control");
  ros::NodeHandle n;
  //
  // Subscribe to the desired topics and assign callbacks
  ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
  //
  // Setup topics to Publish from this node
  ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
  //
  // Velocity control variable
  geometry_msgs::Twist vel;
  //
  // Set the loop rate
  ros::Rate loop_rate(20);    //20Hz update rate
  //
  // Dyanamic reconfigure.
  dynamic_reconfigure::Server<turtlebot_example::lab3Config> srv;
  dynamic_reconfigure::Server<turtlebot_example::lab3Config>::CallbackType f;
  f = boost::bind(&reconfigureCallback, _1, _2);
  //
  // PRM DO NOT REORDER TOO LAZY TO MAKE THIS WORK PROPERLY need reconfig to happen.
  coord start = std::make_pair(0,0);
  std::vector<coord> goals;
  //
  // SIM.
  goals.push_back(std::make_pair(4,0));
  #pragma message("Find out how to actually transform these into a non-zero based thing")
  goals.push_back(std::make_pair(8,4));
  // goals.push_back(std::make_pair(8,-4)); // proper.
  goals.push_back(std::make_pair(8,0));
  //
  // RL.
  // goals.push_back(std::make_pair(1,3));
  // goals.push_back(std::make_pair(3,3.5));
  // goals.push_back(std::make_pair(4.5,0.5));
  g_prm = new PRM(n, start, goals);
  srv.setCallback(f);
  g_prm->buildMap();
  g_prm->rviz();
  //
  // Loop.
  while (ros::ok()) {
  	loop_rate.sleep(); //Maintain the loop rate
  	ros::spinOnce();   //Check for new messages
  	//Main loop code goes here:
  	vel.linear.x = 0.1; // set linear speed
  	vel.angular.z = 0.3; // set angular speed

  	velocity_publisher.publish(vel); // Publish the command velocity
  }

  delete g_prm;
  return 0;
}
