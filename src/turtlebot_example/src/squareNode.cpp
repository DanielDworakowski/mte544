//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various
// inputs and outputs.
//
// Author: James Servos
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <SquareCommander.hpp>

int main(int argc, char **argv)
{
  //
  //Initialize the ROS framework
  ros::init(argc,argv,"main_control");
  ros::NodeHandle n;
  //
  // What deals with driving in a square.
  SquareCommander cmd(n, .2);
  //
  //Set the loop rate
  ros::Rate loop_rate(20);    //20Hz update rate
  //
  // Main control loop.
  while (ros::ok()) {
  	loop_rate.sleep();
  	ros::spinOnce();
    cmd.step();
  }

  return 0;
}
