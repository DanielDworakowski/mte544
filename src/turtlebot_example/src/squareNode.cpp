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
#include <Pose.hpp>
#include <PID.hpp>

int main(int argc, char **argv)
{
  //
  //Initialize the ROS framework
  ros::init(argc,argv,"main_control");
  ros::NodeHandle n;
  //
  //Subscribe to the desired topics and assign callbacks
  Pose pose(n);
  //
  //Setup topics to Publish from this node /cmd_vel_mux/input/teleop
  ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
  //
  //Velocity control variable
  geometry_msgs::Twist vel;
  //
  //Set the loop rate
  ros::Rate loop_rate(20);    //20Hz update rate
  //
  // Main control loop.
  while (ros::ok()) {
  	loop_rate.sleep();
  	ros::spinOnce();

    vel.linear.x = 0.2; // set linear speed
    vel.angular.z = 0.2; // set angular speed
  	velocity_publisher.publish(vel); // Publish the command velocity
  }

  return 0;
}
