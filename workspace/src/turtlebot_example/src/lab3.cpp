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
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include "cpp/turtlebot_example/lab3Config.h"
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <PRM.hpp>
#include <PoseController.hpp>


ros::Publisher marker_pub;
PRM * g_prm = NULL;


#define TAGID 0


double ips_x;
double ips_y;
double ips_yaw;
geometry_msgs::Quaternion ips_orientation;

//Callback function for the Position topic (LIVE)

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg) {
	//This function is called when a new position message is received
	double X = msg.pose.pose.position.x; // Robot X psotition
	double Y = msg.pose.pose.position.y; // Robot Y psotition
 	double Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

	std::cout << "X: " << X << ", Y: " << Y << ", Yaw: " << Yaw << std::endl ;
}

//Callback function for the Position topic (SIMULATION)
void pose_callback_sim(const gazebo_msgs::ModelStates& msg)
{

    uint32_t i;
    for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

    ips_x = msg.pose[i].position.x ;
    ips_y = msg.pose[i].position.y ;
    ips_orientation = msg.pose[i].orientation;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);

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
  // ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
  ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback_sim);
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
  // goals.push_back(std::make_pair(4,0));
  // goals.push_back(std::make_pair(8,-4)); // proper.
  // goals.push_back(std::make_pair(8,0));
  //goals.push_back(std::make_pair(4,2));
  //goals.push_back(std::make_pair(8,4)); // proper.
  goals.push_back(std::make_pair(5,8.5));
  //
  // Setup topics to Publish from this node
  ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
  //
  // Velocity control variable
  geometry_msgs::Twist vel;
  //
  // Pose controller
  PoseController pose_ctrlr(n);
  //
  // Pose measurenment and ref
  geometry_msgs::PoseWithCovarianceStamped pose_meas;
  geometry_msgs::PoseWithCovarianceStamped pose_ref;
  //
  // RL.
  // goals.push_back(std::make_pair(1,3));
  // goals.push_back(std::make_pair(3,3.5));
  // goals.push_back(std::make_pair(4.5,0.5));
  g_prm = new PRM(n, start, goals);
  srv.setCallback(f);
  g_prm->buildMap();

  // Loop.
  while (ros::ok()) {
  	loop_rate.sleep(); //Maintain the loop rate
  	ros::spinOnce();   //Check for new messages

    ////////////////// Follow Path ////////////////////////////
    // Eigen::MatrixXd path_matrix;
    // double dist_error;
    // double min_dist_error = 0.1;
    // int num_points = path_matrix.rows();
    // for (int i = 0; i < num_points; ++i) {
    //   do
    //   {
    //     double x_meas = ips_x; // Robot X position
    //     double y_meas = ips_y; // Robot Y position
    //     double x_ref = path_matrix(i, 0); // Robot ref x position
    //     double y_ref = path_matrix(i, 1); // Robot ref y position
    //     double y_diff = y_ref - y_meas;
    //     double x_diff = x_ref - x_meas;
    //     dist_error = std::sqrt(std::pow(y_diff,2) + std::pow(x_diff,2));

    //     //Main loop code goes here:
    //     pose_meas.pose.pose.position.x = ips_x;
    //     pose_meas.pose.pose.position.y = ips_y;
    //     pose_meas.pose.pose.orientation = ips_orientation;
    //     pose_ref.pose.pose.position.x = x_ref;
    //     pose_ref.pose.pose.position.y = y_ref;
    //     vel = pose_ctrlr.get_vel(pose_meas, pose_ref);

    //     velocity_publisher.publish(vel); // Publish the command velocity
    //   }while(dist_error > min_dist_error);
    // }
  }

  delete g_prm;
  return 0;
}
