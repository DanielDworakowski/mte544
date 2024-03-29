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


double ips_x = 999;
double ips_y;
double ips_yaw;
geometry_msgs::Quaternion ips_orientation;

//Callback function for the Position topic (LIVE)

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg) {
	//This function is called when a new position message is received
	ips_x = msg.pose.pose.position.x; // Robot X psotition
	ips_y = msg.pose.pose.position.y; // Robot Y psotition
 	ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	ips_orientation  = msg.pose.pose.orientation;

	std::cout << "X: " << ips_x << ", Y: " << ips_y << ", Yaw: " << ips_yaw << std::endl ;
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
  ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
	geometry_msgs::Pose pose_ips;
	ros::Publisher ips_pose_publisher = n.advertise<geometry_msgs::PoseArray>("ips_posei", 1);
  // ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback_sim);
  // Set the loop rate
  ros::Rate loop_rate(20);    //20Hz update rate
  //
  // Dyanamic reconfigure.
  dynamic_reconfigure::Server<turtlebot_example::lab3Config> srv;
  dynamic_reconfigure::Server<turtlebot_example::lab3Config>::CallbackType f;
  f = boost::bind(&reconfigureCallback, _1, _2);
  //
  // PRM DO NOT REORDER TOO LAZY TO MAKE THIS WORK PROPERLY need reconfig to happen.
  // coord start = std::make_pair(1,5); // sim
  #pragma message("Goals are integers...")
	while(ips_x>=999)
	{
		ros::spinOnce();   //Check for new messages
	}
	double offsetX = 1.0;
	double offsetY = 3.0;
	dcoord start = std::make_pair(ips_x+offsetX, ips_y+offsetY); // temp RL
  // dcoord start = std::make_pair(ips_x + offsetX,ips_y+offsetY); // temp RL
  std::vector<dcoord> goals;
  //
  // Simulation.
  // double offsetX = 1.0;
  // double offsetY = 5.0;
  //
  // Real life.

  //
  // SIM.
  // goals.push_back(std::make_pair(4 + offsetX, 0 + offsetY));
  // goals.push_back(std::make_pair(8 + offsetX,-4 + offsetY)); // proper.
  // goals.push_back(std::make_pair(8 + offsetX, 0 + offsetY));
  // //
  // RL.
	// goals.push_back(std::make_pair(2 ,0.5));
	goals.push_back(std::make_pair(1 ,3));
	goals.push_back(std::make_pair(3 ,3.5));
	goals.push_back(std::make_pair(4.5 ,0.5));
  // goals.push_back(std::make_pair(2 + 1,0.5+3));
  // goals.push_back(std::make_pair(0 + 1,0+3)); // proper.
	// goals.push_back(std::make_pair(3.5 + 1,-2.5+3));

  // Setup topics to Publish from this node
	// ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
  ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
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
  g_prm = new PRM(n, start, goals, offsetX, offsetY);
  srv.setCallback(f);
  g_prm->buildMap();

  // Loop.
  while (ros::ok()) {

    loop_rate.sleep(); //Maintain the loop rate
  	ros::spinOnce();   //Check for new messages

    ////////////////// Follow Path ////////////////////////////
    std::queue<coord> path_stack = g_prm->m_paths;
    double dist_error;
    double min_dist_error = 0.2;
    // while(!path_stack.empty())
    // {
    //   coord point = path_stack.top();
    //   path_stack.pop();vel
    //   double x_ref = point.first; // Robot ref x position
    //   double y_ref = point.second; // Robot ref y position
    //   std::cout << "x_ref: " << x_ref << std::endl;
    //   std::cout << "y_ref: " << y_ref << std::endl;
    // }
// #pragma message("uncomment!")
    while(!path_stack.empty() && ros::ok()) {
      coord point = path_stack.front();
      path_stack.pop();
      // std::cout << "tru" << '\n';
      do
      {
				geometry_msgs::PoseArray particles;
				particles.header.frame_id = "/map";
				pose_ips.position.x = ips_x;
				pose_ips.position.y = ips_y;
				pose_ips.position.z = 0.0;
				pose_ips.orientation = tf::createQuaternionMsgFromYaw(ips_yaw);
				particles.poses.push_back(pose_ips);
				ips_pose_publisher.publish(particles);
        double x_meas = ips_x; // Robot X position
        double y_meas = ips_y; // Robot Y position
        double x_ref = 0.1*point.first - offsetX; // Robot ref x position
        double y_ref = 0.1*point.second - offsetY; // Robot ref y position
        double y_diff = y_ref - y_meas;
        double x_diff = x_ref - x_meas;
        dist_error = std::sqrt(std::pow(y_diff,2) + std::pow(x_diff,2));
        // std::cout << dist_error << '\n';
        //Main loop code goes here:
        pose_meas.pose.pose.position.x = ips_x;
        pose_meas.pose.pose.position.y = ips_y;
        pose_meas.pose.pose.orientation = ips_orientation;
        pose_ref.pose.pose.position.x = x_ref;
        pose_ref.pose.pose.position.y = y_ref;
        vel = pose_ctrlr.get_vel(pose_meas, pose_ref);
        // std::cout << "----------------" << std::endl;
        // std::cout << "x_meas: " << x_meas << std::endl;
        // std::cout << "x_ref: " << x_ref << std::endl;
        // std::cout << "y_meas: " << y_meas << std::endl;
        // std::cout << "y_ref: " << y_ref << std::endl;
        // std::cout << "dist_error: " << dist_error << std::endl;
        // std::cout << ips_orientation << '\n';
				// std::cout << vel << '\n';


        velocity_publisher.publish(vel); // Publish the command velocity
        loop_rate.sleep(); //Maintain the loop rate
      	ros::spinOnce();   //Check for new messages
      }while(dist_error > min_dist_error && ros::ok());
    }
  }

  delete g_prm;
  return 0;
}
