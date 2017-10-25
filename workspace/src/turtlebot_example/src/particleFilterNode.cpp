//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various
// inputs and outputs needed for this lab
//
// Author: James Servos
// Edited: Nima Mohajerin
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include "Eigen/Dense"
#include <mutex>
#include <random>
#include "lab2Math.hpp"

#include <dynamic_reconfigure/server.h>
#include "cpp/turtlebot_example/lab2Config.h"

ros::Publisher pose_publisher;
ros::Publisher marker_pub;

double g_ips_x;
double g_ips_y;
double g_ips_yaw;
//
// Matricies.
uint32_t g_nParticles = 4;
Eigen::MatrixXd g_particles(3, g_nParticles);
Eigen::MatrixXd g_particlesPred(3, g_nParticles);
Eigen::MatrixXd g_w(1, g_nParticles);
//
// Motion.
Eigen::MatrixXd g_Amat = Eigen::MatrixXd::Identity(3,3);
Eigen::MatrixXd g_Bmat = Eigen::MatrixXd::Identity(3,3); //properly size this.
Eigen::MatrixXd g_Rmat = Eigen::MatrixXd::Identity(3,3);
//
// Measurement.
Eigen::MatrixXd g_Cmat = Eigen::MatrixXd::Identity(3,3);
Eigen::MatrixXd g_Qmat = Eigen::MatrixXd::Identity(3,3);
Eigen::MatrixXd g_Umat = Eigen::MatrixXd::Zero(3,1); //properly size this.
Eigen::MatrixXd g_Meas = Eigen::MatrixXd::Zero(3,1);
//
// Uniform dist generator.
std::default_random_engine g_generator;
std::uniform_real_distribution<double> g_uniform(0.0,1.0);
//
// Mutex for reconfigure.
std::mutex g_mutex;

short sgn(int x) { return x >= 0 ? 1 : -1; }

void reconfigureCallback(turtlebot_example::lab2Config &config, uint32_t level)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  g_particles = Eigen::MatrixXd::Random(3, config.nParticles);
  g_particles.topRows(2) *= config.posPriorRange;
  g_particles.row(2) *= config.thetaPriorRange;
  g_particlesPred = Eigen::MatrixXd::Zero(3, config.nParticles);
  g_w = Eigen::MatrixXd::Zero(1, config.nParticles);

}

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg)
{
    uint32_t i;
    for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

    g_ips_x = msg.pose[i].position.x ;
    g_ips_y = msg.pose[i].position.y ;
    g_ips_yaw = tf::getYaw(msg.pose[i].orientation);

}

//Callback function for the Position topic (LIVE)
/*
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{

	g_ips_x X = msg.pose.pose.position.x; // Robot X psotition
	g_ips_y Y = msg.pose.pose.position.y; // Robot Y psotition
	g_ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}*/

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    //This function is called when a new map is received

    //you probably want to save the map into a form which is easy to work with
}

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty
//	  vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;

    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s) y0+=sgn(dy2); else x0+=sgn(dx2);
        if (d < 0) d += inc1;
        else {
            d += inc2;
            if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

//
// implements particle filtering.
void particleFilter()
{
  std::cout << "-----------------\n";
  std::lock_guard<std::mutex> lock(g_mutex);
  std::cout << "Amat\n" << g_Amat << std::endl;
  std::cout << "g_particles\n" << g_particles << std::endl;
  std::cout << "g_particlescol(0)\n" << g_particles.col(0) << std::endl;
  std::cout << "amat*g_particlescol(0)\n" << g_Amat * g_particles.col(0) << std::endl;
  std::cout << "g_Bmat\n" << g_Bmat << std::endl;
  std::cout << "g_Umat\n" << g_Umat << std::endl;
  for (uint32_t part = 0; part < g_particles.cols(); ++part) {
    Eigen::MatrixXd e = g_Rmat.array().sqrt() /* Eigen::MatrixXd::Ones(3,1)*/ /* random normal*/;
    // std::cout << "e:\n" << e * Eigen::MatrixXd::Ones(3,1) << std::endl;
    e = e * Eigen::MatrixXd::Ones(3,1); // remove
    g_particlesPred.col(part) = g_Amat * g_particles.col(part) + g_Bmat * g_Umat + e;
    // g_w.col(part) = normpdf(g_Meas, g_Cmat * g_particlesPred.col(part), g_Qmat);
  }
  auto W_mat = cumsum1D(g_w);
  std::cout << "W_mat\n" << W_mat << std::endl;
  //
  // Importance sampling.
  double maxW = W_mat(g_particles.cols() - 1);
  for (uint32_t part = 0; part < g_particles.cols(); ++part) {
    Eigen::MatrixXd::Scalar seed = maxW * g_uniform(g_generator);
    //
    // Too lazy to implement a binary search...
    uint32_t firstLarger = W_mat.cols() - 1; // last index.
    for (uint32_t w_idx; w_idx < W_mat.cols(); ++w_idx) {
      if (W_mat.col(w_idx)(0) > seed){
        firstLarger = w_idx;
        break;
      }
    }
    g_particles.col(part) = g_particlesPred.col(firstLarger);
  }
  Eigen::MatrixXd centered = g_particles.colwise() - g_particles.rowwise().mean();
  Eigen::MatrixXd cov = (centered.adjoint() * centered) / double(g_particles.rows() - 1);
  std::cout << "Mean X: " << g_particles.rowwise().mean() << std::endl;
  std::cout << "Var X: " << cov << std::endl;
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
  ros::init(argc,argv,"main_control");
  ros::NodeHandle n;
  //
  //Subscribe to the desired topics and assign callbacks
  ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
  ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
  //
  //Setup topics to Publish from this node
  ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
  pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
  //
  // Initialize dynamic reconfiguration.
  dynamic_reconfigure::Server<turtlebot_example::lab2Config> srv;
  dynamic_reconfigure::Server<turtlebot_example::lab2Config>::CallbackType f;
  f = boost::bind(&reconfigureCallback, _1, _2);
  srv.setCallback(f);
  //
  //Velocity control variable
  geometry_msgs::Twist vel;
  //
  //Set the loop rate
  ros::Rate loop_rate(20);    //20Hz update rate
  //
  // Matrix initialization


  while (ros::ok()) {
    //
    // Looper.
  	loop_rate.sleep(); //Maintain the loop rate
  	ros::spinOnce();   //Check for new messages
  	//Main loop code goes here:
  	vel.linear.x = 0.1; // set linear speed
  	vel.angular.z = 0.3; // set angular speed

  	velocity_publisher.publish(vel); // Publish the command velocity

    particleFilter();
  }

  return 0;
}