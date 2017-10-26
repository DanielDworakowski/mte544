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
#include "nav_msgs/Odometry.h"

#include <dynamic_reconfigure/server.h>
#include "cpp/turtlebot_example/lab2Config.h"
#include "Visualizer.hpp"

ros::Publisher pose_publisher;
ros::Publisher marker_pub;

double g_ips_x;
double g_ips_y;
double g_ips_yaw;
double g_odom_x;
double g_odom_y;
double g_odom_yaw;
//
// Matricies.
#define NUM_STATES 3
#define F (20.0) //20
#define PERIOD (1/F)

enum MeasType {
  MEAS_ODOM,
  MEAS_IPS
};

Eigen::VectorXf first_vec(3);
bool init_pose_set = false;
bool sampled = false;
uint32_t g_nParticles = 4;
Eigen::MatrixXd g_particles(3, g_nParticles);
Eigen::MatrixXd g_particlesPred(3, g_nParticles);
Eigen::MatrixXd g_w(1, g_nParticles);
Eigen::MatrixXd g_odom_cov(3,3);
//
// Motion.
Eigen::MatrixXd g_Amat = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
Eigen::MatrixXd g_Bmat = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES); //properly size this.
Eigen::MatrixXd g_Rmat = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES) * 1e-3;
//
// Measurement.
Eigen::MatrixXd g_Cmat = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES);
Eigen::MatrixXd g_Qmat = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES) * 1e-2;
Eigen::MatrixXd g_Umat = Eigen::MatrixXd::Zero(NUM_STATES, 1); //properly size this.
Eigen::MatrixXd g_Meas = Eigen::MatrixXd::Zero(NUM_STATES, 1);
//
// Uniform dist generator.
std::default_random_engine g_generator;
std::uniform_real_distribution<double> g_uniform(0.0,1.0);
//
// Normal Dist generator.
Eigen::EigenMultivariateNormal<Eigen::MatrixXd::Scalar> g_normMatGen(Eigen::MatrixXd::Zero(NUM_STATES,1), Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES));
//
// Mutex for reconfigure.
std::mutex g_mutex;
//
// Indicate if there was a new measurement.
bool g_newOdom = false;
bool g_newIPS = false;

short sgn(int x) { return x >= 0 ? 1 : -1; }

void reconfigureCallback(turtlebot_example::lab2Config &config, uint32_t level)
{
  config.nParticles = 100;
  config.thetaPriorRange = 1;
  std::lock_guard<std::mutex> lock(g_mutex);
  g_particles = Eigen::MatrixXd::Random(NUM_STATES, config.nParticles); //.colwise()+first_vec.cast<double>();
  g_particles.topRows(2) *= config.posPriorRange;
  g_particles.row(2) *= config.thetaPriorRange;
  g_particlesPred = Eigen::MatrixXd::Zero(NUM_STATES, config.nParticles);
  g_w = Eigen::MatrixXd::Zero(1, config.nParticles);
}

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg)
{
    uint32_t i;
    for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

    if (!init_pose_set) {
      first_vec(0) = msg.pose[i].position.x;
      first_vec(1) = msg.pose[i].position.y;
      first_vec(2) = tf::getYaw(msg.pose[i].orientation);
      init_pose_set = true;
      sampled = true;
    }
    g_ips_x = msg.pose[i].position.x - first_vec(0);
    g_ips_y = msg.pose[i].position.y - first_vec(1);
    g_ips_yaw = tf::getYaw(msg.pose[i].orientation)- first_vec(2);
    g_ips_yaw = floatMod(g_ips_yaw + M_PI, 2 * M_PI) - M_PI;
    g_newIPS = true;
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

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
 void odom_callback(const nav_msgs::Odometry& msg)
 {
   g_odom_x = msg.pose.pose.position.x;
   g_odom_y = msg.pose.pose.position.y;
   g_odom_yaw = tf::getYaw(msg.pose.pose.orientation);
   auto odom_cov_36 = msg.pose.covariance;
   g_odom_cov(0,0) = odom_cov_36[0];
   g_odom_cov(0,1) = odom_cov_36[1];
   g_odom_cov(0,2) = odom_cov_36[5];
   g_odom_cov(1,0) = odom_cov_36[6];
   g_odom_cov(1,1) = odom_cov_36[7];
   g_odom_cov(1,2) = odom_cov_36[11];
   g_odom_cov(2,0) = odom_cov_36[30];
   g_odom_cov(2,1) = odom_cov_36[31];
   g_odom_cov(2,2) = odom_cov_36[35];
   g_newOdom = true;
  //  std::cout << "odom cov:\n" << g_odom_cov << std::endl;
  //  ROS_DEBUG("odom_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
 }

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
// Implements measurement updates.
void measUpdate(MeasType type)
{

  // for1 each of meas types.
  // if meas recived
  // get Cmat
  // for2
  Eigen::MatrixXd qMeas;
  Eigen::MatrixXd meas;
  qMeas.resizeLike(g_Qmat);
  meas.resizeLike(g_Meas);
  switch (type) {
    case MEAS_IPS:
      // unkown cov.
      meas(0) = g_ips_x;
      meas(1) = g_ips_y;
      meas(2) = g_ips_yaw;
      qMeas = g_Qmat;
    break;
    case MEAS_ODOM:
      meas(0) = g_odom_x;
      meas(1) = g_odom_y;
      meas(2) = g_odom_yaw;
      qMeas = g_odom_cov;
    break;
  }
  //
  // Update the weights.
  for (uint32_t part = 0; part < g_particles.cols(); ++part) {
    g_w.col(part) = normpdf<Eigen::MatrixXd::Scalar>(meas, g_Cmat * g_particlesPred.col(part), qMeas);
  }
  auto W_mat = cumsum1D(g_w);
  //
  // Importance sampling.
  double maxW = W_mat(g_particles.cols() - 1);
  for (uint32_t part = 0; part < g_particles.cols(); ++part) {
    Eigen::MatrixXd::Scalar seed = maxW * g_uniform(g_generator);
    //
    // Too lazy to implement a binary search...
    uint32_t firstLarger = W_mat.cols() - 1; // last index.
    for (uint32_t w_idx = 0; w_idx < W_mat.cols(); ++w_idx) {
      if (W_mat.col(w_idx)(0) > seed){
        firstLarger = w_idx;
        break;
      }
    }
    g_particles.col(part) = g_particlesPred.col(firstLarger);
  }
  //std::cout << "meas: \n" << meas << std::endl;
}

//
// implements particle filtering.
void particleFilter()
{
  //std::cout << "-----------------\n";
  std::lock_guard<std::mutex> lock(g_mutex);
  //
  // Motion model.
  for (uint32_t part = 0; part < g_particles.cols(); ++part) {
    Eigen::MatrixXd e = g_Rmat.array().sqrt();
    e *= g_normMatGen.samples(1);
    //
    // Ensure that all angles are -pi to pi.
    // g_particlesPred.col(part) = g_Amat * g_particles.col(part) + g_Bmat * g_Umat + e;

    auto partCol = g_particles.col(part);
    auto partPredCol = g_particlesPred.col(part);
    partPredCol(0) = partCol(0) + g_Umat(0) * std::cos(partCol(2)) * PERIOD;
    partPredCol(1) = partCol(1) + g_Umat(0) * std::sin(partCol(2)) * PERIOD;
    partPredCol(2) = partCol(2) + g_Umat(2) * PERIOD;
    partPredCol(2) = floatMod(partPredCol(2) + M_PI, 2 * M_PI) - M_PI;
    g_particlesPred.col(part) = partPredCol + e;
  }
  // if ips updated
  if (g_newIPS) {
    measUpdate(MEAS_IPS);
    g_newIPS = false;
  }
  // if (g_newOdom) {
  //   g_particlesPred = g_particles;
  //   measUpdate(MEAS_ODOM);
  //   g_newOdom = false;
  // }
  Eigen::MatrixXd centered = g_particles.rowwise() - g_particles.colwise().mean();
  Eigen::MatrixXd cov = (centered.adjoint() * centered) / double(g_particles.rows() - 1);
  //std::cout << "Mean X:\n" << g_particles.rowwise().mean() << std::endl;
  // std::cout << "Var X:\n" << cov << std::endl;
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
  ros::init(argc,argv,"main_control");
  ros::NodeHandle n;
  //
  //Instanciate Vis
  Visualizer viz(n);

  //
  //Subscribe to the desired topics and assign callbacks
  ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
  ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
  //
  //Setup topics to Publish from this node
  ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
  pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
  ros::Subscriber odom_sub = n.subscribe("/odom", 1, odom_callback);
  //
  //Set the loop rate
  ros::Rate loop_rate(F);    //20Hz update rate
  //
  // Initialize dynamic reconfiguration.
  while (!sampled && !g_newOdom) {
    loop_rate.sleep(); //Maintain the loop rate
    ros::spinOnce();   //Check for new messages
  }
  dynamic_reconfigure::Server<turtlebot_example::lab2Config> srv;
  dynamic_reconfigure::Server<turtlebot_example::lab2Config>::CallbackType f;
  f = boost::bind(&reconfigureCallback, _1, _2);
  srv.setCallback(f);
  //
  //Velocity control variable
  geometry_msgs::Twist vel;

  //
  // ROS loop.
  while (ros::ok()) {
    //
    // Looper.
  	loop_rate.sleep(); //Maintain the loop rate
  	ros::spinOnce();   //Check for new messages
  	//Main loop code goes here:
  	// vel.linear.x = 0.1; // set linear speed
  	// vel.angular.z = 0.3; // set angular speed

    vel.linear.x = 0.1;
    vel.angular.z = 0.3;
    g_Umat(0) = 0.1;
    g_Umat(2) = 0.3;

  	// velocity_publisher.publish(vel); // Publish the command velocity

    particleFilter();
    //std::cout << "g bef:\n" << g_particles << std::endl;
    //std::cout << "g aft:\n" <<g_particles.colwise()+first_vec.cast<double>()  << std::endl;
    //std::cout << "vec:\n" <<first_vec  << std::endl;

    viz.visualize_particle(g_particles.colwise()+first_vec.cast<double>()); //vel
  }

  return 0;
}
