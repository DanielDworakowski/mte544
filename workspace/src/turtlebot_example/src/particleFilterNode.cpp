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
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include "Eigen/Dense"
#include <mutex>
#include <random>
#include "lab2Math.hpp"
#include "nav_msgs/Odometry.h"

#include <dynamic_reconfigure/server.h>
#include "cpp/turtlebot_example/lab2Config.h"
#include "Visualizer.hpp"

ros::Publisher pose_pub;
ros::Publisher map_pub;
ros::Publisher marker_pub;

double g_ips_x;
double g_ips_y;
double g_ips_yaw;
double g_odom_x;
double g_odom_y;
double g_odom_yaw;
//
// Absolute location.
double g_abs_x;
double g_abs_y;
double g_abs_yaw;
//
// Matricies.
#define NUM_STATES 3
#define F (20.0) //20
#define PERIOD (1/F)

enum MeasType {
  MEAS_ODOM,
  MEAS_IPS
};

///////////////////////////////////////////////////////////////////////////////
//                       Particle filter.                                    //
///////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf first_vec(3);
bool init_pose_set = false;
bool sampled = false;
uint32_t g_nParticles = 100;
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
Eigen::MatrixXd g_Qmat = Eigen::MatrixXd::Identity(NUM_STATES, NUM_STATES) * 1e-4;
Eigen::MatrixXd g_Umat = Eigen::MatrixXd::Zero(NUM_STATES, 1); //properly size this.
Eigen::MatrixXd g_Meas = Eigen::MatrixXd::Zero(NUM_STATES, 1);
Eigen::MatrixXd g_lastOdom = Eigen::MatrixXd::Zero(NUM_STATES, 1);
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
///////////////////////////////////////////////////////////////////////////////
//                       End particle filter.                                //
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//                          Mapping.                                         //
///////////////////////////////////////////////////////////////////////////////

double g_maxR = 10.;
double g_maxTheta = 1.;
double g_gridResM = 0.01; // Length / width of a grid in (m).
double g_mapSizeM = 10;
Eigen::MatrixXd g_M = Eigen::MatrixXd::Ones(g_mapSizeM / g_gridResM, g_mapSizeM / g_gridResM); //Map of probabilities.
Eigen::MatrixXd g_L = Eigen::MatrixXd::Ones(g_mapSizeM / g_gridResM, g_mapSizeM / g_gridResM); //Map of logits.
Eigen::MatrixXd g_L0 = Eigen::MatrixXd::Ones(g_mapSizeM / g_gridResM, g_mapSizeM / g_gridResM); //Map of logits.
nav_msgs::MapMetaData g_mapMeta;
sensor_msgs::LaserScan g_laser;
int8_t * g_mapData = NULL;
uint32_t g_newScanCnt = 0;
double g_emptyProb = 0.4;
double g_notEmptyProb = 0.6;
int32_t g_mapBeta = 5;

double g_pose_scan_x = 0.;
double g_pose_scan_y = 0.;
double g_pose_scan_yaw = 0.;
auto g_poseTime = ros::Time();
///////////////////////////////////////////////////////////////////////////////
//                        End mapping.                                       //
///////////////////////////////////////////////////////////////////////////////
short sgn(int x) { return x >= 0 ? 1 : -1; }

void reconfigureCallback(turtlebot_example::lab2Config &config, uint32_t level)
{
  (void) level;
  //
  // Particle filter.
  std::lock_guard<std::mutex> lock(g_mutex);
  g_particles = Eigen::MatrixXd::Random(NUM_STATES, config.nParticles);
  g_particles.topRows(2) *= config.posPriorRange;
  g_particles.row(2) *= config.thetaPriorRange;
  g_particlesPred = Eigen::MatrixXd::Zero(NUM_STATES, config.nParticles);
  g_w = Eigen::MatrixXd::Zero(1, config.nParticles);
  //
  // Mapping.
  g_maxR = config.maxMeasRange;
  g_maxTheta = config.maxMeasTheta;
  g_gridResM = config.mapGridSize;
  g_mapSizeM = config.mapSize;
  uint32_t mapSize = g_mapSizeM / g_gridResM;
  mapSize += !(mapSize % 2); //If the map is an even number add another gridblock.
  g_M = Eigen::MatrixXd::Ones(mapSize, mapSize) * 0.5;
  g_L.resizeLike(g_M);
  g_L0.resizeLike(g_M);
  g_L0 = log(g_M.array() / (1. - g_M.array()));
  g_L = g_L0;
  g_mapMeta.map_load_time = ros::Time::now();
  g_mapMeta.resolution = g_gridResM;
  g_mapMeta.width = mapSize;
  g_mapMeta.height = mapSize;
  if (g_mapData != NULL) {
    delete[] g_mapData;
  }
  g_mapData = new int8_t[mapSize * mapSize];
  g_mapMeta.origin.position.x = 0 - g_mapMeta.width * g_gridResM / 2.0;
  g_mapMeta.origin.position.y = 0 - g_mapMeta.height * g_gridResM / 2.0;
  g_mapMeta.origin.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
  g_emptyProb = config.emptySpaceProbModifier;
  g_notEmptyProb = config.notEmptySpaceProbModifier;
  g_maxR = config.maxMeasRange;
  g_maxTheta = config.maxMeasTheta;
  g_mapBeta = config.mapBeta;
}

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  g_poseTime = ros::Time::now();
  uint32_t i;
  for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

  if (!init_pose_set) {
    g_mapMeta.origin = msg.pose[i];
    first_vec(0) = msg.pose[i].position.x;
    first_vec(1) = msg.pose[i].position.y;
    first_vec(2) = tf::getYaw(msg.pose[i].orientation);
    init_pose_set = true;
    sampled = true;
  }
  Eigen::MatrixXd e = g_Rmat.array().sqrt();
  e *= g_normMatGen.samples(1);
  // no noise.
  // g_abs_x = msg.pose[i].position.x - first_vec(0);
  // g_abs_y = msg.pose[i].position.y - first_vec(1);
  // g_abs_yaw = tf::getYaw(msg.pose[i].orientation) - first_vec(2);
  g_abs_x = msg.pose[i].position.x;
  g_abs_y = msg.pose[i].position.y;
  g_abs_yaw = tf::getYaw(msg.pose[i].orientation);
  //
  // noisy.
  g_ips_x = msg.pose[i].position.x - first_vec(0) + e(0);
  g_ips_y = msg.pose[i].position.y - first_vec(1) + e(1);
  g_ips_yaw = tf::getYaw(msg.pose[i].orientation) - first_vec(2) + e(2);
  g_ips_yaw = floatMod(g_ips_yaw + M_PI, 2 * M_PI) - M_PI;
  g_abs_yaw = floatMod(g_abs_yaw + M_PI, 2 * M_PI) - M_PI;
  g_newIPS = true;
}

//Callback function for the Position topic (LIVE)
void pose_callbackTbot(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  g_poseTime = ros::Time::now();
	double ips_x = msg.pose.pose.position.x; // Robot X psotition
	double ips_y = msg.pose.pose.position.y; // Robot Y psotition
	double ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
  if (!init_pose_set) {
    g_mapMeta.origin = msg.pose[i];
    first_vec(0) = ips_x;
    first_vec(1) = ips_y;
    first_vec(2) = ips_yaw;
    init_pose_set = true;
    sampled = true;
  }
  Eigen::MatrixXd e = g_Rmat.array().sqrt();
  e *= g_normMatGen.samples(1);
  // no noise.
  // g_abs_x = msg.pose[i].position.x - first_vec(0);
  // g_abs_y = msg.pose[i].position.y - first_vec(1);
  // g_abs_yaw = tf::getYaw(msg.pose[i].orientation) - first_vec(2);
  g_abs_x = ips_x;
  g_abs_y = ips_y;
  g_abs_yaw = ips_yaw;
  //
  // noisy.
  g_ips_x = ips_x - first_vec(0) + e(0);
  g_ips_y = ips_y - first_vec(1) + e(1);
  g_ips_yaw = ips_yaw - first_vec(2) + e(2);
  g_ips_yaw = floatMod(g_ips_yaw + M_PI, 2 * M_PI) - M_PI;
  g_abs_yaw = floatMod(g_abs_yaw + M_PI, 2 * M_PI) - M_PI;
  g_newIPS = true;
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
 void odom_callback(const nav_msgs::Odometry& msg)
 {
   std::lock_guard<std::mutex> lock(g_mutex);
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

void cmd_callback(const geometry_msgs::Twist& msg)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  g_Umat(0) = msg.linear.x;
  g_Umat(2) = msg.angular.z;
}

void scan_callback(const sensor_msgs::LaserScan& msg)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  auto diff = msg.header.stamp - g_poseTime;
  if (std::abs(diff.toSec() * 1e6) > 100000) {
    std::cout << "Scan / pose info too old skipping. " << std::abs(diff.toSec() * 1e6) << "\n";
    return;
  }
  ++g_newScanCnt;
  g_laser = msg;
  g_pose_scan_x = g_abs_x;
  g_pose_scan_y = g_abs_y;
  g_pose_scan_yaw = g_abs_yaw;
}

//
//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
  (void) msg;
  //This function is called when a new map is received

  //you probably want to save the map into a form which is easy to work with
}

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty
//	  vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y)
{
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
void measUpdate(MeasType type, Eigen::MatrixXd lastMean)
{
  Eigen::MatrixXd qMeas;
  Eigen::MatrixXd meas;
  qMeas.resizeLike(g_Qmat);
  meas.resizeLike(g_Meas);
  switch (type) {
    case MEAS_IPS:
      meas(0) = g_ips_x;
      meas(1) = g_ips_y;
      meas(2) = g_ips_yaw;
      qMeas = g_Qmat;
    break;
    case MEAS_ODOM:
      meas(0) = g_odom_x;
      meas(1) = g_odom_y;
      meas(2) = g_odom_yaw;
      auto diff = meas - g_lastOdom;
      g_lastOdom = meas;
      meas = lastMean + diff;
      meas(2) = floatMod(meas(2) + M_PI, 2 * M_PI) - M_PI;
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
}

//
// implements particle filtering.
void particleFilter()
{
  std::lock_guard<std::mutex> lock(g_mutex);
  //
  // Motion model.
  for (uint32_t part = 0; part < g_particles.cols(); ++part) {
    Eigen::MatrixXd e = g_Rmat.array().sqrt();
    e *= g_normMatGen.samples(1);
    auto partCol = g_particles.col(part);
    auto partPredCol = g_particlesPred.col(part);
    partPredCol(0) = partCol(0) + g_Umat(0) * std::cos(partCol(2)) * PERIOD;
    partPredCol(1) = partCol(1) + g_Umat(0) * std::sin(partCol(2)) * PERIOD;
    partPredCol(2) = partCol(2) + g_Umat(2) * PERIOD;
    partPredCol(2) = floatMod(partPredCol(2) + M_PI, 2 * M_PI) - M_PI;
    g_particlesPred.col(part) = partPredCol + e;
  }
  //
  // Collect the mean measurement for odometry.
  Eigen::MatrixXd lastMean = g_particles.rowwise().mean();
  //
  // Update using both sensors.
  if (g_newIPS) {
    measUpdate(MEAS_IPS, lastMean);
    g_newIPS = false;
  }
  if (g_newOdom) {
    g_particlesPred = g_particles;
    measUpdate(MEAS_ODOM, lastMean);
    g_newOdom = false;
  }
  Eigen::MatrixXd centered = g_particles.rowwise() - g_particles.colwise().mean();
  Eigen::MatrixXd cov = (centered.adjoint() * centered) / double(g_particles.rows() - 1);
}

//
// Implements OccupancyGrid visualization.
void visOcc()
{
  typedef Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix8u;
  nav_msgs::OccupancyGrid grid;
  grid.header.stamp = ros::Time::now();
  grid.header.frame_id = "/odom";
  grid.info = g_mapMeta;
  g_M = g_L.array().exp() / (1 + g_L.array().exp());
  auto prob = (g_M * 100.).cast<int8_t>();
  Eigen::Map<Matrix8u>(g_mapData, prob.rows(), prob.cols() ) = prob;
  grid.data.assign(g_mapData, g_mapData + prob.rows() * prob.cols());
  map_pub.publish(grid);
}

//
// The logit function.
inline double logit(double prob)
{
  return std::log(prob / (1-prob));
}

//
// Implements mapping.
void map()
{
  std::lock_guard<std::mutex> lock(g_mutex);
  //
  // wait until we get sufficiently new information.
  if (g_newScanCnt < 1) {
    return;
  }
  g_newScanCnt = 0;
  //
  // current position.
  double y_map = g_pose_scan_x - g_mapMeta.origin.position.x;
  double x_map = g_pose_scan_y - g_mapMeta.origin.position.y;
  uint32_t x_map_idx = x_map / g_gridResM;
  uint32_t y_map_idx = y_map / g_gridResM;
  std::cout << "pose yaw: " << g_pose_scan_yaw << " orientation yaw " << tf::getYaw(g_mapMeta.origin.orientation) << std::endl;
  double yaw_map = g_pose_scan_yaw - tf::getYaw(g_mapMeta.origin.orientation);
  std::cout << "yaw_map: " << yaw_map << std::endl;
  yaw_map = floatMod(yaw_map + M_PI, 2 * M_PI) - M_PI;
  double x_rel = 0;
  double y_rel = 0;
  double x_map_rel = 0;
  double y_map_rel = 0;
  double x_scan_map = 0;
  double y_scan_map = 0;
  uint32_t x_scan_idx = 0;
  uint32_t y_scan_idx = 0;
  double angle = 0;
  double range = 0;

  std::cout << "-------------------------- "  << std::endl;
  for (uint32_t idx = 0; idx < g_laser.ranges.size(); ++idx) {
    range = g_laser.ranges[idx];
    //
    // Check if measurement is invalid.
    // if (std::isnan(range) || range < g_laser.range_min || range > g_laser.range_max) { // may want to make it that these cases count as max dist to update.
    //   continue;
    // }
    //
    // Invalid measurement.
    if (std::isnan(range) || range < g_laser.range_min) { // may want to make it that these cases count as max dist to update.
      continue;
    }
    if (range > g_laser.range_max || range > g_maxR) {
      range = g_maxR;
    }
    std::vector<int> y_coords;
    std::vector<int> x_coords;
    angle = g_laser.angle_min + idx * g_laser.angle_increment;
    //
    // Polar coordinate to (x,y).
    x_rel = range * std::sin(angle);
    y_rel = range * std::cos(angle);
    //
    // Relative coordinates in the map frame.
    x_map_rel = x_rel * std::cos(-yaw_map) - y_rel * std::sin(-yaw_map);
    y_map_rel = x_rel * std::sin(-yaw_map) + y_rel * std::cos(-yaw_map);
    //
    // Map coordinates.
    x_scan_map = x_map + x_map_rel;
    y_scan_map = y_map + y_map_rel;
    //
    // Map index coordinates.
    x_scan_idx = x_scan_map / g_gridResM;
    y_scan_idx = y_scan_map / g_gridResM;
    bresenham(x_map_idx, y_map_idx, x_scan_idx, y_scan_idx, x_coords, y_coords);
    //
    // Iterate through ray traced map.
    // Do we ever update the same thing twice??
    for (uint32_t rayIdx = 0; rayIdx < x_coords.size() - g_mapBeta; ++rayIdx) {
      g_L(x_coords[rayIdx], y_coords[rayIdx]) = g_L(x_coords[rayIdx], y_coords[rayIdx]) - g_L0(x_coords[rayIdx], y_coords[rayIdx]) + logit(g_emptyProb);
    }
    //
    // Iterate through the last beta indicies and fill them.
    uint32_t rayIdx = x_coords.size() - g_mapBeta;
    if (abs(range - g_maxR) > 1e-3) { // did not read the max range.
      for (; rayIdx < x_coords.size(); ++rayIdx) {
        g_L(x_coords[rayIdx], y_coords[rayIdx]) = g_L(x_coords[rayIdx], y_coords[rayIdx]) - g_L0(x_coords[rayIdx], y_coords[rayIdx]) + logit(g_notEmptyProb);
      }
    }
    else {
      for (; rayIdx < x_coords.size(); ++rayIdx) {
        g_L(x_coords[rayIdx], y_coords[rayIdx]) = g_L(x_coords[rayIdx], y_coords[rayIdx]) - g_L0(x_coords[rayIdx], y_coords[rayIdx]) + logit(g_emptyProb);
      }
    }
  }
  // loop over all messages.
    // find angle of meas.
    // find xy components of measurement
    // add to the current location of the Robot
    // convert into grid coordinates
    // run the ray tracing algorithm
    // perform update over each of the xy coordinates
  // end for
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
  // ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback); // Gazebo
  ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callbackTbot);
  ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
  ros::Subscriber scan_sub = n.subscribe("/scan", 1, scan_callback);
  //
  // Setup topics to Publish from this node
  // ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("OccupancyGrid", 1, true);
  ros::Subscriber odom_sub = n.subscribe("/odom", 1, odom_callback);
  ros::Subscriber cmd_sub = n.subscribe("/mobile_base/commands/velocity", 1, odom_callback);
  //
  //Set the loop rate
  ros::Rate loop_rate(F);    //20Hz update rate
  //
  // Initialize dynamic reconfiguration.
  while (!sampled && !g_newOdom) {
    loop_rate.sleep(); //Maintain the loop rate
    ros::spinOnce();   //Check for new messages
  }
  g_newOdom = false;
  g_lastOdom(0) = g_odom_x;
  g_lastOdom(1) = g_odom_y;
  g_lastOdom(2) = g_odom_yaw;
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
    particleFilter();
    ros::spinOnce();
    map();
    viz.visualize_particle(g_particles.colwise()+first_vec.cast<double>()); //vel
    visOcc();
  }

  return 0;
}
