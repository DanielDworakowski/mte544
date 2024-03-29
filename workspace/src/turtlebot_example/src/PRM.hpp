#pragma once
//
// Standard.
#include <mutex>
#include <stdint.h>
//
// Ros.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include "Eigen/Dense"
#include <lab2Math.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include "cpp/turtlebot_example/lab3Config.h"
#include <random>
#include <cstdint>
#include "Graph.hpp"
#include <stack>
//
// Typedefs.
typedef Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXu;
//
// PRM class.
class PRM {
public:
  //
  // Constructor.
  PRM(
    ros::NodeHandle,
    dcoord start,
    std::vector<dcoord> goals,
    double ox,
    double oy
  );
  //
  // Destructor.
  virtual ~PRM (
  );
  //
  // Build the map.
  void buildMap (
  );
  //
  // Reconfigure the map.
  void reconfigure (
    turtlebot_example::lab3Config &config,
    uint32_t level
  );
  //
  // Publish map to rviz.
  void rviz(
  );
  //
  // Convert to real.
  coord convtToReal(coord c) {
    c.first *= m_res;
    c.second *= m_res;
    return c;
  }
  //
  // The path.
  std::queue<coord>  m_paths;

private:
  //
  // Read the map.
  void map_callback (
    const nav_msgs::OccupancyGrid& msg
  );
  //
  // Get a sample.
  void getSample (
    uint32_t & x,
    uint32_t & y
  );
  //
  // Build the graph.
  void buildGraph (
    Eigen::VectorXi & I,
    Eigen::MatrixXd & dists
  );
  //
  // Rviz the graph nodes.
  void vizNodes (
    Eigen::MatrixXd parts
  );
  //
  // Rviz the graph nodes.
  void vizPath (
    std::queue<coord> path
  );
  //
  // Map expansion
  void expansion (
  );
  //
  // Handle.
  ros::NodeHandle m_n;
  //
  // Subscriber.
  ros::Subscriber m_mapSub;
  //
  // Vis marker pub.
  ros::Publisher m_markPub;
  //
  // Vis traj pub.
  ros::Publisher m_trajPub;
  //
  // Node publisher.
  ros::Publisher m_nodePub;
  //
  // Expanded Map Publisher.
  ros::Publisher m_expanded;
  //
  // The map.
  MatrixXu m_map;
  //
  // Collision Detection between 2 points
  bool collision (
    uint32_t,
    uint32_t,
    uint32_t,
    uint32_t
  );
  //
  // Has a map.
  bool m_hasMap = false;
  //
  // Has been reconfigured.
  bool m_configured = false;
  //
  // Number of samples to make.
  uint32_t m_nSamples = 1;
  //
  // Physical size of the map.
  double m_mapl = 0;
  //
  // Resolution of the map.
  double m_res = 0;
  //
  // The cut off for distance linking.
  double m_cutoff_upper = 0;
  double m_cutoff_lower = 0;
  //
  // The number of bins in one edge of the map.
  uint32_t m_nBins = 0;
  //
  // List of all samples.
  Eigen::MatrixXd m_samples;
  //
  // The starting location.
  coord m_start;
  //
  // The list of goal locations.
  std::vector<coord> m_goals;
  //
  // double start
  dcoord m_dstart;
  //
  // The list of goal locations.
  std::vector<dcoord> m_dgoals;
  //
  // The actual graph.
  Graph m_g;
  //
  // Random number stuff.
  std::random_device m_rd;  // Will be used to obtain a seed for the random number engine
  std::mt19937 m_gen; // Standard mersenne_twister_engine seeded with rd()
  std::uniform_real_distribution<> m_dis;
  double offsetX;
  double offsetY;
};
