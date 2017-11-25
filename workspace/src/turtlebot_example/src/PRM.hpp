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
#include "cpp/turtlebot_example/lab3Config.h"
#include <random>
#include <cstdint>
#include "Graph.hpp"
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
    ros::NodeHandle
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
  // Handle.
  ros::NodeHandle m_n;
  //
  // Subscriber.
  ros::Subscriber m_mapSub;
  //
  // The map.
  MatrixXu m_map;
  //
  // The actual graph.
  Graph m_g;
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
  // The number of bins in one edge of the map.
  uint32_t m_nBins = 0;
  //
  // List of all samples.
  Eigen::MatrixXd m_samples;
  //
  // Random number stuff.
  std::random_device m_rd;  // Will be used to obtain a seed for the random number engine
  std::mt19937 m_gen; // Standard mersenne_twister_engine seeded with rd()
  std::uniform_real_distribution<> m_dis;
};
