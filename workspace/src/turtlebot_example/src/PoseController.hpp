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
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <random>
#include <PID.hpp>

//
// PRM class.
class PoseController {
public:
  //
  // Constructor.
  PoseController(
     ros::NodeHandle
  );
  //
  // Destructor.
  ~PoseController (
  );
  //
  // Build the map.
  geometry_msgs::Twist get_vel (
    geometry_msgs::PoseWithCovarianceStamped,
    geometry_msgs::PoseWithCovarianceStamped
  );

private:
  //
  // Handle.
  ros::NodeHandle m_n;
  //
  // The PID for orientation.
  PID yaw_pid;
};
