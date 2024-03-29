#pragma once
//
// Standard Headers.
#include <stdint.h>
//
// Ros headers.
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include "Eigen/Dense"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "nav_msgs/Odometry.h"


//
// Control the square path.
class Visualizer {
  public:
    Visualizer(ros::NodeHandle);
    ~Visualizer();
    //
    // Take a step at moving in a square.
    void visualize_particle(Eigen::MatrixXd);
    void visualize_path(geometry_msgs::Pose);
  private:
    void viz_pose_callback(const gazebo_msgs::ModelStates&);
    void viz_pose_callbacktb(const nav_msgs::Odometry&);
    //
    // Node handle.
    ros::NodeHandle m_n;
    //
    // Publisher for pose goals.
    ros::Subscriber pose_sub;
    ros::Publisher particle_pub;
    ros::Publisher path_pub;
    ros::Publisher path_odom_pub;
    geometry_msgs::PoseArray path;
    geometry_msgs::PoseArray path_odom;
};

// /move_base/result
// /mobile_base/events/bumper
// -> state 1 is a crash
