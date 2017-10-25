#pragma once
//
// Standard.
#include <mutex>
#include <stdint.h>
//
// Ros.
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
//
// Pose data.
typedef struct {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
} PoseData_t;
//
// Pose class.
class Pose {
public:
  //
  // Constructor.
  Pose(ros::NodeHandle);
  //
  // Destructor.
  virtual ~Pose();
  //
  // Get fn.
  bool getPose(PoseData_t *);
private:
  //
  // Handle.
  ros::NodeHandle m_n;
  //
  // Subscriber.
  ros::Subscriber m_poseSub;
  //
  // The pose callback.
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  //
  // Values.
  PoseData_t m_p;
  //
  // Guard for updates.
  std::mutex m_mtx;
  //
  // Have gotten a pose.
  bool m_gotPose = false;
};
