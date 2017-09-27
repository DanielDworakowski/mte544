#pragma once
//
// Standard Headers.
#include <stdint.h>
//
// Ros headers.
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <kobuki_msgs/BumperEvent.h>
#include <tf/transform_datatypes.h>
//
// Our headers.
#include <Pose.hpp>
//
// What state are we in in drawing the square.
enum class SquareState{
  bottom,
  left,
  right,
  top,
  none
};
//
// Control the square path.
class SquareCommander {
  public:
    SquareCommander(ros::NodeHandle, double);
    ~SquareCommander();
    //
    // Take a step at moving in a square.
    bool step();
    //
    // Send a move command.
    bool moveXY(double, double, double);
  private:
    //
    // callbacks.
    void moveBaseResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr &);
    void crashCallback(const kobuki_msgs::BumperEvent::ConstPtr &);
    //
    // Node handle.
    ros::NodeHandle m_n;
    //
    // Publisher for pose goals.
    ros::Publisher m_poseGoal;
    //
    // Move result subscriber.
    ros::Subscriber m_moveResultSub;
    //
    // Crash subscriber.
    ros::Subscriber m_crashSub;
    //
    // Gets pose estimations.
    Pose m_pose;
    //
    // Side length.
    double m_sl = 0.;
    //
    // Current state.
    SquareState m_state = SquareState::none;
};

// /move_base/result
// /mobile_base/events/bumper
// -> state 1 is a crash
