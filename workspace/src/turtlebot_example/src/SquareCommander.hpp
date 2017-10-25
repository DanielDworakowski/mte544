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
#include <PID.hpp>
//
// What state are we in in drawing the square.
enum class SquareState{
  bottom,
  left,
  right,
  top,
  spin,
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
    // Decideds how a square is meant to be made.
    bool squareManager();
    //
    // Node handle.
    ros::NodeHandle m_n;
    //
    // Publisher for pose goals.
    ros::Publisher m_vel;
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
    double m_refSpeed = 0.2;
    double m_goalDist = 1;
    //
    // Current state.
    SquareState m_state = SquareState::none;
    SquareState m_stateNext = SquareState::bottom;
    //
    // The start of the manuever.
    PoseData_t m_startPose;
    //
    // Pose data for work.
    PoseData_t m_poseData;
    //
    // The PID for orientation.
    PID m_orientpid;
    //
    // The reference orientation.
    double m_refOrient = 0;
};

// /move_base/result
// /mobile_base/events/bumper
// -> state 1 is a crash
