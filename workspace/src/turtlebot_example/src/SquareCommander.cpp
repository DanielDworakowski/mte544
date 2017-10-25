#include "SquareCommander.hpp"

///////////////////////////////////////////////////////////////
SquareCommander::SquareCommander(
  ros::NodeHandle n,
  double speed
)
  : m_n(n)
  , m_vel(n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1))
  // , m_moveResultSub(m_n.subscribe("/move_base/result", 1, &SquareCommander::moveBaseResultCallback, this))
  , m_crashSub(m_n.subscribe("/mobile_base/events/bumper", 1, &SquareCommander::crashCallback, this))
  , m_pose(n)
  , m_refSpeed(speed)
  , m_orientpid(0.5, 0, 0, M_PI / 2., -M_PI / 2.)
{
}

///////////////////////////////////////////////////////////////
SquareCommander::~SquareCommander(

)
{
}

///////////////////////////////////////////////////////////////
bool SquareCommander::squareManager(

)
{
  //
  // Move as required.
  bool stat = false;
  switch (m_state) {
    case SquareState::left:
      if (std::abs(m_poseData.y - m_startPose.y) < 1e-1) {
        m_state = SquareState::spin;
        m_stateNext = SquareState::bottom;
        m_refOrient = 0;
      }
      break;
    case SquareState::right:
      std::cout << "Pose y: " << m_poseData.y << " Starty " << m_startPose.y << " err: " << std::abs(m_poseData.y - m_startPose.y - m_goalDist) << std::endl;
      if (std::abs(m_poseData.y - m_startPose.y - m_goalDist) < 1e-1) {
        m_state = SquareState::spin;
        m_stateNext = SquareState::top;
        m_refOrient = 2.0 * M_PI / 2.0;
      }
      break;
    case SquareState::top:
      if (std::abs(m_poseData.x - m_startPose.x) < 1e-1) {
        m_state = SquareState::spin;
        m_stateNext = SquareState::left;
        m_refOrient = 3.0 * M_PI / 2.0;
      }
      break;
    case SquareState::spin:
      std::cout << "Spin state \n";
      std::cout << "yaw: " << m_poseData.yaw << std::endl;
      m_refSpeed = 0;
      if (std::abs(m_poseData.yaw - m_refOrient) < 1e-1) {
        m_state = m_stateNext;
        m_refSpeed = 0.2;
      }
      break;
    default:
    case SquareState::bottom:
    std::cout << "Pose x: " << m_poseData.x << " Startx " << m_startPose.x << std::endl;
      if (std::abs(m_poseData.x - m_startPose.x - m_goalDist) < 1e-1) {
        m_state = SquareState::spin;
        m_stateNext = SquareState::right;
        m_refOrient = M_PI / 2.0;
      }
      break;
    case SquareState::none:
      m_state = SquareState::spin;
      m_refOrient = 0;
      m_stateNext = SquareState::bottom;
      m_startPose = m_poseData;
      break;
  }
  return true;
}

///////////////////////////////////////////////////////////////
bool SquareCommander::step(

)
{
  if (!m_pose.getPose(&m_poseData)) {
    std::cerr << "Failed to get pose data.\n";
    return false;
  }
  if (!squareManager()) {
    std::cerr << "Failed to manage making the square.\n";
    return false;
  }
  double cmd = m_orientpid.getCmd(m_refOrient, m_poseData.yaw);
  geometry_msgs::Twist vel;
  vel.linear.x = m_refSpeed;
  vel.angular.z = cmd;
  m_vel.publish(vel);
  std::cout << "Linear: " << m_refSpeed << " Angular: " << cmd << std::endl;
  return true;
}

///////////////////////////////////////////////////////////////
bool SquareCommander::moveXY(
  double dx,
  double dy,
  double dyaw
)
{

}

///////////////////////////////////////////////////////////////
void SquareCommander::moveBaseResultCallback(
  const move_base_msgs::MoveBaseActionResult::ConstPtr & msg
)
{
}

///////////////////////////////////////////////////////////////
void SquareCommander::crashCallback(
  const kobuki_msgs::BumperEvent::ConstPtr & msg
)
{
  std::cout << "crash.\n";
}
