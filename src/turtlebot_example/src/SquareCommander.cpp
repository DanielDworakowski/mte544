#include "SquareCommander.hpp"

///////////////////////////////////////////////////////////////
SquareCommander::SquareCommander(
  ros::NodeHandle n,
  double sl
)
  : m_n(n)
  , m_poseGoal(n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1))
  // , m_moveResultSub(m_n.subscribe("/move_base/result", 1, &SquareCommander::moveBaseResultCallback, this))
  , m_crashSub(m_n.subscribe("/mobile_base/events/bumper", 1, &SquareCommander::crashCallback, this))
  , m_pose(n)
  , m_sl(sl)
{
}

///////////////////////////////////////////////////////////////
SquareCommander::~SquareCommander(

)
{
}

///////////////////////////////////////////////////////////////
bool SquareCommander::step(

)
{
  PoseData_t data;
  if (!m_pose.getPose(&data)) {
    return false;
  }
  //
  // Check if we have crashed.
  //
  // if (crashed)
  //
  //
  // Move as required.
  bool stat = false;
  switch (m_state) {
    case SquareState::left:
      stat = moveXY(0, -m_sl, M_PI / 2.);
      break;
    case SquareState::right:
      stat = moveXY(0, m_sl, M_PI / 2.);
      break;
    case SquareState::top:
      stat = moveXY(-m_sl, 0, M_PI / 2.);
      break;
    default:
    case SquareState::bottom:
    case SquareState::none:
      std::cout << "Beginning to move.\n";
      stat = moveXY(m_sl, 0, M_PI / 2.);
      break;
  }
  m_working = true;
  return true;
}

///////////////////////////////////////////////////////////////
bool SquareCommander::moveXY(
  double dx,
  double dy,
  double dyaw
)
{
  PoseData_t data;
  if (!m_pose.getPose(&data)) {
    std::cerr << "Could not gain pose data.\n";
    return false;
  }
  data.x += dx;
  data.y += dy;
  data.yaw += dyaw;
  //
  // Create the ros msg.
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = ros::Time();
  goal.pose.position.x = data.x;
  goal.pose.position.y = data.y;
  auto q1 = tf::createQuaternionFromYaw(data.yaw);
  tf::quaternionTFToMsg(q1, goal.pose.orientation);
  m_poseGoal.publish(goal);
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
