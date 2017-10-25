#include <Pose.hpp>

///////////////////////////////////////////////////////////////
Pose::Pose(
  ros::NodeHandle n
)
 : m_n(n)
 , m_poseSub(m_n.subscribe("/amcl_pose", 1, &Pose::poseCallback, this))
{
}

///////////////////////////////////////////////////////////////
Pose::~Pose(

)
{
}

///////////////////////////////////////////////////////////////
void Pose::poseCallback(
  const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg
)
{
  std::lock_guard<std::mutex> g(m_mtx);
  //
  // Fill member data.
  m_p.x = msg->pose.pose.position.x;
  m_p.y = msg->pose.pose.position.y;
  m_p.yaw = tf::getYaw(msg->pose.pose.orientation);
  m_gotPose = true;
}

bool Pose::getPose(
  PoseData_t * pose
)
{
  std::lock_guard<std::mutex> g(m_mtx);
  if (pose != NULL) {
    *pose = m_p;
    return m_gotPose;
  }
  throw std::invalid_argument("Pose: NULL value.");
}
