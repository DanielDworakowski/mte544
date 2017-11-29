#include <PoseController.hpp>

///////////////////////////////////////////////////////////////
PoseController::PoseController(
  ros::NodeHandle n
)
 : m_n(n)
 , yaw_pid(0.5, 0, 0, 0.1, -0.1)
{
}

///////////////////////////////////////////////////////////////
PoseController::~PoseController(

)
{
}

///////////////////////////////////////////////////////////////
geometry_msgs::Twist PoseController::get_vel(
  geometry_msgs::PoseWithCovarianceStamped pose_meas,
  geometry_msgs::PoseWithCovarianceStamped pose_ref
)
{
  double yaw_meas = tf::getYaw(pose_meas.pose.pose.orientation);
  double x_meas = pose_meas.pose.pose.position.x; // Robot X position
  double y_meas = pose_meas.pose.pose.position.y; // Robot Y position

  double x_ref = pose_ref.pose.pose.position.x; // Robot X position
  double y_ref = pose_ref.pose.pose.position.y; // Robot Y position

  double y_diff = y_ref - y_meas;
  double x_diff = x_ref - x_meas;
  double dist = std::sqrt(std::pow(y_diff,2) + std::pow(x_diff,2));

  double yaw_ratio_ref = x_diff/dist;
  double sign = -yaw_meas/std::abs(yaw_meas);
  double yaw_ratio_meas = std::cos(yaw_meas);

  double yaw_error = std::abs(yaw_ratio_ref - yaw_ratio_meas);

  //
  // Velocity control variable
  geometry_msgs::Twist vel;

  if (yaw_error < 0.1 && std::abs(dist)>0.2)
  {
    vel.linear.x = 0.2; // set linear speed
  }
  else
  {
    vel.linear.x = 0.0;
  }
  vel.angular.z = sign*0.5*(yaw_ratio_ref - yaw_ratio_meas); // set angular speed
  if (vel.angular.z > 0.1)
    vel.angular.z = 0.1;
  if (vel.angular.z < -0.1)
    vel.angular.z = -0.1;
  // vel.angular.z = sign*yaw_pid.getCmd(yaw_ratio_ref, yaw_ratio_meas); // set angular speed
  if((y_diff>0 && yaw_meas<0))
  {
    vel.angular.z = -0.1;
    vel.linear.x = 0.0;
  }
  else if((y_diff<0 && yaw_meas>0))
  {
    vel.angular.z = 0.5;
    vel.linear.x = 0.0;
  }

  // std::cout << "----------------" << std::endl;
  // std::cout << "x_meas: " << x_meas << std::endl;
  // std::cout << "x_ref: " << x_ref << std::endl;
  // std::cout << "y_meas: " << y_meas << std::endl;
  // std::cout << "y_ref: " << y_ref << std::endl;
  // std::cout << "yaw_ratio_ref: " << yaw_ratio_ref << std::endl;
  // std::cout << "yaw_ratio_meas: " << yaw_ratio_meas << std::endl;
  // std::cout << "yaw_meas: " << yaw_meas << std::endl;
  // std::cout << "dist: " << std::abs(dist) << std::endl;
  // std::cout << "yaw_error: " << yaw_error << std::endl;
  // std::cout << "yaw_u: " << vel.angular.z << std::endl;
  // std::cout << "x_u: " << vel.linear.x << std::endl;

  return vel;
}
