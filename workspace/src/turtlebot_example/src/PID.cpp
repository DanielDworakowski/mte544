#include "PID.hpp"

///////////////////////////////////////////////////////////////
PID::PID(float p, float i, float d, float max, float min)
  : m_p(p)
  , m_i(i)
  , m_d(d)
  , m_lastErr(0)
  , m_integralErr(0)
  , m_max(max)
  , m_min(min)
{
  m_lastTime = ros::Time::now();
}

///////////////////////////////////////////////////////////////
PID::~PID()
{
}

///////////////////////////////////////////////////////////////
float PID::getCmd(float ref, float meas)
{
  float err = ref - meas;
  ros::Time curTime = ros::Time::now();
  float tDiff = (curTime - m_lastTime).toSec();
  float cmd = 0;
  //
  // Prevent division by zero.
  if (tDiff <= 0.0) {
      return 0;
  }
  //
  // Integrate.
  m_integralErr += tDiff * m_i * err;
  //
  // Anti-windup.
  m_integralErr = std::min(std::max(m_integralErr, m_min), m_max);
  //
  // Calculate the PID and clamp.
  cmd = err * m_p + m_integralErr + m_d * (err - m_lastErr) / tDiff;
  cmd = std::min(std::max(cmd, m_min), m_max);
  m_lastErr = err;
  m_lastTime = curTime;
  return cmd;
}
