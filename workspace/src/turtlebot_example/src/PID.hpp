#pragma once

#include <stdint.h>
#include <ros/time.h>

class PID {
  public:
    PID(float p, float i, float d, float max, float min);
    ~PID();
    //
    // Error function.
    float getCmd(float ref, float meas);

  private:
    float m_p, m_i, m_d;
    float m_lastErr;
    float m_integralErr;
    float m_max, m_min;
    ros::Time m_lastTime;
};
