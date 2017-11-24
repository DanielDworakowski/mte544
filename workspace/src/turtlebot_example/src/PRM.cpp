#include <PRM.hpp>

///////////////////////////////////////////////////////////////
PRM::PRM(
  ros::NodeHandle n
)
 : m_n(n)
 , m_mapSub(m_n.subscribe("/map", 1, &PRM::map_callback, this))
 , m_gen(m_rd())
 , m_dis(0., 1.)
{
  //
  // Set the loop rate
  ros::Rate loop_rate(20);    //20Hz update rate
  while (!m_hasMap){
    std::cout << "Waiting for map\n";
    loop_rate.sleep(); //Maintain the loop rate
    ros::spinOnce();   //Check for new messages
  }
}

///////////////////////////////////////////////////////////////
PRM::~PRM(

)
{
}

///////////////////////////////////////////////////////////////
void PRM::reconfigure(
  turtlebot_example::lab3Config &config,
  uint32_t level
)
{
  (void) level;
  m_nSamples = config.nSamples;
  m_nSamples = 10;
  m_samples.resize(m_nSamples, 2);
  m_samples.setZero();
  m_configured = true;
}

///////////////////////////////////////////////////////////////
void PRM::map_callback(
  const nav_msgs::OccupancyGrid& msg
)
{
  uint32_t sz = std::sqrt(msg.data.size());
  m_map = Eigen::Map<const MatrixXu>(msg.data.data(), sz, sz);
  m_res = msg.info.resolution;
  m_mapl = sz * m_res;
  m_nBins = sz;
  m_hasMap = true;
}

///////////////////////////////////////////////////////////////
void PRM::getSample(
  uint32_t & x,
  uint32_t & y
)
{
  x = m_dis(m_gen) * m_nBins;
  y = m_dis(m_gen) * m_nBins;
}

///////////////////////////////////////////////////////////////
void PRM::buildMap(

)
{
  MatrixXu sampled;
  Eigen::VectorXd sample(2);
  Eigen::MatrixXd dists(m_nSamples, m_nSamples);
  sampled.resizeLike(m_map);
  sampled.setZero();
  uint32_t nSampled = 0;
  uint32_t x = 0;
  uint32_t y = 0;
  while (nSampled < m_nSamples) {
    getSample(x, y);
    //
    // Check if point has already been sampled or if it is in an obstacle.
    if (m_map(x, y) == 1 || sampled(x, y) == 1) {
      continue;
    }
    //
    // Update the sample and place it into the matrix.
    sampled(x, y) = 1;
    sample(0) = x * m_res;
    sample(1) = y * m_res;
    m_samples.row(nSampled) = sample.transpose();
    ++nSampled;
  }
  getDists(m_samples, dists);
  PRINT_MATRIX(dists);
}
