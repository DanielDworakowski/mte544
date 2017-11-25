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
  m_nSamples = 100;
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
  Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> comp(m_nSamples, m_nSamples);
  Eigen::VectorXi I = Eigen::VectorXi::LinSpaced(comp.size(),0,comp.size()-1);
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
    sample(0) = x;
    sample(1) = y;
    m_samples.row(nSampled) = sample.transpose();
    ++nSampled;
  }
  getDists(m_samples, dists);
  dists *= m_res;
  comp = dists.array() < 4.;
  I.conservativeResize(std::stable_partition(I.data(), I.data()+I.size(), [&comp](int i){return comp(i);})-I.data());
  //
  // Now that all of the sample points have been created, build the graph.
  buildGraph(I, dists);
}

///////////////////////////////////////////////////////////////
void PRM::buildGraph(
  Eigen::VectorXi & I,
  Eigen::MatrixXd & dists

)
{
  // col by modulo, row by division
  uint32_t x_idx = 0, y_idx = 0;
  Eigen::VectorXd origin, dest;
  coord o, d;
  double cost;
  //
  // Iterate over all values who have distances less than x meters.
  for (uint32_t idx = 0; idx < I.size(); ++idx) {
    x_idx = I(idx) / m_nSamples;
    y_idx = I(idx) % m_nSamples;
    //
    // No need to link with itself.
    if (x_idx == y_idx) {
      continue;
    }
    origin = m_samples.row(x_idx);
    dest = m_samples.row(y_idx);
    o = std::make_pair(origin(0), origin(1));
    d = std::make_pair(dest(0), dest(1));
    cost = dists(x_idx, y_idx);
    //
    // The graph itself will deal with doubles, and the second connection happens since we have all values in both directions.
    m_g.addvertex(o);
    m_g.addvertex(d);
    m_g.addedge(o, d, cost);
  }
  m_g.print();
}
