#include <PRM.hpp>

///////////////////////////////////////////////////////////////
PRM::PRM(
  ros::NodeHandle n,
  coord start,
  std::vector<coord> goals
)
 : m_n(n)
 , m_mapSub(m_n.subscribe("/map", 1, &PRM::map_callback, this))
 , m_markPub(m_n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true))
 , m_nodePub(m_n.advertise<geometry_msgs::PoseArray>("particle_pose_array", 10))
 , m_start(start)
 , m_goals(goals)
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
  m_nSamples = 1000;
  m_samples.resize(m_nSamples, 2);
  m_samples.setZero();
  m_cutoff = config.cutoff;
  #pragma message("REMOVE ME")
  m_cutoff = 1;
  m_configured = true;
}

///////////////////////////////////////////////////////////////
void PRM::map_callback(
  const nav_msgs::OccupancyGrid & msg
)
{
  uint32_t sz = std::sqrt(msg.data.size());
  m_map = Eigen::Map<const MatrixXu>(msg.data.data(), sz, sz);
  m_map.transposeInPlace();
  m_res = msg.info.resolution;
  std::cout << "info: " << msg.info << std::endl;
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
  //
  // Place the goal locations.
  std::vector<coord> tmpGoals = m_goals;
  tmpGoals.push_back(m_start);
  for (auto & kv : tmpGoals) {
    x = kv.first / m_res;
    y = kv.second / m_res;
    if (x >= m_nBins || y >= m_nBins) {
      std::cout << "Likely passed Negative goal locations will fail now.\n";
    }
    sampled(x, y) = 1;
    sample(0) = x;
    sample(1) = y;
    m_samples.row(nSampled) = sample.transpose();
    ++nSampled;
  }
  //
  // Start random sampling.
  while (nSampled < m_nSamples) {
    getSample(x, y);
    //
    // Check if point has already been sampled or if it is in an obstacle.
    if (m_map(x, y) != 0 || sampled(x, y) != 0) {
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
  comp = dists.array() < m_cutoff;
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
    if (x_idx == y_idx || dists(x_idx, y_idx) < 0.0001) {
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
  // PRINT_MATRIX(m_samples)
  // m_g.print();
}

///////////////////////////////////////////////////////////////
void PRM::rviz(
)
{
  uint32_t cnt = 0;
  geometry_msgs::Point p;
  geometry_msgs::Point p2;
  visualization_msgs::Marker lines;
  visualization_msgs::Marker deleteMsg;
  p.z = 0; //not used
  p2.z = 0; //not used
  //
  // Visualize all samples.
  vizNodes(m_samples);
  //
  // Delete all previous visualizations.
  deleteMsg.header.frame_id = "/map";
  deleteMsg.action = visualization_msgs::Marker::DELETEALL;
  m_markPub.publish(deleteMsg);
  //
  // Visualize connections.
  lines.header.frame_id = "/map";
  lines.id = ++cnt; //each curve must have a unique id or you will overwrite an old ones
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;
  lines.ns = "curves";
  lines.scale.x = 0.01;
  lines.color.r = 0.2;
  lines.color.b = 1.;
  lines.color.a = 1.0;
  for (auto & kv : m_g.m_vtx) {
    for (auto & vtx : kv.second->adj) {
      p.x = kv.first.first * m_res;
      p.y = kv.first.second * m_res;
      p2.x = vtx.second->loc.first * m_res;
      p2.y = vtx.second->loc.second * m_res;
      lines.points.push_back(p);
      lines.points.push_back(p2);
    }
  }
  m_markPub.publish(lines);
}

///////////////////////////////////////////////////////////////
void PRM::vizNodes(
  Eigen::MatrixXd particle_matrix
)
{
	geometry_msgs::PoseArray particles;
	geometry_msgs::Pose pose;
	particles.header.stamp = ros::Time::now();
  particles.header.frame_id = "/map";
  int num_particles = particle_matrix.rows();
  for (int i = 0; i < num_particles; ++i) {
  	double x = particle_matrix(i, 0);
  	double y = particle_matrix(i, 1);
  	pose.position.x = x * m_res;
  	pose.position.y = y * m_res;
  	pose.position.z = 0.0;
  	pose.orientation = tf::createQuaternionMsgFromYaw(0);
    particles.poses.push_back(pose);
  }
  m_nodePub.publish(particles);
}
