#include <PRM.hpp>

///////////////////////////////////////////////////////////////
PRM::PRM(
  ros::NodeHandle n,
  dcoord start,
  std::vector<dcoord> goals,
  double ox,
  double oy
)
 : m_n(n)
 , m_mapSub(m_n.subscribe("/map", 1, &PRM::map_callback, this))
 , m_markPub(m_n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true))
 , m_trajPub(m_n.advertise<visualization_msgs::Marker>("traj", 2, true))
 , m_nodePub(m_n.advertise<geometry_msgs::PoseArray>("particle_pose_array", 10))
 , m_dstart(start)
 , m_dgoals(goals)
 , m_gen(m_rd())
 , m_dis(0., 1.)
 , offsetX(ox)
 , offsetY(oy)
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
  m_nSamples = 200;
  m_samples.resize(m_nSamples, 2);
  m_samples.setZero();
  m_cutoff_upper = config.cutoff_upper;
  m_cutoff_lower = config.cutoff_lower;
  #pragma message("REMOVE ME")
  m_cutoff_upper = 4;
  m_cutoff_lower = 0.1;
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
  expansion();
  m_res = msg.info.resolution;
  m_mapl = sz * m_res;
  m_nBins = sz;
  m_hasMap = true;
  coord tmp;
  //
  // Translate the goal locations from physical coordinates to map coordinates.
  for (auto & kv : m_dgoals) {
    tmp.first = kv.first / m_res;
    tmp.second = kv.second / m_res;
    m_goals.push_back(tmp);
  }

  m_start.first = m_dstart.first / m_res;
  m_start.second = m_dstart.second / m_res;

  m_g.setStartAndGoals(m_start, m_goals, sz, m_res);
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
  std::cout << "Building the map -- sample\n";
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
    x = kv.first;
    y = kv.second;
    PRINT_CORD(kv)
    if (x >= m_nBins || y >= m_nBins) {
      std::cout << "Likely passed Negative goal locations will fail now.\n";
    }
    if (m_map(x,y) != 0) {
      std::cerr << "\n\n\nMap location is in a boundary\n\n\n";
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
  std::cout << "Building the map -- dist\n";
  getDists(m_samples, dists);
  dists *= m_res;
  comp = (dists.array() < m_cutoff_upper).array() * (dists.array() > m_cutoff_lower).array();

  I.conservativeResize(std::stable_partition(I.data(), I.data()+I.size(), [&comp](int i){return comp(i);})-I.data());
  std::cout << "Building the map -- graph\n";
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
    if (collision(origin(0), origin(1), dest(0), dest(1)) /*&& !(o == m_start || d == m_start)*/) {
      continue;
    }
    cost = dists(x_idx, y_idx);
    //
    // The graph itself will deal with doubles, and the second connection happens since we have all values in both directions.
    m_g.addvertex(o);
    m_g.addvertex(d);
    m_g.addedge(o, d, cost);
  }
  rviz();

  m_paths = m_g.aStar();
  vizPath(m_paths);
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
      p.x = kv.first.first * m_res -offsetX;
      p.y = kv.first.second * m_res -offsetY;
      p2.x = vtx.second->loc.first * m_res -offsetX;
      p2.y = vtx.second->loc.second * m_res -offsetY;
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
  	pose.position.x = x * m_res -offsetX;
  	pose.position.y = y * m_res -offsetY;
  	pose.position.z = 0.0;
  	pose.orientation = tf::createQuaternionMsgFromYaw(0);
    particles.poses.push_back(pose);
  }
  m_nodePub.publish(particles);
}

///////////////////////////////////////////////////////////////
void PRM::vizPath(
  std::queue<coord> path
)
{
  coord top;
  geometry_msgs::Point p;
  visualization_msgs::Marker lines;
  visualization_msgs::Marker deleteMsg;
  p.z = 0; //not used
  //
  // Visualize all samples.
  vizNodes(m_samples);
  //
  // Delete all previous visualizations.
  deleteMsg.header.frame_id = "/map";
  deleteMsg.action = visualization_msgs::Marker::DELETEALL;
  m_trajPub.publish(deleteMsg);
  //
  // Visualize connections.
  lines.header.frame_id = "/map";
  lines.id = 100; //each curve must have a unique id or you will overwrite an old ones
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;
  lines.ns = "curves";
  lines.scale.x = 0.08;
  lines.color.r = 0.0;
  lines.color.b = 0.0;
  lines.color.g = 1.0;
  lines.color.a = 1.0;


  top = path.front();
  path.pop();
  p.x = top.first * m_res -offsetX;
  p.y = top.second * m_res -offsetY;
  lines.points.push_back(p);

  while (!path.empty()) {
    top = path.front();
    p.x = top.first * m_res -offsetX;
    p.y = top.second * m_res -offsetY;
    // p.x = top.first * m_res ;
    // p.y = top.second * m_res  ;
    lines.points.push_back(p);
    lines.points.push_back(p);
    path.pop();
  }
  lines.points.pop_back();

  std::cout << lines << std::endl;

  m_trajPub.publish(lines);

}

///////////////////////////////////////////////////////////////
void PRM::expansion()
{
  /*new_vector  = Eigen::VectorXu
  for (uint32_t i = 0; idx < sz; ++idx) {
    for (uint32_t i = 0; idx < sz; ++idx) {
      new_vector()
    }
  }
  m_expanded.publish();*/
}

///////////////////////////////////////////////////////////////
short sgn(int x) { return x >= 0 ? 1 : -1; }

bool PRM::collision(
  uint32_t x0,
  uint32_t y0,
  uint32_t x1,
  uint32_t y1
)
{
  bool collided = false;

  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  int dx2 = x1 - x0;
  int dy2 = y1 - y0;

  const bool s = abs(dy) > abs(dx);

  if (s) {
      int dx2 = dx;
      dx = dy;
      dy = dx2;
  }

  int inc1 = 2 * dy;
  int d = inc1 - dx;
  int inc2 = d - dx;

  while (x0 != x1 || y0 != y1) {
      if (s) y0+=sgn(dy2); else x0+=sgn(dx2);
      if (d < 0) d += inc1;
      else {
          d += inc2;
          if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
      }

      //Add point to vector
      if (m_map(x0,y0) !=0){
        collided = true;
        break;
      }
  }
  return collided;
}
