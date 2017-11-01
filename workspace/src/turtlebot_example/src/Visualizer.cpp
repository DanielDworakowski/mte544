#include "Visualizer.hpp"

///////////////////////////////////////////////////////////////
Visualizer::Visualizer(
  ros::NodeHandle n
)
	: m_n(n)
  // , pose_sub(n.subscribe("/gazebo/model_states", 1, &Visualizer::viz_pose_callback,this))
  , pose_sub(n.subscribe("/odom", 1, &Visualizer::viz_pose_callbacktb,this))
	, particle_pub(n.advertise<geometry_msgs::PoseArray>("particle_pose_array", 10))
	, path_pub(n.advertise<geometry_msgs::PoseArray>("path_pose_array", 10))
  , path_odom_pub(n.advertise<geometry_msgs::PoseArray>("path_odom_pose_array", 10))
{
}

///////////////////////////////////////////////////////////////
Visualizer::~Visualizer(
)
{
}

///////////////////////////////////////////////////////////////
void Visualizer::visualize_particle(Eigen::MatrixXd particle_matrix)
{
	geometry_msgs::PoseArray particles;
	geometry_msgs::Pose pose;
	particles.header.stamp = ros::Time::now();
    particles.header.frame_id = "/odom";
    int num_particles = particle_matrix.cols();
    for (int i = 0; i < num_particles; ++i) {
    	double x = particle_matrix(0,i);
    	double y = particle_matrix(1,i);
    	double theta = particle_matrix(2,i);
    	pose.position.x = x;
    	pose.position.y = y;
    	pose.position.z = 0.0;
    	pose.orientation = tf::createQuaternionMsgFromYaw(theta);
        particles.poses.push_back(pose);
    }
    particle_pub.publish(particles);
}

void Visualizer::visualize_path(geometry_msgs::Pose pose)
{
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "/odom";
  path.poses.push_back(pose);
  path_pub.publish(path);

  // geometry_msgs::PoseArray particles;
  // geometry_msgs::Pose pose;
  // particles.header.stamp = ros::Time::now();
  //   particles.header.frame_id = "/odom";
  //   int num_particles = particle_matrix.cols();
  //   for (int i = 0; i < num_particles; ++i) {
  //     double x = particle_matrix(0,i);
  //     double y = particle_matrix(1,i);
  //     double theta = particle_matrix(2,i);
  //     pose.position.x = x;
  //     pose.position.y = y;
  //     pose.position.z = 0.0;
  //     pose.orientation = tf::createQuaternionMsgFromYaw(theta);
  //       particles.poses.push_back(pose);
  //   }
  //   particle_pub.publish(particles);
}

void Visualizer::viz_pose_callback(const gazebo_msgs::ModelStates& msg)
{
	uint32_t i;
    for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;
	path.header.stamp = ros::Time::now();
    path.header.frame_id = "/odom";
	path.poses.push_back(msg.pose[i]);
    path_pub.publish(path);
}

void Visualizer::viz_pose_callbacktb(const nav_msgs::Odometry& msg)
{
  #pragma message("fix me")
  path_odom.header.stamp = ros::Time::now();
  path_odom.header.frame_id = "/odom";
  path_odom.poses.push_back(msg.pose.pose);
  path_odom_pub.publish(path_odom);

	// uint32_t i;
  //   for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;
	// path.header.stamp = ros::Time::now();
  //   path.header.frame_id = "/odom";
	// path.poses.push_back(msg.pose[i]);
  //   path_pub.publish(path);
}

// int main(int argc, char **argv)
// {
// 	//Initialize the ROS framework
//     ros::init(argc,argv,"main_control");
//     ros::NodeHandle n;
//    	Visualizer viz = Visualizer(n);
//     ros::Rate loop_rate(20);    //20Hz update rate

//     while (ros::ok())
//     {
//     	loop_rate.sleep(); //Maintain the loop rate
//     	ros::spinOnce();   //Check for new messages
//     	Eigen::MatrixXd g_particles = Eigen::MatrixXd::Random(3, 20);
//     	viz.visualize_particle(g_particles);
//     }
//     return 0;
// }
