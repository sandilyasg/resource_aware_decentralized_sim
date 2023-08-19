#include "turtlebot3_communicator.hpp"
#include "utilfunctions.hpp"

// class Turtlebot3_Communicator constructor defined here
Turtlebot3_Communicator::Turtlebot3_Communicator(ros::NodeHandle &node_handle) {
  node_handle_ = node_handle;

  // Initialize subscriber that is set to the channel "/tb3_0/odom". Set callback function to be OdomModifier_callback
  subscriber1_topicname_ = "/tb3_0/odom";
  tb3_0_subscriber1_ = node_handle_.subscribe(subscriber1_topicname_, 100, &Turtlebot3_Communicator::OdomModifier_callback, this);

  //Initialize a publisher that is set to the channel "/tb3_0/odom_modified"
  publisher1_topicname_ = "/tb3_0/odom_modified";
  tb3_0_publisher1_ = node_handle_.advertise<nav_msgs::Odometry>(publisher1_topicname_, 100);
}

double Turtlebot3_Communicator::calculateInformationGain(const nav_msgs::Odometry::ConstPtr& robotPose, double& linVel, double& angVel, double& timeStep, 
  double& radius, const resource_aware_coordination::MarkerArrayHeader::ConstPtr& frontiers) {

    ROS_INFO("CHECKPOINT 3 ");
    // calculate the potential robot pose after applying the given linear and angular velocities for the given time step
    double x = robotPose->pose.pose.position.x + linVel * cos(robotPose->pose.pose.orientation.z) * timeStep;
    double y = robotPose->pose.pose.position.y + linVel * sin(robotPose->pose.pose.orientation.z) * timeStep;
    double theta = robotPose->pose.pose.orientation.z + angVel * timeStep;

    nav_msgs::Odometry newRobotPose = *robotPose; // create a copy of the robot pose to modify
    newRobotPose.pose.pose.position.x = x;
    newRobotPose.pose.pose.position.y = y;
    newRobotPose.pose.pose.orientation.z = theta;
    
    double covered = 0.0;
    // iterate over all frontiers and check if they would be covered by the robot at the potential pose
    resource_aware_coordination::MarkerArrayHeader newFrontiers = *frontiers;

    for (size_t i = 0; i < newFrontiers.markers.size(); i++) {
    auto& frontier = newFrontiers.markers[i];

        for (size_t j = 0; j < frontier.points.size(); j++) {
            double distance = euclideanDistance(frontier.points[j], newRobotPose.pose.pose.position);
            if (distance <= radius) {
                covered++;
                /*frontier.colors[j].r = 1.0; // set color to red
                frontier.colors[j].g = 0.0;
                frontier.colors[j].b = 0.0;*/
            }
        }
    }
    ROS_INFO("Covered: %f", covered);
    return covered; // and also the new visualization_msgs::MarkerArray& frontiers
}   


void Turtlebot3_Communicator::OdomModifier_callback(const nav_msgs::Odometry::ConstPtr &odom_msg_data) {

  /* read the incoming odom message, create a copy, modify the copy, 
  publish the modified copy over the /tb3_0/odom_modified topic*/

  const std::chrono::_V2::system_clock::time_point t0 = std::chrono::high_resolution_clock::now();
  nav_msgs::Odometry odom_msg_modified;
  geometry_msgs::Point odom_in_position = odom_msg_data->pose.pose.position;

  odom_msg_modified.pose.pose.position.x = odom_in_position.x + 1.;
  odom_msg_modified.pose.pose.position.y = odom_in_position.y + 1.;
  odom_msg_modified.pose.pose.position.z = odom_in_position.z + 1.;

  // // Introduce a 500 ms delay
  // ros::Duration(0.5).sleep();

  // // Introduce a 50 ms delay
  double comm_delay = 0.05;
  ros::Duration(comm_delay).sleep();

  tb3_0_publisher1_.publish(odom_msg_modified);

  const std::chrono::_V2::system_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> t_callback = t1 - t0;
  ROS_INFO("callback processing time: %f ms", t_callback.count());

}


void Turtlebot3_Communicator::deserializeOptimizedMsg(const slam_karto_gtsam_landmark::OptimizationOutput::ConstPtr &optim_msg, 
std::vector<Eigen::Matrix<double, 3, 3>> &pose_covariances, 
std::vector<Eigen::Matrix<double, 2, 2>> &mapfeature_covariances,
std::vector<int> &observed_keyids,
std::vector<Eigen::Vector3d> &robot_poses, 
std::vector<Eigen::Vector2d> &mapfeature_points)
 {
/*Header header
float64[] poses
float64[] pose_covariances
float64[] points
float64[] point_covariances
int32[] keys_poses
int32[] keys_points
int32[] keys_observed_points*/



}