/* Local communication node for  turtlebot3 1 (tb3_1) in ROS and Gazebo simulation for implementing
Resource-Aware distributed Greedy (RAG) algorithm. The node from this code facilitates communication
from the base station node (rag_base_station) where it is able to subscribe to or publish to any topic.

Written by: Sandilya Sai Garimella March/19/2023 
Affiliation: University of Michigan Intelligent Robotics and Autonomy Lab (iRaL)
*/
#include "turtlebot3_communicator.hpp"
#include "utilfunctions.hpp"

class Tb3_1_Local_Comm : public Turtlebot3_Communicator {
  protected:
    // Tb3_1_Local_Comm(ros::NodeHandle& node_handle);
  void ProcessIncomingData(const nav_msgs::Odometry::ConstPtr &odom_msg, 
                                        const slam_karto_gtsam_landmark::OptimizationOutput::ConstPtr &optim_msg,
                                        const resource_aware_coordination::MarkerArrayHeader::ConstPtr &frontiers_msg);

  message_filters::Subscriber<nav_msgs::Odometry> tb3_1_subscriber1_;
  message_filters::Subscriber<slam_karto_gtsam_landmark::OptimizationOutput> tb3_1_optim_sub_;
  message_filters::Subscriber<resource_aware_coordination::MarkerArrayHeader> tb3_1_frontiers_sub_;

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, 
  slam_karto_gtsam_landmark::OptimizationOutput, resource_aware_coordination::MarkerArrayHeader> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  ros::Publisher frontier_array_publisher_;

  public:
    Tb3_1_Local_Comm(ros::NodeHandle &node_handle) : tb3_1_subscriber1_(node_handle, "/tb3_1/odom", 500),
                                                   tb3_1_optim_sub_(node_handle, "/tb3_1/optimized_graph_output", 500),
                                                   tb3_1_frontiers_sub_(node_handle, "/tb3_1_explore/frontiers", 500)
  {
    sync_.reset(new Sync(MySyncPolicy(10), tb3_1_subscriber1_, tb3_1_optim_sub_, tb3_1_frontiers_sub_));
    sync_->registerCallback(boost::bind(&Tb3_1_Local_Comm::ProcessIncomingData, this, _1, _2, _3));
    frontier_array_publisher_ = node_handle.advertise<visualization_msgs::MarkerArray>("/tb3_1/frontier_markers", 200);
  }

};

void Tb3_1_Local_Comm::ProcessIncomingData(const nav_msgs::Odometry::ConstPtr &odom_msg, 
                                        const slam_karto_gtsam_landmark::OptimizationOutput::ConstPtr &optim_msg,
                                        const resource_aware_coordination::MarkerArrayHeader::ConstPtr &frontiers_msg) {
  // de-serialize message using inherited function
  ROS_INFO("ProcessIncomingData CHECKPOINT: Received optim_msg");
  std::vector<visualization_msgs::Marker> frontiers_vis_points = frontiers_msg->markers;
  visualization_msgs::MarkerArray frontier_markers_msg;
  frontier_markers_msg.markers = frontiers_vis_points;
  frontier_array_publisher_.publish(frontier_markers_msg);
}


/*Tb3_1_Local_Comm::Tb3_1_Local_Comm(ros::NodeHandle& node_handle) : Turtlebot3_Communicator(node_handle) {
  // add any additional constructor code
}*/

int main(int argc, char **argv) {

  ros::init(argc, argv, "tb3_1_local_comm_node"); // node name

  ros::NodeHandle nh_tb3_1;

  Tb3_1_Local_Comm my_tb3(nh_tb3_1); // initialize inherited class

  ros::spin(); // ROS spin for callback function update
  return 0;
}