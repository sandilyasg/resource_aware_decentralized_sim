/* Local communication node for  turtlebot3 0 (tb3_0) in ROS and Gazebo simulation for implementing
Resource-Aware distributed Greedy (RAG) algorithm. The node from this code facilitates communication
from the base station node (rag_base_station) where it is able to subscribe to or publish to any topic.

Written by: Sandilya Sai Garimella March/19/2023
Affiliation: University of Michigan Intelligent Robotics and Autonomy Lab (iRaL)
*/
#include "turtlebot3_communicator.hpp"
#include "utilfunctions.hpp"
// #include "message_filters/subscriber.h"
// #include "message_filters/time_synchronizer.h"
// #include <message_filters/sync_policies/approximate_time.h>
// #include "tf/message_filter.h"
// #include "tf/transform_listener.h"
// #include "filters/filter_chain.h"
// #include "resource_aware_coordination/MarkerArrayHeader.h"

class Tb3_0_Local_Comm : public Turtlebot3_Communicator
{

protected:
  // void ProcessIncomingData(const nav_msgs::Odometry::ConstPtr &odom_msg, 
  //                       const slam_karto_gtsam_landmark::OptimizationOutput::ConstPtr &optim_msg,
  //                       const resource_aware_coordination::MarkerArrayHeader::ConstPtr &frontiers_msg);

  void ProcessIncomingData(const nav_msgs::Odometry::ConstPtr &odom_msg, 
                                        const slam_karto_gtsam_landmark::OptimizationOutput::ConstPtr &optim_msg,
                                        const resource_aware_coordination::MarkerArrayHeader::ConstPtr &frontiers_msg);

  void InfogainMarkerCallback(const nav_msgs::Odometry::ConstPtr &odom1,
                              const resource_aware_coordination::MarkerArrayHeader::ConstPtr &markers_msg,
                              const nav_msgs::OccupancyGrid::ConstPtr &ogm_msg);

  message_filters::Subscriber<nav_msgs::Odometry> tb3_0_subscriber1_;
  // message_filters::Subscriber<resource_aware_coordination::MarkerArrayHeader> tb3_0_subscriber2_;
  // message_filters::Subscriber<nav_msgs::OccupancyGrid> tb3_0_subscriber3_;

  message_filters::Subscriber<slam_karto_gtsam_landmark::OptimizationOutput> tb3_0_optim_sub_;
  message_filters::Subscriber<resource_aware_coordination::MarkerArrayHeader> tb3_0_frontiers_sub_;

  // typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,
  //                                                         resource_aware_coordination::MarkerArrayHeader, nav_msgs::OccupancyGrid>
  //     MySyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, 
  slam_karto_gtsam_landmark::OptimizationOutput, resource_aware_coordination::MarkerArrayHeader> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  ros::Publisher frontier_array_publisher_;

public:
  /*Tb3_0_Local_Comm(ros::NodeHandle &node_handle) : tb3_0_subscriber1_(node_handle, "/tb3_0/odom", 500),
                                                   tb3_0_subscriber2_(node_handle, "/globalmap_fronter_explore/frontiers", 500),
                                                   tb3_0_subscriber3_(node_handle, "/map", 500)
  {
    sync_.reset(new Sync(MySyncPolicy(10), tb3_0_subscriber1_, tb3_0_subscriber2_, tb3_0_subscriber3_));
    sync_->registerCallback(boost::bind(&Tb3_0_Local_Comm::InfogainMarkerCallback, this, _1, _2, _3));
    // note the publisher actually publishes the markerarray that rviz needs
    tb3_0_publisher1_ = node_handle.advertise<visualization_msgs::MarkerArray>("covered_frontiers", 500);
  }*/

    Tb3_0_Local_Comm(ros::NodeHandle &node_handle) : tb3_0_subscriber1_(node_handle, "/tb3_0/odom", 500),
                                                   tb3_0_optim_sub_(node_handle, "/tb3_0/optimized_graph_output", 500),
                                                   tb3_0_frontiers_sub_(node_handle, "/tb3_0_explore/frontiers", 500) {
    sync_.reset(new Sync(MySyncPolicy(50), tb3_0_subscriber1_, tb3_0_optim_sub_,tb3_0_frontiers_sub_));
    sync_->registerCallback(boost::bind(&Tb3_0_Local_Comm::ProcessIncomingData, this, _1, _2, _3));
    frontier_array_publisher_ = node_handle.advertise<visualization_msgs::MarkerArray>("/tb3_0/frontier_markers", 200);
  }
};

void Tb3_0_Local_Comm::ProcessIncomingData(const nav_msgs::Odometry::ConstPtr &odom_msg, 
                                        const slam_karto_gtsam_landmark::OptimizationOutput::ConstPtr &optim_msg,
                                        const resource_aware_coordination::MarkerArrayHeader::ConstPtr &frontiers_msg) {
  // de-serialize message using inherited function
  ROS_INFO("ProcessIncomingData CHECKPOINT: Received optim_msg");
  std::vector<visualization_msgs::Marker> frontiers_vis_points = frontiers_msg->markers;
  // visualization_msgs::MarkerArray frontiers_vis_points = frontiers_msg->markers;
  visualization_msgs::MarkerArray frontier_markers_msg;
  frontier_markers_msg.markers = frontiers_vis_points;
  frontier_array_publisher_.publish(frontier_markers_msg);

}


void Tb3_0_Local_Comm::InfogainMarkerCallback(const nav_msgs::Odometry::ConstPtr &odom1,
                                              const resource_aware_coordination::MarkerArrayHeader::ConstPtr &markers_msg,
                                              const nav_msgs::OccupancyGrid::ConstPtr &ogm_msg)
{

  /*ROS_INFO("CHECKPOINT 1 ");
  ROS_INFO_THROTTLE(0.1, "Resolution: %f", ogm_msg->info.resolution);
  ROS_INFO_THROTTLE(0.1, "Width: %d", ogm_msg->info.width);
  ROS_INFO_THROTTLE(0.1,"Height: %d", ogm_msg->info.height);
  ROS_INFO_THROTTLE(0.5,"Origin X: %f", ogm_msg->info.origin.position.x);
  ROS_INFO_THROTTLE(0.5,"Origin Y: %f", ogm_msg->info.origin.position.y);*/
  int nonbincount = 0;
  for (int i = 0; i < ogm_msg->data.size(); i++)
  {
    // if (ogm_msg->data[i] != -1 || ) {
    //   ROS_INFO("Value at index %d: %d", i, ogm_msg->data[i]);
    // }
    if (ogm_msg->data[i] >= 0 && ogm_msg->data[i] <= 100)
    {
      // ROS_INFO_THROTTLE("Value at index %d: %d", i, ogm_msg->data[i]);
      nonbincount++;
    }
  }

  ROS_INFO_THROTTLE(0.5, "Value of nonbincount: %d", nonbincount);
  // produce the number of frontiers covered by robot's potential position
  // !!!NOTE: Control inputs below are produced by the output of RAG
  double linvel = 0.18;
  double angvel = 0.06;
  double radius = 3.5;
  double timestep = 0.5;

  double covered_frontiers = Turtlebot3_Communicator::calculateInformationGain(odom1, linvel, angvel,
                                                                               timestep, radius, markers_msg);
  ROS_INFO("CHECKPOINT 4 ");
  ROS_INFO_THROTTLE(0.5, "Number of covered frontiers for tb3_0: %f", covered_frontiers);
  // tb3_0_publisher1_.publish(markers_msg.markers);
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "tb3_0_local_comm_node"); // node name

  ros::NodeHandle nh_tb3_0;

  Tb3_0_Local_Comm my_tb3(nh_tb3_0); // initialize inherited class

  ros::spin(); // ROS spin for callback function update
  return 0;
}