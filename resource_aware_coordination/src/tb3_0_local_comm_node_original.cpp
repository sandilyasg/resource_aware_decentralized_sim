/* Local communication node for  turtlebot3 0 (tb3_0) in ROS and Gazebo simulation for implementing
Resource-Aware distributed Greedy (RAG) algorithm. The node from this code facilitates communication
from the base station node (rag_base_station) where it is able to subscribe to or publish to any topic.

Written by: Sandilya Sai Garimella March/19/2023 
Affiliation: University of Michigan Intelligent Robotics and Autonomy Lab (iRaL)
*/
#include "turtlebot3_communicator.hpp"
#include "utilfunctions.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>
#include "tf/message_filter.h"
#include "tf/transform_listener.h"
#include "filters/filter_chain.h"
#include "resource_aware_coordination/MarkerArrayHeader.h"


class Tb3_0_Local_Comm : public Turtlebot3_Communicator {

  protected:
    // declare any additional private members or functions
    // void InfogainMarkerCallback(nav_msgs::Odometry& odom1, visualization_msgs::MarkerArray& markers_msg);

    void InfogainMarkerCallback(const nav_msgs::Odometry::ConstPtr& odom1, 
          const visualization_msgs::MarkerArray::ConstPtr& markers_msg);

    /*message_filters::Subscriber<nav_msgs::Odometry> tb3_0_subscriber1_;
    message_filters::Subscriber<visualization_msgs::MarkerArray> tb3_0_subscriber2_;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, 
        visualization_msgs::MarkerArray> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_; */

    // subscriber1_topicname_ = "/tb3_0/odom";

  public:
  // declare any additional public  members or functions
  Tb3_0_Local_Comm(ros::NodeHandle& node_handle);
  /*Tb3_0_Local_Comm(ros::NodeHandle& node_handle) :
    tb3_0_subscriber1_(node_handle, "/tb3_0/odom", 1),
    tb3_0_subscriber2_(node_handle, "/globalmap_fronter_explore/frontiers", 1)
    {
      sync_.reset(new Sync(MySyncPolicy(10), tb3_0_subscriber1_, tb3_0_subscriber2_));
      sync_->registerCallback(boost::bind(&Tb3_0_Local_Comm::InfogainMarkerCallback, this, _1, _2));
      tb3_0_publisher1_ = node_handle.advertise<visualization_msgs::MarkerArray>("covered_frontiers", 500);
    }*/
};

Tb3_0_Local_Comm::Tb3_0_Local_Comm(ros::NodeHandle& node_handle) : Turtlebot3_Communicator(node_handle) {
  // add any additional constructor code
  // overwrite the derived constructor
  
}

void Tb3_0_Local_Comm::InfogainMarkerCallback(const nav_msgs::Odometry::ConstPtr& odom1, 
const visualization_msgs::MarkerArray::ConstPtr& markers_msg) {
  // produce the number of frontiers covered by robot's potential position
  double linvel = 0.18; 
  double angvel = 0.06;
  double radius = 3.5;
  double timestep = 0.5;

  double covered_frontiers = Turtlebot3_Communicator::calculateInformationGain(odom1, linvel, angvel,
      timestep, radius, markers_msg);

  ROS_INFO_THROTTLE(0.5, "Number of covered frontiers for tb3_0: %f", covered_frontiers);
  tb3_0_publisher1_.publish(markers_msg);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "tb3_0_local_comm_node"); // node name

  ros::NodeHandle nh_tb3_0;

  Tb3_0_Local_Comm my_tb3(nh_tb3_0); // initialize inherited class

  ros::spin(); // ROS spin for callback function update
  return 0;
}