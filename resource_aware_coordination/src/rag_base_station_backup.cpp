/* Main communication node for all 3 turtlebot3s in ROS and Gazebo simulation for implementing
Resource-Aware distributed Greedy (RAG) algorithm. The node from this code facilitates communication
in the form of a "base station" where it is able to subscribe to or publish to any topic.

Written by: Sandilya Sai Garimella March/19/2023 
Affiliation: University of Michigan Intelligent Robotics and Autonomy Lab (iRaL)
*/

#include <string>
#include <iostream>
#include "ros/ros.h"
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Time.h>
#include <chrono>

class Turtlebot3_Communicator {
  public:
    Turtlebot3_Communicator(ros::NodeHandle &node_handle);

  private:
    void OdomModifier_callback(const nav_msgs::Odometry::ConstPtr &odom_msg_data);
    ros::NodeHandle node_handle_;
    ros::Subscriber tb3_0_odom_subscriber_;
    ros::Publisher tb3_0_odom_publisher_;
    std::string odom_subscriber_topicname_;
    std::string odom_publisher_topicname_;

};

// class Turtlebot3_Communicator constructor defined here
Turtlebot3_Communicator::Turtlebot3_Communicator(ros::NodeHandle &node_handle) {
  node_handle_ = node_handle;

  // Initialize subscriber that is set to the channel "/tb3_0/odom". Set callback function to be OdomModifier_callback
  odom_subscriber_topicname_ = "/tb3_0/odom";
  tb3_0_odom_subscriber_ = node_handle_.subscribe(odom_subscriber_topicname_, 1000, &Turtlebot3_Communicator::OdomModifier_callback, this);

  //Initialize a publisher that is set to the channel "/tb3_0/odom_modified"
  odom_publisher_topicname_ = "/tb3_0/odom_modified";
  tb3_0_odom_publisher_ = node_handle_.advertise<nav_msgs::Odometry>(odom_publisher_topicname_, 1000);
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

  tb3_0_odom_publisher_.publish(odom_msg_modified);

  const std::chrono::_V2::system_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> t_callback = t1 - t0;
  ROS_INFO("callback processing time: %f ms", t_callback.count());

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "rag_base_station_node"); // node name

  // ros::NodeHandle nh("~"); // can set namespace in nodehandle
  ros::NodeHandle nh;

  Turtlebot3_Communicator tb3_comm(nh); // initialize class

  ros::spin(); // ROS spin for callback function update
  return 0;
}
