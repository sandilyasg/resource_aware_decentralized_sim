#ifndef TURTLEBOT3_COMMUNICATOR_HPP_
#define TURTLEBOT3_COMMUNICATOR_HPP_

#include <string>
#include <iostream>
#include "ros/ros.h"
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Time.h>
#include <chrono>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
// #include "utilfunctions.hpp"
#include "resource_aware_coordination/MarkerArrayHeader.h"

class Turtlebot3_Communicator {
  public:
    Turtlebot3_Communicator() {}
    Turtlebot3_Communicator(ros::NodeHandle &node_handle);

  // private:
  protected:
    void OdomModifier_callback(const nav_msgs::Odometry::ConstPtr &odom_msg_data);
    // double calculateInformationGain(nav_msgs::Odometry& robotPose, double& linVel, double& angVel, double& timeStep, 
    //   double& radius, visualization_msgs::MarkerArray& frontiers);

  // double calculateInformationGain(const nav_msgs::Odometry::ConstPtr& robotPose, double& linVel, double& angVel, double& timeStep, 
  //   double& radius, const visualization_msgs::MarkerArray::ConstPtr& frontiers);

  double calculateInformationGain(const nav_msgs::Odometry::ConstPtr& robotPose, double& linVel, double& angVel, double& timeStep, 
    double& radius, const resource_aware_coordination::MarkerArrayHeader::ConstPtr& frontiers);

    ros::NodeHandle node_handle_;
    // ros::Subscriber tb3_0_odom_subscriber_;
    // ros::Publisher tb3_0_odom_publisher_;
    ros::Subscriber tb3_0_subscriber1_;
    ros::Publisher tb3_0_publisher1_;
    // std::string odom_subscriber_topicname_;
    // std::string odom_publisher_topicname_;
    std::string subscriber1_topicname_;
    std::string publisher1_topicname_;

};

#endif  // TURTLEBOT3_COMMUNICATOR_HPP_
