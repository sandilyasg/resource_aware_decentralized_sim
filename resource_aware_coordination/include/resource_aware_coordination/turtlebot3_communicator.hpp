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
#include "utilfunctions.hpp"
#include "resource_aware_coordination/MarkerArrayHeader.h"
#include "slam_karto_gtsam_landmark/OptimizationOutput.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>
#include "tf/message_filter.h"
#include "tf/transform_listener.h"
#include "resource_aware_coordination/MarkerArrayHeader.h"
#include <eigen3/Eigen/Dense>

class Turtlebot3_Communicator {
  public:
    Turtlebot3_Communicator() {}
    Turtlebot3_Communicator(ros::NodeHandle &node_handle);

  // private:
  protected:
    void OdomModifier_callback(const nav_msgs::Odometry::ConstPtr &odom_msg_data);
    double calculateInformationGain(const nav_msgs::Odometry::ConstPtr& robotPose, double& linVel, double& angVel, double& timeStep, 
    double& radius, const resource_aware_coordination::MarkerArrayHeader::ConstPtr& frontiers);

    

    void deserializeOptimizedMsg(const slam_karto_gtsam_landmark::OptimizationOutput::ConstPtr &optim_msg, 
    std::vector<Eigen::Matrix<double, 3, 3>>& pose_covariances, 
    std::vector<Eigen::Matrix<double, 2, 2>> & mapfeature_covariances,
    std::vector<int> & observed_keyids,
    std::vector<Eigen::Vector3d> &robot_poses, 
    std::vector<Eigen::Vector2d> &mapfeature_points);

    ros::NodeHandle node_handle_;
    ros::Subscriber tb3_0_subscriber1_;
    ros::Publisher tb3_0_publisher1_;
    std::string subscriber1_topicname_;
    std::string publisher1_topicname_;

    std::vector<Landmark> landmarks;

};

#endif  // TURTLEBOT3_COMMUNICATOR_HPP_
