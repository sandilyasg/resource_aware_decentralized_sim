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

#include <sensor_msgs/LaserScan.h>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>
#include "tf/message_filter.h"
#include "tf/transform_listener.h"

#include <falkolib/Feature/FALKO.h>
#include <falkolib/Feature/CGH.h>
#include <falkolib/Feature/BSC.h>
#include <falkolib/Feature/FALKOExtractor.h>

#include <falkolib/Feature/BSCExtractor.h>
#include <falkolib/Feature/CGHExtractor.h>

#include <falkolib/Matching/NNMatcher.h>
#include <falkolib/Matching/AHTMatcher.h>

#include <math.h>
#include <eigen3/Eigen/Dense>

using namespace falkolib;

class LaserScanDetection {
  protected:

    FALKOExtractor fe;
    LaserScan scan1;

    std::vector<geometry_msgs::Point> bufferMapKeypoints;

    void FALKOMapFeatureMarkerCallback(const nav_msgs::Odometry::ConstPtr& odom1, 
    const sensor_msgs::LaserScan::ConstPtr& laserscan1);

    message_filters::Subscriber<nav_msgs::Odometry> odom1_subscriber_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laserscan_subscriber_;

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,sensor_msgs::LaserScan> MySyncPolicy;

    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;   

    ros::Publisher lasermapf_publisher_;

  public:
    LaserScanDetection(ros::NodeHandle& node_handle) :
      odom1_subscriber_(node_handle, "/odom", 500),
      laserscan_subscriber_(node_handle, "/scan", 500)
      {
        bufferMapKeypoints = std::vector<geometry_msgs::Point>(); // Initialize bufferMapKeypoints as an empty vector
        sync_.reset(new Sync(MySyncPolicy(1.0), odom1_subscriber_, laserscan_subscriber_));
        sync_->registerCallback(boost::bind(&LaserScanDetection::FALKOMapFeatureMarkerCallback, this, _1, _2));
        // sync_->registerCallback(boost::bind(&LaserScanDetection::FALKOMapFeatureMarkerCallback, this, _1, _2));
        lasermapf_publisher_ = node_handle.advertise<visualization_msgs::MarkerArray>("map_features", 500);
        
        scan1 = LaserScan(0.0, 2.0 * M_PI, 360);
        // Initialize the FALKO extractor and set its parameters
        fe.setMinExtractionRange(0.05);
        fe.setMaxExtractionRange(4.0);
        fe.enableSubbeam(true);
        fe.setNMSRadius(0.025); // Non-Maxima Suppression Radius
        fe.setNeighB(0.05); // 0.07
        fe.setBRatio(3); //2.5
        fe.setGridSectors(16);
      }
};

void LaserScanDetection::FALKOMapFeatureMarkerCallback(const nav_msgs::Odometry::ConstPtr& odom1, 
  const sensor_msgs::LaserScan::ConstPtr& laserscan1) {

  // Store the robot's pose
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.pose = odom1->pose.pose;
    robot_pose.header = odom1->header;

    visualization_msgs::MarkerArray marker_array;

    std::vector<FALKO> keypoints1;

    std::vector<float> rangesFloat = laserscan1->ranges;
    std::vector<double> rangesDouble(rangesFloat.begin(), rangesFloat.end());
    scan1.fromRanges(rangesDouble);

    fe.extract(scan1, keypoints1);

    ROS_INFO("num keypoints1 extracted: %lu", static_cast<unsigned long>(keypoints1.size()));

    // Extract indices of keypoints from laserscan1
    std::vector<int> keypointIndices;
    for (const auto& keypoint : keypoints1) {
        keypointIndices.push_back(keypoint.index);
    }

    // Get robot's position and orientation
    double robot_x = robot_pose.pose.position.x;
    double robot_y = robot_pose.pose.position.y;
    // double robot_yaw = tf::getYaw(robot_pose.pose.orientation);
    tf::Quaternion quat;
    tf::quaternionMsgToTF(robot_pose.pose.orientation, quat);
    double robot_yaw = tf::getYaw(quat);

    // Create transformation matrix
    Eigen::Matrix3d transform;
    transform << std::cos(robot_yaw), -std::sin(robot_yaw), robot_x,
                std::sin(robot_yaw), std::cos(robot_yaw), robot_y,
                0, 0, 1;

    // Transform keypoints from laser frame to world frame
    std::vector<geometry_msgs::Point> transformedKeypoints;
    for (const auto& keypointIndex : keypointIndices) {
      // Retrieve range and angle from laserscan1 using the keypoint index
      float range = laserscan1->ranges[keypointIndex];
      float angle = laserscan1->angle_min + keypointIndex * laserscan1->angle_increment;

      // Calculate Cartesian coordinates in the laser's frame
      double xLaser = range * std::cos(angle);
      double yLaser = range * std::sin(angle);

      // Apply transformation from laser frame to world frame
      Eigen::Vector3d pointLaser(xLaser, yLaser, 1.0);
      Eigen::Vector3d pointWorld = transform * pointLaser;

      // Store the transformed keypoint coordinates
      geometry_msgs::Point transformedPoint;
      transformedPoint.x = pointWorld[0];
      transformedPoint.y = pointWorld[1];
      // transformedPoint.z = pointWorld[2];
      transformedPoint.z = 0.0;
      transformedKeypoints.push_back(transformedPoint);
    }

     // Iterate over transformed keypoints
    for (const auto& transformedPoint : transformedKeypoints) {
      bool isDuplicate = false;

      // Check if the transformed point is within the specified distance of any point in bufferMapKeypoints
      for (const auto& bufferPoint : bufferMapKeypoints) {
        double distance = std::hypot(transformedPoint.x - bufferPoint.x, transformedPoint.y - bufferPoint.y);
        if (distance <= 0.18) {
          isDuplicate = true;
          break;
        }
      }

      // Add the transformed point to bufferMapKeypoints if it is not a duplicate
      if (!isDuplicate) {
        bufferMapKeypoints.push_back(transformedPoint);
      }
    }

    // Create marker array for visualization
    visualization_msgs::MarkerArray markerArray;
    for (size_t i = 0; i < bufferMapKeypoints.size(); ++i) {
      visualization_msgs::Marker marker;
      // marker.header = laserscan1->header; // !!!WRONG! assigns parent frame as base_scan
      marker.header.frame_id = "map"; 
      marker.header.stamp = ros::Time::now();
      marker.ns = "transformed_keypoints";
      marker.id = i; // Each marker should have a unique ID
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position = bufferMapKeypoints[i];

      // Set the scale of the marker
      marker.scale.x = 0.1; marker.scale.y = 0.1; marker.scale.z = 0.1;

      // Set the color and transparency of the marker
      marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
      markerArray.markers.push_back(marker); // Add the marker to the array
    }

    // Publish the marker array
    lasermapf_publisher_.publish(markerArray);
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "laserscan_feature_detection_node"); // node name

  ros::NodeHandle laserscan_nh;

  LaserScanDetection laserscan_fdn(laserscan_nh); // initialize inherited class

  ros::spin(); // ROS spin for callback function update
  return 0;
}

