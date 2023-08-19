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

    void FourPointCallback(const nav_msgs::Odometry::ConstPtr& odom1, 
    const sensor_msgs::LaserScan::ConstPtr& laserscan1);

    void FALKOMapFeatureMarkerCallbackOdom(const nav_msgs::Odometry::ConstPtr& odom1, 
    const sensor_msgs::LaserScan::ConstPtr& laserscan1);

    void MapFeatureMarkerCallback(const nav_msgs::Odometry::ConstPtr& odom1, 
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
        sync_.reset(new Sync(MySyncPolicy(1.0), odom1_subscriber_, laserscan_subscriber_));
        sync_->registerCallback(boost::bind(&LaserScanDetection::FALKOMapFeatureMarkerCallbackOdom, this, _1, _2));
        // sync_->registerCallback(boost::bind(&LaserScanDetection::FALKOMapFeatureMarkerCallbackOdom, this, _1, _2));
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

void LaserScanDetection::FourPointCallback(const nav_msgs::Odometry::ConstPtr& odom1,
                                          const sensor_msgs::LaserScan::ConstPtr& laserscan1) {
    // Store the robot's pose
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.pose = odom1->pose.pose;
    robot_pose.header = odom1->header;

    visualization_msgs::MarkerArray markerArray;

    // ROS_INFO_THROTTLE(0.5, "Size of laserscan1->ranges: %zu", laserscan1->ranges.size());
    // std::vector<size_t> indices = {0, 90, 180, 270, laserscan1->ranges.size() - 1};
    std::vector<size_t> indices = {0, 90, 180, 270};


    // Iterate over the specified indices
    for (size_t i : indices) {
        // Get the range at the specified index
        float range = laserscan1->ranges[i];

        // Check if the range is finite
        if (!std::isinf(range)) {
            // Calculate the corresponding angle based on index and laser scan parameters
            float bearing = laserscan1->angle_min + i * laserscan1->angle_increment;
            // ROS_INFO_THROTTLE(0.5, "Index: %zu, Range: %.2f, Bearing: %.2f", i, range, bearing);
            ROS_INFO("Index: %zu, Range: %.2f, Bearing: %.2f", i, range, bearing);
 
            // Extract the robot's position and orientation
            double robot_x = robot_pose.pose.position.x;
            double robot_y = robot_pose.pose.position.y;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(robot_pose.pose.orientation, quat);
            double robot_yaw = tf::getYaw(quat);
            //  ROS_INFO_THROTTLE(0.5, "robot_yaw: %.2f", robot_yaw);  
            ROS_INFO("Robot pose: [x: %.2f, y: %.2f, yaw: %.2f]", robot_x, robot_y, robot_yaw);

            // Create a transformation matrix
            Eigen::Matrix3d transform;
            transform << std::cos(robot_yaw), -std::sin(robot_yaw), robot_x,
                std::sin(robot_yaw), std::cos(robot_yaw), robot_y,
                0, 0, 1;

            ROS_INFO("Transform matrix:");
            for (int i = 0; i < transform.rows(); ++i) {
              ROS_INFO("[%f, %f, %f]", transform(i, 0), transform(i, 1), transform(i, 2));
            }

            // ROS_INFO("Transform matrix:");
            // for (int i = 0; i < transform.rows(); ++i) {
            //   for (int j = 0; j < transform.cols(); ++j) {
            //     ROS_INFO("[%d, %d]: %f", i, j, transform(i, j));
            //   }
            // }

            // Transform the point to the world frame using the transformation matrix
            auto xR = range * std::cos(bearing); auto yR = range * std::sin(bearing);
            ROS_INFO("xR: %.2f, yR: %.2f", xR, yR);
            Eigen::Vector3d point_local(xR, yR, 1.0);
            Eigen::Vector3d point_world = transform * point_local;
            ROS_INFO("point_world: [x: %.2f, y: %.2f, z: %.2f]", point_world[0], point_world[1], point_world[2]);

            // Create a marker for the point
            visualization_msgs::Marker marker;
            // marker.header = laserscan1->header;  // !!!WRONG! assigns parent frame as base_scan
            marker.header.frame_id = "map";
            marker.ns = "cardinal_points";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = point_world[0];
            marker.pose.position.y = point_world[1];
            // marker.pose.position.z = point_world[2];
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            // Set marker color based on index
            if (i == 90) {
                marker.color.r = 1.0;  // Red
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            } else if (i == 180) {
                marker.color.r = 1.0;  // White
                marker.color.g = 1.0;
                marker.color.b = 1.0;
            } else if (i == 270) {
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;  // Blue
            } 
            else {
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 1.0;  // White (default color)
            }

            markerArray.markers.push_back(marker);
        }
    }

    // Create a marker for the robot's pose
    visualization_msgs::Marker robotMarker;
    robotMarker.header = odom1->header;
    robotMarker.ns = "robot_pose";
    robotMarker.id = 0;
    robotMarker.type = visualization_msgs::Marker::SPHERE;
    robotMarker.action = visualization_msgs::Marker::ADD;
    robotMarker.pose = robot_pose.pose;
    robotMarker.scale.x = 0.1;
    robotMarker.scale.y = 0.1;
    robotMarker.scale.z = 0.1;
    robotMarker.color.a = 1.0;
    robotMarker.color.r = 0.0;
    robotMarker.color.g = 1.0;
    robotMarker.color.b = 0.0;

    markerArray.markers.push_back(robotMarker);
    // Publish the marker array
    lasermapf_publisher_.publish(markerArray);
}


void LaserScanDetection::FALKOMapFeatureMarkerCallbackOdom(const nav_msgs::Odometry::ConstPtr& odom1, 
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

    // Create marker array for visualization
    visualization_msgs::MarkerArray markerArray;
    for (size_t i = 0; i < transformedKeypoints.size(); ++i) {
      visualization_msgs::Marker marker;
      // marker.header = laserscan1->header; // !!!WRONG! assigns parent frame as base_scan
      marker.header.frame_id = "map"; 
      marker.header.stamp = ros::Time::now();
      marker.ns = "transformed_keypoints";
      marker.id = i; // Each marker should have a unique ID
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position = transformedKeypoints[i];

      // Set the scale of the marker
      marker.scale.x = 0.1; marker.scale.y = 0.1; marker.scale.z = 0.1;

      // Set the color and transparency of the marker
      marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
      markerArray.markers.push_back(marker); // Add the marker to the array
    }

    // Publish the marker array
    lasermapf_publisher_.publish(markerArray);
}



void LaserScanDetection::MapFeatureMarkerCallback(const nav_msgs::Odometry::ConstPtr& odom1, 
  const sensor_msgs::LaserScan::ConstPtr& laserscan1) {

    /*  // Access individual range and bearing values
    for (size_t i = 0; i < laserscan1->ranges.size(); ++i) {
        float range = laserscan1->ranges[i];
        float bearing = laserscan1->angle_min + i * laserscan1->angle_increment;

        // ROS_INFO_THROTTLE(0.5, "  Range[%zu]: %f, Bearing[%zu]: %f", i, range, i, bearing);
        ROS_INFO("  Range[%zu]: %f, Bearing[%zu]: %f", i, range, i, bearing);
    }
    ROS_INFO("\n");   */

  // Store the robot's pose
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.pose = odom1->pose.pose;
    robot_pose.header = odom1->header;

    visualization_msgs::MarkerArray markerArray;

    // Iterate over the laser scan ranges
    for (size_t i = 1; i < laserscan1->ranges.size() - 1; ++i) {
        float prev_range = laserscan1->ranges[i - 1];
        float curr_range = laserscan1->ranges[i];
        float next_range = laserscan1->ranges[i + 1];

        // Check if the range change between consecutive ranges exceeds a threshold
        float range_threshold = 0.05; // Modify the threshold as per your requirement
        float prev_range_change = std::abs(curr_range - prev_range);
        float next_range_change = std::abs(curr_range - next_range);

        // Check if the current range is finite and the range change exceeds the threshold
         if (!std::isinf(curr_range) && prev_range_change > range_threshold && next_range_change > range_threshold) {
            // Calculate the global position of the corner point
            float curr_bearing = laserscan1->angle_min + i * laserscan1->angle_increment;

            float x = robot_pose.pose.position.x + curr_range * cos(curr_bearing);
            float y = robot_pose.pose.position.y + curr_range * sin(curr_bearing);

            // Create a marker for the corner point
            visualization_msgs::Marker marker;
            // marker.header = laserscan1->header; // !!!WRONG! assigns parent frame as base_scan
            marker.header.frame_id = "map"; 
            marker.ns = "corners";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            markerArray.markers.push_back(marker);
        }
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

