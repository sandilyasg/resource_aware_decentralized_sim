#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "nav_msgs/Odometry.h"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>

#include "tf/message_filter.h"
#include "tf/transform_listener.h"
#include "filters/filter_chain.h"
#include <cmath>

class MultirobotPositionFilter
{
private:
  // Our NodeHandle
  ros::NodeHandle nh_;

  message_filters::Subscriber<sensor_msgs::LaserScan> laserscan_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_self;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_1_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_2_;

  // message_filters::TimeSynchronizer<sensor_msgs::LaserScan, nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry> sync_;
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, 
  nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  // Components for publishing
  sensor_msgs::LaserScan msg_copy;
  ros::Publisher output_pub_;

public:
  // Constructor
  MultirobotPositionFilter() :
    laserscan_sub_(nh_, "scan", 500),
    odom_sub_self(nh_, "odom0", 500),
    odom_sub_1_(nh_, "odom1", 500),
    odom_sub_2_(nh_, "odom2", 500)
    // sync_.reset(new Sync(MySyncPolicy(500), laserscan_sub_, odom_sub_self, odom_sub_1_, odom_sub_2_))
    // sync_.registerCallback(boost::bind(&MultirobotPositionFilter::callback, this, _1, _2, _3, _4))
    // sync_(laserscan_sub_, odom_sub_self, odom_sub_1_, odom_sub_2_, 500)
    // sync_(laserscan_sub_, odom_sub_self, odom_sub_1_, odom_sub_2_, ros::Duration(0.1))
    // sync_(new message_filters::TimeSynchronizer<sensor_msgs::LaserScan, nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry>(laserscan_sub_, odom_sub_self, odom_sub_1_, odom_sub_2_, 50))
  { 
    // ROS_INFO("CHECKPOINT 1");
    sync_.reset(new Sync(MySyncPolicy(10), laserscan_sub_, odom_sub_self, odom_sub_1_, odom_sub_2_));

    // sync_.registerCallback(boost::bind(&MultirobotPositionFilter::callback, this, _1, _2, _3, _4));
    // sync_.registerCallback(&MultirobotPositionFilter::callback, this);
    sync_->registerCallback(boost::bind(&MultirobotPositionFilter::callback, this, _1, _2, _3, _4));

    // ROS_INFO("CHECKPOINT 2");
    // Advertise output
    output_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_filtered", 500);
    // ROS_INFO("CHECKPOINT 3");
  }
  
  // find Euclidean distance between odom pose of 2 turtlebot3s
  double getEuclideanDistance(const nav_msgs::Odometry& odom1, const nav_msgs::Odometry& odom2)
{
    double x_diff = odom1.pose.pose.position.x - odom2.pose.pose.position.x;
    double y_diff = odom1.pose.pose.position.y - odom2.pose.pose.position.y;
    double z_diff = odom1.pose.pose.position.z - odom2.pose.pose.position.z;
    double distance = sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
    return distance;
}

  // find Euclidean distance and bearing between odom pose of 2 turtlebot3s
std::pair<double, double> getDistanceAndBearing(nav_msgs::Odometry odom1, nav_msgs::Odometry odom2) {
    double x1 = odom1.pose.pose.position.x;
    double y1 = odom1.pose.pose.position.y;
    double x2 = odom2.pose.pose.position.x;
    double y2 = odom2.pose.pose.position.y;

    double distance = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    double bearing = std::atan2(y2 - y1, x2 - x1);

    return std::make_pair(distance, bearing);
}
  
  // Callback
  // void callback(const sensor_msgs::LaserScan::ConstPtr& msg_in,
  void callback(const sensor_msgs::LaserScan::ConstPtr& msg_in,
                const nav_msgs::Odometry::ConstPtr& odom_self,
                const nav_msgs::Odometry::ConstPtr& odom_1,
                const nav_msgs::Odometry::ConstPtr& odom_2)
  {
    // sensor_msgs::LaserScan msg_copy = *msg_in;
    msg_copy = *msg_in;
    // Run the filter
    // ROS_INFO("CHECKPOINT 4");
    // ROS_INFO_THROTTLE(1.5, "Received a LaserScan message with %d ranges", msg_in->ranges.size());
    // ROS_INFO("CHECKPOINT 5");
    double dist_0to1 = getEuclideanDistance(*odom_self, *odom_1);
    double dist_0to2 = getEuclideanDistance(*odom_self, *odom_2);
    // ROS_INFO_THROTTLE(1, "Distance from rob0 to rob1 %.3f ", dist_0to1);
    // ROS_INFO_THROTTLE(1, "Distance from rob0 to rob2 %.3f ", dist_0to2);

    int rangesize = msg_in->ranges.size();
    int rangemidpoint = rangesize/2;
    int lowerbound_index = rangemidpoint - 180;
    int upperbound_index = rangemidpoint + 180;
    double range_tolerance = 0.095;

    if (dist_0to1 <= 8.0 || dist_0to2 <= 8.0)
    {
      for (int i = lowerbound_index; i < upperbound_index; i++)
      {
        if (((msg_in->ranges.at(i) <= (dist_0to2 + range_tolerance)) && 
                  (msg_in->ranges.at(i) >= (dist_0to2 - range_tolerance))) || 
                  ((msg_in->ranges.at(i) <= (dist_0to1 + range_tolerance)) && 
                  (msg_in->ranges.at(i) >= (dist_0to1 - range_tolerance))))
        {
          msg_copy.ranges.at(i) = std::numeric_limits<sensor_msgs::Range::_range_type>::infinity();
        }
      }
    }
    
    // for (int i = 0; i < msg_in->ranges.size(); i++) {
    //   ROS_INFO("Range[%d]: %.2f", static_cast<int>(i), msg_in->ranges[i]);
    // }
    // Publish the output
    output_pub_.publish(msg_copy);
    // ROS_INFO("CHECKPOINT 5.1");
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multirobot_scan_filter_node");
  // ROS_INFO("CHECKPOINT 6");
  MultirobotPositionFilter t;
  // ROS_INFO("CHECKPOINT 7");
  ros::spin();
  // ROS_INFO("CHECKPOINT 8");

  return 0;
}
