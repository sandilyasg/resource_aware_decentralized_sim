#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <std_msgs/Header.h>
#include <nav_msgs/MapMetaData.h>

// Define global message 
nav_msgs::OccupancyGrid slam0_map, slam1_map, slam2_map;

// Callback function for map0
void map0Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  slam0_map = *msg;
}

// Callback function for map1
void map1Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  slam1_map = *msg;
}

// Callback function for map2
void map2Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  slam2_map = *msg;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "localmap_expander");
  ros::NodeHandle nh;

  // Create publishers and subscribers
  const auto new_tb3_0_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("new_tb3_0_map", 100);
  const auto map_meta0_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/tb3_0/map", 100, map0Callback);

  const auto new_tb3_1_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("new_tb3_1_map", 100);
  const auto map_meta1_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/tb3_1/map", 100, map1Callback);

  const auto new_tb3_2_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("new_tb3_2_map", 100);
  const auto map_meta2_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/tb3_2/map", 100, map2Callback);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    if (slam0_map.data.size() != 0 && slam1_map.data.size() != 0 && slam2_map.data.size() != 0)
    {
      // Create the new larger local maps for each robot
      nav_msgs::OccupancyGrid new_tb3_0_map, new_tb3_1_map, new_tb3_2_map;
      new_tb3_0_map.header.frame_id = "new_tb3_0_map";
      new_tb3_1_map.header.frame_id = "new_tb3_1_map";
      new_tb3_2_map.header.frame_id = "new_tb3_2_map";

      // Set the new map resolution
      new_tb3_0_map.info.resolution = slam0_map.info.resolution;
      new_tb3_1_map.info.resolution = slam1_map.info.resolution;
      new_tb3_2_map.info.resolution = slam2_map.info.resolution;

      // Set the new map origin to match the original SLAM maps
      new_tb3_0_map.info.origin.position.x = slam0_map.info.origin.position.x;
      new_tb3_0_map.info.origin.position.y = slam0_map.info.origin.position.y;

      new_tb3_1_map.info.origin.position.x = slam1_map.info.origin.position.x;
      new_tb3_1_map.info.origin.position.y = slam1_map.info.origin.position.y;

      new_tb3_2_map.info.origin.position.x = slam2_map.info.origin.position.x;
      new_tb3_2_map.info.origin.position.y = slam2_map.info.origin.position.y;

      // Calculate the dimensions of the new maps
      int bottom0_width = slam0_map.info.width;
      int bottom0_height = slam0_map.info.height;

      int bottom1_width = slam1_map.info.width;
      int bottom1_height = slam1_map.info.height;

      int bottom2_width = slam2_map.info.width;
      int bottom2_height = slam2_map.info.height;

      int width = std::max(std::max(bottom0_width, bottom1_width), bottom2_width);
      int height = std::max(std::max(bottom0_height, bottom1_height), bottom2_height);

      // Set the new map dimensions
      new_tb3_0_map.info.width = width;
      new_tb3_0_map.info.height = height;

      new_tb3_1_map.info.width = width;
      new_tb3_1_map.info.height = height;

      new_tb3_2_map.info.width = width;
      new_tb3_2_map.info.height = height;

      // Resize the new map data
      new_tb3_0_map.data.resize(width * height);
      new_tb3_1_map.data.resize(width * height);
      new_tb3_2_map.data.resize(width * height);

      // Merge the maps
      for (int i = 0; i < width * height; ++i) {
        int x = i % width;
        int y = i / width;

        int index0 = x + slam0_map.info.width * y;
        int index1 = x + slam1_map.info.width * y;
        int index2 = x + slam2_map.info.width * y;

        int value = -1;

        if (x < slam0_map.info.width && y < slam0_map.info.height) {
          value = slam0_map.data[index0];
        } else if (x < slam1_map.info.width && y < slam1_map.info.height) {
          value = slam1_map.data[index1];
        } else if (x < slam2_map.info.width && y < slam2_map.info.height) {
          value = slam2_map.data[index2];
        }

        new_tb3_0_map.data[i] = value;
        new_tb3_1_map.data[i] = value;
        new_tb3_2_map.data[i] = value;
      }

      // Publish the new maps
      new_tb3_0_map_pub.publish(new_tb3_0_map);
      new_tb3_1_map_pub.publish(new_tb3_1_map);
      new_tb3_2_map_pub.publish(new_tb3_2_map);

      // Reset the SLAM maps
      slam0_map.data.clear();
      slam1_map.data.clear();
      slam2_map.data.clear();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
