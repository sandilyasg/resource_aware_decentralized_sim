#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>

class OccGridMapExpander {

public:

  OccGridMapExpander(std::string map_topic, std::string new_map_topic) {
    map_sub = nh.subscribe(map_topic, 100, &OccGridMapExpander::mapCallback, this);
    new_map_pub = nh.advertise<nav_msgs::OccupancyGrid>(new_map_topic, 100);
  }

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map = *msg;
  }

  void expandMap() {

    // Create new map
    nav_msgs::OccupancyGrid new_map;
    new_map.header.frame_id = "new_map";
    new_map.info.resolution = 0.05;
    new_map.info.origin.position.x = -10.0;
    new_map.info.origin.position.y = -10.0; 
    new_map.info.origin.position.z = 0.0;
    new_map.info.origin.orientation.w = 0.0;

    // Set size
    const int width = 384;
    const int height = 384;
    new_map.info.width = width;
    new_map.info.height = height;

    // Calculate padding 
    int bottom_width = (map.info.origin.position.x - new_map.info.origin.position.x) / new_map.info.resolution;
    int bottom_height = (map.info.origin.position.y - new_map.info.origin.position.y) / new_map.info.resolution;

    // Fill new map with unknown cells
    for (int i = 0; i < width * bottom_height; i++) {
      new_map.data.push_back(-1); 
    }

    int counter = 0;
    for (int i = 0; i < map.info.height; i++) {
      
      // Add bottom padding
      for (int j = 0; j < bottom_width; j++) {
        new_map.data.push_back(-1);
      }
      
      // Copy original map data
      for (int j = 0; j < map.info.width; j++) {
        new_map.data.push_back(map.data[counter]);
        counter++;
      }
      
      // Add top padding  
      for (int j = 0; j < (width - map.info.width - bottom_width); j++) {
        new_map.data.push_back(-1);
      }
      
    }

    // Add left padding
    for (int i = 0; i < (height - map.info.height - bottom_height) * width; i++) {
      new_map.data.push_back(-1);
    }

    // Publish new map
    new_map_pub.publish(new_map);

  }

private:

  ros::NodeHandle nh;
  
  ros::Subscriber map_sub;
  ros::Publisher new_map_pub;

  nav_msgs::OccupancyGrid map;

};


int main(int argc, char** argv) {

  ros::init(argc, argv, "map_expander");

  OccGridMapExpander map_expander_1("/tb3_0/map", "/new_tb3_0_map");
  OccGridMapExpander map_expander_2("/tb3_1/map", "/new_tb3_1_map");
  OccGridMapExpander map_expander_3("/tb3_2/map", "/new_tb3_2_map");

  ros::spin();

  return 0;

}