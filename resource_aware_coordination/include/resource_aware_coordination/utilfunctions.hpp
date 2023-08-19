#ifndef UTILFUNCTIONS_HPP_
#define UTILFUNCTIONS_HPP_

#include <string>
#include <iostream>
// #include <nav_msgs/Odometry.h>
// #include <std_msgs/Time.h>
// #include <chrono>
#include <nav_msgs/OccupancyGrid.h>
// #include <nav_msgs/Odometry.h>
// #include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <cmath>

// function to calculate the Euclidean distance between two points
double euclideanDistance(geometry_msgs::Point p1, geometry_msgs::Point p2);
// function to calculate the index of a point in a grid map
int getMapIndex(nav_msgs::OccupancyGrid map, geometry_msgs::Point point);

struct Landmark {
    std::string name;
    double x;
    double y;
    double z;
};

#endif  // UTILFUNCTIONS_HPP_
