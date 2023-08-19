#include "utilfunctions.hpp"

double euclideanDistance(geometry_msgs::Point p1, geometry_msgs::Point p2) {
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}



int getMapIndex(nav_msgs::OccupancyGrid map, geometry_msgs::Point point) {
    int x = (point.x - map.info.origin.position.x) / map.info.resolution;
    int y = (point.y - map.info.origin.position.y) / map.info.resolution;
    return x + y * map.info.width;
}
