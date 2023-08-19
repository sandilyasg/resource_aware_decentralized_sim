#include "turtlebot3_communicator.hpp"
#include "utilfunctions.hpp"


// Turtlebot3_Communicator::Turtlebot3_Communicator() {}
// class Turtlebot3_Communicator constructor defined here
Turtlebot3_Communicator::Turtlebot3_Communicator(ros::NodeHandle &node_handle) {
  node_handle_ = node_handle;

  // Initialize subscriber that is set to the channel "/tb3_0/odom". Set callback function to be OdomModifier_callback
  subscriber1_topicname_ = "/tb3_0/odom";
  tb3_0_subscriber1_ = node_handle_.subscribe(subscriber1_topicname_, 100, &Turtlebot3_Communicator::OdomModifier_callback, this);

  //Initialize a publisher that is set to the channel "/tb3_0/odom_modified"
  publisher1_topicname_ = "/tb3_0/odom_modified";
  tb3_0_publisher1_ = node_handle_.advertise<nav_msgs::Odometry>(publisher1_topicname_, 100);
}

double Turtlebot3_Communicator::calculateInformationGain(const nav_msgs::Odometry::ConstPtr& robotPose, double& linVel, double& angVel, double& timeStep, 
  double& radius, const resource_aware_coordination::MarkerArrayHeader::ConstPtr& frontiers) {

    ROS_INFO("CHECKPOINT 3 ");
    // calculate the potential robot pose after applying the given linear and angular velocities for the given time step
    double x = robotPose->pose.pose.position.x + linVel * cos(robotPose->pose.pose.orientation.z) * timeStep;
    double y = robotPose->pose.pose.position.y + linVel * sin(robotPose->pose.pose.orientation.z) * timeStep;
    double theta = robotPose->pose.pose.orientation.z + angVel * timeStep;

    nav_msgs::Odometry newRobotPose = *robotPose; // create a copy of the robot pose to modify
    newRobotPose.pose.pose.position.x = x;
    newRobotPose.pose.pose.position.y = y;
    newRobotPose.pose.pose.orientation.z = theta;
    
    double covered = 0.0;
    ROS_INFO("CHECKPOINT 3.1 ");
    // iterate over all frontiers and check if they would be covered by the robot at the potential pose
    // visualization_msgs::MarkerArray newFrontiers = *frontiers; // create a copy of the frontiers to modify
    resource_aware_coordination::MarkerArrayHeader newFrontiers = *frontiers;
    ROS_INFO("CHECKPOINT 3.2 ");

    /*if (newFrontiers.markers.empty()) {
    ROS_WARN("newFrontiers is invalid");
    return 0.0; // or some other appropriate value
    }
    else {
        ROS_INFO("newFrontiers is valid");
    }

    for (size_t i = 0; i < newFrontiers.markers.size(); i++) {
    auto& marker = newFrontiers.markers[i];
    ROS_INFO_STREAM("Marker ID: " << marker.id);
    ROS_INFO_STREAM("Marker Pose: " << marker.pose);
    ROS_INFO_STREAM("Marker Points: ");
    for (size_t j = 0; j < marker.points.size(); j++) {
        auto& point = marker.points[j];
        ROS_INFO_STREAM("   (" << point.x << ", " << point.y << ")");
    }
    }*/
    
    ROS_INFO("CHECKPOINT 3.3 ");
    // Segmentation fault (core dumped) error after this checkpoint

    /*for (auto& frontier : newFrontiers.markers) {
        if (frontier.points.size() != newFrontiers.markers[0].points.size()) {
        ROS_WARN("Frontier points sizes are not equal, skipping this frontier");
        continue;
        }
        for (int i = 0; i < frontier.points.size(); i++) {
            double distance = euclideanDistance(frontier.points[i], newRobotPose.pose.pose.position);
            if (distance <= radius) {
                covered++;
                frontier.colors[i].r = 1.0; // set color to red
                frontier.colors[i].g = 0.0;
                frontier.colors[i].b = 0.0;
            }
        }
    }*/

    for (size_t i = 0; i < newFrontiers.markers.size(); i++) {
    ROS_INFO("CHECKPOINT 4 ");
    auto& frontier = newFrontiers.markers[i];
    // if (frontier.points.size() != newFrontiers.markers[0].points.size()) {
    //     ROS_WARN("Frontier points sizes are not equal, skipping this frontier");
    //     // continue;
    // }

        for (size_t j = 0; j < frontier.points.size(); j++) {
            ROS_INFO("CHECKPOINT 4.1 ");
            double distance = euclideanDistance(frontier.points[j], newRobotPose.pose.pose.position);
            if (distance <= radius) {
                covered++;
                ROS_INFO("CHECKPOINT 4.2 ");
                /*frontier.colors[j].r = 1.0; // set color to red
                frontier.colors[j].g = 0.0;
                frontier.colors[j].b = 0.0;*/
                ROS_INFO("CHECKPOINT 4.3 ");
            }
        }
    }
    ROS_INFO("CHECKPOINT 5 ");
    ROS_INFO("Covered: %f", covered);
    return covered; // and also the new visualization_msgs::MarkerArray& frontiers
    ROS_INFO("CHECKPOINT 6 ");
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

  tb3_0_publisher1_.publish(odom_msg_modified);

  const std::chrono::_V2::system_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> t_callback = t1 - t0;
  ROS_INFO("callback processing time: %f ms", t_callback.count());

}


// double Turtlebot3_Communicator::calculateInformationGain(nav_msgs::OccupancyGrid& map, 
//     nav_msgs::Odometry& robotPose, double& linVel, double& angVel, double& timeStep, 
//       double& radius, visualization_msgs::MarkerArray& frontiers) {
    
//     double infoGain = 0.0;
//     // calculate the potential robot pose after applying the given linear and angular velocities for the given time step
//     double x = robotPose.pose.pose.position.x + linVel * cos(robotPose.pose.pose.orientation.z) * timeStep;
//     double y = robotPose.pose.pose.position.y + linVel * sin(robotPose.pose.pose.orientation.z) * timeStep;
//     double theta = robotPose.pose.pose.orientation.z + angVel * timeStep;
    
//     robotPose.pose.pose.position.x = x;
//     robotPose.pose.pose.position.y = y;
//     robotPose.pose.pose.orientation.z = theta;
//     double covered = 0.0;
//     // iterate over all frontiers and check if they would be covered by the robot at the potential pose
//     for (auto frontier : frontiers.markers) {
//         // int covered = 0;
//         for (int i = 0; i < frontier.points.size(); i++) {
//             double distance = euclideanDistance(frontier.points[i], robotPose.pose.pose.position);
//             if (distance <= radius) {
//                 // int index = getMapIndex(map, frontier.points[i]);
//                 // if (map.data[index] == -1) {
//                     covered++;
//                     frontier.colors[i].r = 1.0; // set color to red
//                     frontier.colors[i].g = 0.0;
//                     frontier.colors[i].b = 0.0;
//                 // }
//             }
//         }
//         // if (covered == frontier.points.size()) {
//         //     infoGain += 1.0;
//         // }
//     }
//     // return infoGain;
//     return covered;
// }

// !!!original calculateInformationGain function
/*double Turtlebot3_Communicator::calculateInformationGain(nav_msgs::Odometry& robotPose, double& linVel, double& angVel, double& timeStep, 
      double& radius, visualization_msgs::MarkerArray& frontiers) {
    
    // calculate the potential robot pose after applying the given linear and angular velocities for the given time step
    double x = robotPose.pose.pose.position.x + linVel * cos(robotPose.pose.pose.orientation.z) * timeStep;
    double y = robotPose.pose.pose.position.y + linVel * sin(robotPose.pose.pose.orientation.z) * timeStep;
    double theta = robotPose.pose.pose.orientation.z + angVel * timeStep;
    
    robotPose.pose.pose.position.x = x;
    robotPose.pose.pose.position.y = y;
    robotPose.pose.pose.orientation.z = theta;
    double covered = 0.0;

    // iterate over all frontiers and check if they would be covered by the robot at the potential pose
    for (auto frontier : frontiers.markers) {
        for (int i = 0; i < frontier.points.size(); i++) {
            double distance = euclideanDistance(frontier.points[i], robotPose.pose.pose.position);
            if (distance <= radius) {
                    covered++;
                    frontier.colors[i].r = 1.0; // set color to red
                    frontier.colors[i].g = 0.0;
                    frontier.colors[i].b = 0.0;
            }
        }
    }
    return covered; // and also the new visualization_msgs::MarkerArray& frontiers
}*/


/*double Turtlebot3_Communicator::calculateInformationGain(const nav_msgs::Odometry::ConstPtr& robotPose, double& linVel, double& angVel, double& timeStep, 
  double& radius, const visualization_msgs::MarkerArray::ConstPtr& frontiers) {

    // calculate the potential robot pose after applying the given linear and angular velocities for the given time step
    double x = robotPose->pose.pose.position.x + linVel * cos(robotPose->pose.pose.orientation.z) * timeStep;
    double y = robotPose->pose.pose.position.y + linVel * sin(robotPose->pose.pose.orientation.z) * timeStep;
    double theta = robotPose->pose.pose.orientation.z + angVel * timeStep;

    nav_msgs::Odometry newRobotPose = *robotPose; // create a copy of the robot pose to modify
    newRobotPose.pose.pose.position.x = x;
    newRobotPose.pose.pose.position.y = y;
    newRobotPose.pose.pose.orientation.z = theta;
    
    double covered = 0.0;

    // iterate over all frontiers and check if they would be covered by the robot at the potential pose
    visualization_msgs::MarkerArray newFrontiers = *frontiers; // create a copy of the frontiers to modify
    for (auto& frontier : newFrontiers.markers) {
        for (int i = 0; i < frontier.points.size(); i++) {
            double distance = euclideanDistance(frontier.points[i], newRobotPose.pose.pose.position);
            if (distance <= radius) {
                covered++;
                frontier.colors[i].r = 1.0; // set color to red
                frontier.colors[i].g = 0.0;
                frontier.colors[i].b = 0.0;
            }
        }
    }
    return covered; // and also the new visualization_msgs::MarkerArray& frontiers
}*/