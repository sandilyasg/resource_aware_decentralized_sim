cmake_minimum_required(VERSION 3.0.2)
project(multirobot_laserscan_filter)

set(CMAKE_CXX_STANDARD 11)

##############################################################################
# Find dependencies
##############################################################################

set(THIS_PACKAGE_ROS_DEPS sensor_msgs roscpp tf filters message_filters
  laser_geometry pluginlib angles dynamic_reconfigure nodelet)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})
find_package(Boost REQUIRED COMPONENTS system)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

##############################################################################
# Define package
##############################################################################

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/IntensityFilter.cfg
  cfg/PolygonFilter.cfg
  cfg/RangeFilter.cfg
  cfg/ScanShadowsFilter.cfg
  cfg/SpeckleFilter.cfg
  cfg/SectorFilter.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pointcloud_filters laser_scan_filters
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
  DEPENDS
  )

##############################################################################
# Build
##############################################################################

add_library(pointcloud_filters src/pointcloud_filters.cpp)
target_link_libraries(pointcloud_filters ${catkin_LIBRARIES})

add_library(laser_scan_filters
  src/laser_scan_filters.cpp
  src/multirobot_pos_filter.cpp
)
target_link_libraries(laser_scan_filters ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(scan_to_cloud_filter_chain src/scan_to_cloud_filter_chain.cpp)
target_link_libraries(scan_to_cloud_filter_chain ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(scan_to_cloud_filter_chain_nodelet src/scan_to_cloud_filter_chain.cpp)
target_link_libraries(scan_to_cloud_filter_chain_nodelet ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_compile_definitions(scan_to_cloud_filter_chain_nodelet PRIVATE BUILDING_NODELET=1)

add_executable(scan_to_scan_filter_chain src/scan_to_scan_filter_chain.cpp)
target_link_libraries(scan_to_scan_filter_chain ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(generic_laser_filter_node src/generic_laser_filter_node.cpp)
target_link_libraries(generic_laser_filter_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(multirobot_position_filter src/multirobot_position_filter.cpp)
target_link_libraries(multirobot_position_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(laser_scan_filters ${PROJECT_NAME}_gencfg)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest)
  add_executable(test_scan_filter_chain test/test_scan_filter_chain.cpp)
  target_link_libraries(test_scan_filter_chain laser_scan_filters ${rostest_LIBRARIES} ${GTEST_LIBRARIES})
  add_dependencies(test_scan_filter_chain gtest)
  
endif()

##############################################################################
# Install
##############################################################################

install(TARGETS pointcloud_filters laser_scan_filters 
  scan_to_cloud_filter_chain
  scan_to_cloud_filter_chain_nodelet
  scan_to_scan_filter_chain
  generic_laser_filter_node
  multirobot_position_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES multirobot_laserscan_filter_plugins.xml nodelets.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
