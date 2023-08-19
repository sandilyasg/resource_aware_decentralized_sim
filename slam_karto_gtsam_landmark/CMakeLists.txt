cmake_minimum_required(VERSION 2.8.3)
project(slam_karto_gtsam_landmark)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    message_filters
    nav_msgs
    open_karto
    rosconsole
    roscpp
    sensor_msgs
    geometry_msgs
    tf
    visualization_msgs
    cpp_common
    gencpp
    geneus
    genlisp
    genmsg
    gennodejs
    genpy
    message_generation
    message_runtime
    ros_environment
    rosbuild
    roscpp_serialization
    roscpp_traits
    rosgraph
    rosgraph_msgs
    roslib
    rospack
    rospy
    rostime
    std_msgs
    xmlrpcpp
    actionlib
    actionlib_msgs
    map_msgs
    move_base_msgs

    # cmake_modules
    # message_filters
    # nav_msgs
    # open_karto
    # rosconsole
    # roscpp
    # sensor_msgs
    # geometry_msgs
    # tf
    # visualization_msgs
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
# set(MKL_INCLUDE_DIR "/opt/intel/oneapi/mkl/2023.1.0/include")
# set(MKL_LIBRARIES "/opt/intel/oneapi/mkl/2023.1.0/lib/intel64/mkl_rt.lib") #/opt/intel/oneapi/mkl/2023.1.0/lib/cmake/mkl
# set(MKL_LIBRARIES "/opt/intel/oneapi/mkl/2023.1.0/lib/intel64/libmkl_rt.so")
# set(MKL_LIBRARIES "/opt/intel/oneapi/mkl/2023.1.0/lib/cmake/mkl")

add_message_files(
  FILES
  OptimizationOutput.msg
)

generate_messages(
  DEPENDENCIES
  rosgraph_msgs #   std_msgs
  std_msgs
  visualization_msgs
)

find_package(MKL REQUIRED)
if(MKL_FOUND)
    message(WARNING "MKL FOUND!!!")
    include_directories(${MKL_INCLUDE_DIR})
else()
    message(WARNING "MKL libs not found")
endif()

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
if(GTSAM_FOUND)
  message(STATUS "Found GTSAM ${GTSAM_VERSION}")
else()
  message(STATUS "GTSAM Not Found")
endif()

find_package(Boost REQUIRED COMPONENTS timer thread serialization chrono)
# SSG: MAKE SURE TO SET THESE PATHS ACCORDING TO YOUR FILE SYSTEM. THIS IS SPECIFIC TO
# SSG kartoslam_sandbox WORSKPACE USED WHILE DEVELOPING THE PROGRAM
# set(falkolib_DIR "/home/sgari/iral_research/kartoslam_ws/src/openslam_falkolib/cmake_modules")
# set(falkolib_INCLUDE_DIR "/home/sgari/iral_research/kartoslam_ws/src/openslam_falkolib/include")

find_package(falkolib REQUIRED)  

if(${falkolib_FOUND}) 
  include_directories(${falkolib_INCLUDE_DIRS})
  link_directories(${falkolib_LIBRARY_DIRS})
endif()


include_directories(
    include/${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS} 
    ${EIGEN_INCLUDE_DIRS}
    ${MKL_INCLUDE_DIR}
    ${GTSAM_INCLUDE_DIRS}
    ${falkolib_INCLUDE_DIRS}
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp message_filters nav_msgs open_karto rosconsole roscpp sensor_msgs geometry_msgs tf visualization_msgs 
#  CATKIN_DEPENDS roscpp 
#  message_filters 
#  nav_msgs open_karto 
#  rosconsole 
#  sensor_msgs 
#  geometry_msgs 
#  tf 
#  visualization_msgs 
#  cpp_common 
#  gencpp 
#  geneus 
#  genlisp 
#  genmsg 
#  gennodejs 
#  genpy 
#  message_generation 
#  message_runtime 
#  ros_environment 
#  rosbuild 
#  roscpp_serialization 
#  roscpp_traits 
#  rosgraph 
#  rosgraph_msgs 
#  roslib 
#  rospack 
#  rospy 
#  rostime 
#  std_msgs 
#  xmlrpcpp 
#  actionlib 
#  actionlib_msgs 
#  costmap_2d 
#  map_msgs 
#  move_base_msgs
DEPENDS Eigen3
)

add_executable(slam_karto_gtsam_landmark src/slam_karto_gtsam_landmark.cpp src/GTSAMSolver.cpp)

get_target_property(gtsam_link_lib gtsam INTERFACE_LINK_LIBRARIES)
message("${gtsam_link_lib}")

target_link_libraries(slam_karto_gtsam_landmark gtsam ${catkin_LIBRARIES} ${MKL_LIBRARIES} ${falkolib_LIBRARIES})

# target_link_libraries(slam_karto_gtsam_landmark "-Wl,--start-group" gtsam ${catkin_LIBRARIES} ${MKL_LIBRARIES} ${falkolib_LIBRARIES} "-Wl,--end-group -lpthread -lm -ldl")

add_executable(laserscan_feature_detector src/laserscan_feature_detection.cpp)
target_link_libraries(laserscan_feature_detector ${catkin_LIBRARIES} ${falkolib_LIBRARIES})

#install(TARGETS slam_karto_gtsam_landmark
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)