cmake_minimum_required(VERSION 3.0.2)
project(goodsalm)
# Set C++ standard
set(CMAKE_CXX_STANDARD 14)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_generation
  nav_msgs
  pcl_ros
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common io octree)
find_package(GTest REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread)
set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.10)  #指定pcl1.8路径


catkin_package(
  CATKIN_DEPENDS 
  INCLUDE_DIRS
  DEPENDS EIGEN3 PCL
  
)
include_directories(${PCL_INCLUDE_DIRS})


# Include directories
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
    include
)

# Link directories
link_directories(
    ${PCL_LIBRARY_DIRS}
)

# Add definitions
add_definitions(${PCL_DEFINITIONS})

# Create the main library
add_library(octree_voxel_lib src/octree_voxel.cpp)
target_link_libraries(octree_voxel_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Create the test executable
add_executable(octree_voxel_test src/octree_voxel_test.cpp)
target_link_libraries(octree_voxel_test octree_voxel_lib ${GTEST_BOTH_LIBRARIES} pthread)

# Enable testing
enable_testing()
add_test(NAME OctreeVoxelTests COMMAND octree_voxel_test)