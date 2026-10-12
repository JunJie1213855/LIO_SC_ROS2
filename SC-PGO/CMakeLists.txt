cmake_minimum_required(VERSION 3.5)
project(aloam_velodyne)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# find_package(rosbag REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED QUIET)

include_directories(
  include
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

# catkin_package(
#   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#   DEPENDS EIGEN3 PCL
#   INCLUDE_DIRS include
# )

ament_package()

# SC-PGO backend only: ScanContext loop closure + GTSAM pose-graph optimization.
# (The A-LOAM frontend — scanRegistration / laserOdometry / laserMapping — was removed;
#  Point-LIO is the frontend instead.)
add_executable(alaserPGO
  src/laserPosegraphOptimization.cpp
  src/sc_pgo_node.cpp
  include/scancontext/Scancontext.cpp
)
ament_target_dependencies(alaserPGO rclcpp rclpy std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge pcl_conversions)
target_compile_options(alaserPGO
  PRIVATE ${OpenMP_CXX_FLAGS}
)
target_link_libraries(alaserPGO
  ${PCL_LIBRARIES} ${CERES_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  gtsam
)

install(TARGETS
  alaserPGO
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz_cfg config
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/superlio_scpgo_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
