cmake_minimum_required(VERSION 3.13)
project(MyCOMMONLIB)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  cv_bridge
)

find_package(PCL REQUIRED)
find_package(Ceres  REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)

include_directories(
  include
  include/ros
  ${EIGEN_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Ceres_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(mycommonlib
src/HandEyeNonlinear.cc
#src/ros/BagReader.cc
)

target_link_libraries(mycommonlib
  ${catkin_LIBRARIES}
  ${EIGEN_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
)

# install lib to system
install()
