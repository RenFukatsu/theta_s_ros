cmake_minimum_required(VERSION 3.0.2)
project(theta_s_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
)

find_package(OpenCV 4 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES theta_s_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
    image_converter src/image_converter.cpp include/opencv_theta_s/EquirectangularConversion/ThetaConversion.cpp
)

add_dependencies(
    image_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(image_converter
    ${catkin_LIBRARIES}
)
