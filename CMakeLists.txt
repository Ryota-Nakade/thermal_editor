cmake_minimum_required(VERSION 3.0.2)
project(thermal_editor)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros

  OpenCV REQUIRED
  #PCL REQUIRED
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(thermal_imager src/thermal_imager.cpp)
add_executable(feature_imager src/feature_imager.cpp)
add_executable(thermal_image_divider src/thermal_image_divider.cpp)
add_executable(thermal_feature_extractor src/thermal_feature_extractor.cpp)
add_executable(usb_pub src/usb_pub.cpp)
add_executable(optris_correct_distortion src/correct_distortion_cpp.cpp)

target_link_libraries(thermal_imager
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(feature_imager
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(thermal_image_divider
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(thermal_feature_extractor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(usb_pub
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(optris_correct_distortion
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)