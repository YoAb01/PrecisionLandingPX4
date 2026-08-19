cmake_minimum_required(VERSION 3.8)
project(drone_vision_landing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(detect_aruco_tag src/detect_aruco_tag.cpp)
target_include_directories(detect_aruco_tag PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(detect_aruco_tag PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  detect_aruco_tag
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "image_transport"
  "std_msgs"
  "cv_bridge"
  "eigen3_cmake_module"
  Eigen3
  OpenCV
)
target_link_libraries(detect_aruco_tag Eigen3::Eigen)

install(TARGETS detect_aruco_tag
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
