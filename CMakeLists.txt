cmake_minimum_required(VERSION 3.8)
project(pnp_rgbd_vio)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(pnp_rgbd_vio_node
  src/pnp_rgbd_vio.cpp
)

ament_target_dependencies(pnp_rgbd_vio_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
)

target_link_libraries(pnp_rgbd_vio_node
  ${OpenCV_LIBS}
)

install(TARGETS
  pnp_rgbd_vio_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

