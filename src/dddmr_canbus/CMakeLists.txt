cmake_minimum_required(VERSION 3.5)
project(dddmr_canbus)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pluginlib REQUIRED)

## Build libraries
find_package(Threads REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  visualization_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  message_filters
  tf2_ros
  tf2
  tf2_geometry_msgs
  tf2_eigen
  pluginlib
)

include_directories(
  include
)

# add sdk main library
add_library(dddmr_canbus
    ########################
    src/async_can.cpp
    ########################
)
target_link_libraries(dddmr_canbus PUBLIC Threads::Threads)
target_compile_definitions(dddmr_canbus PUBLIC ASIO_ENABLE_OLD_SERVICES)
target_include_directories(dddmr_canbus PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# canbus_read_node
add_executable(canbus_read_node
  src/canbus_read_node.cpp
)
target_link_libraries(canbus_read_node
  #${PCL_LIBRARIES}
  dddmr_canbus
)
ament_target_dependencies(canbus_read_node
  ${dependencies}
)

#[[
install(TARGETS
  parameters
  lidar_measurement_model_likelihood
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
#]]
#install node
install(TARGETS
canbus_read_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
 DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
