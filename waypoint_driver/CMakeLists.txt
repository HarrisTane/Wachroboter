cmake_minimum_required(VERSION 3.8)
project(waypoint_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Bibliothek mit der wiederverwendbaren Node-Klasse.
# Andere Pakete können diese Bibliothek + den Header später einbinden.
add_library(waypoint_driver_lib
  src/waypoint_driver.cpp
)
ament_target_dependencies(waypoint_driver_lib
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)
target_include_directories(waypoint_driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Ausführbarer Node, der nur die Bibliothek startet (main-Funktion)
add_executable(waypoint_driver_node
  src/waypoint_driver_node.cpp
)
target_link_libraries(waypoint_driver_node
  waypoint_driver_lib
)

# Header für andere Pakete installieren
install(
  DIRECTORY include/
  DESTINATION include
)

# Bibliothek installieren und für andere Pakete exportieren
install(
  TARGETS waypoint_driver_lib
  EXPORT export_waypoint_driver_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Ausführbaren Node installieren
install(
  TARGETS waypoint_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_waypoint_driver_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)
ament_export_include_directories(include)

ament_package()
