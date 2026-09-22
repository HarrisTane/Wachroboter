cmake_minimum_required(VERSION 3.8)
project(guard_robot)

# C++17 erzwingen (rclcpp braucht mindestens C++17)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Abhaengigkeiten suchen ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# Paket mit dem Service CheckCodeword
find_package(wachroboter_interfaces REQUIRED)
# Paket mit der wiederverwendbaren Fahr-Klasse WaypointDriver
find_package(waypoint_driver REQUIRED)

include_directories(include)

# --- Bibliothek mit der Wachroboter-Logik ---
add_library(guard_robot_lib
  src/guard_robot.cpp
)
ament_target_dependencies(guard_robot_lib
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  wachroboter_interfaces
  waypoint_driver
)
target_include_directories(guard_robot_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- Ausfuehrbarer Node (main-Funktion) ---
add_executable(guard_robot_node
  src/guard_robot_node.cpp
)
target_link_libraries(guard_robot_node
  guard_robot_lib
)
ament_target_dependencies(guard_robot_node
  rclcpp
  std_msgs
  wachroboter_interfaces
  waypoint_driver
)

# --- Installation ---
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS guard_robot_lib
  EXPORT export_guard_robot_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS guard_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

# Launch-Dateien und Parameter-Dateien mitinstallieren
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_guard_robot_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs wachroboter_interfaces waypoint_driver)
ament_export_include_directories(include)

ament_package()
