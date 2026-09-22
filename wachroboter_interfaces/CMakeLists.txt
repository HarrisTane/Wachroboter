cmake_minimum_required(VERSION 3.8)
project(wachroboter_interfaces)

# Nötig, damit Compiler-Warnungen sauber angezeigt werden
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Standard-ROS-2-Buildsystem einbinden
find_package(ament_cmake REQUIRED)

# Paket, das aus .srv-Dateien C++/Python-Code generiert
find_package(rosidl_default_generators REQUIRED)

# Hier wird festgelegt, welche Interface-Dateien generiert werden sollen
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CheckCodeword.srv"
)

# Macht sicher, dass abhängige Pakete auch die Laufzeit-Bibliothek finden
ament_export_dependencies(rosidl_default_runtime)

ament_package()
