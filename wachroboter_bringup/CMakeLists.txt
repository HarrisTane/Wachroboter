cmake_minimum_required(VERSION 3.8)
project(wachroboter_bringup)

# Dieses Paket enthaelt NUR Launch- und YAML-Dateien, keinen C++-Code.
# Deshalb reicht ament_cmake und ein install()-Aufruf.

find_package(ament_cmake REQUIRED)

# Launch-Dateien und Parameter-Dateien nach
# install/wachroboter_bringup/share/wachroboter_bringup/ kopieren.
# Ohne diesen Block findet "ros2 launch" die Dateien nicht!
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
