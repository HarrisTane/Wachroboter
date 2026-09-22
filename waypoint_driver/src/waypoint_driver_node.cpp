#include "waypoint_driver/waypoint_driver.hpp"

// Diese Datei enthält nur die main-Funktion. Die eigentliche Logik steckt
// in der Klasse waypoint_driver::WaypointDriver (siehe waypoint_driver.hpp/.cpp),
// damit andere Pakete die Klasse direkt einbinden können.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<waypoint_driver::WaypointDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
