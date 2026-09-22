#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "visitor_robot/visitor_robot.hpp"
#include "waypoint_driver/waypoint_driver.hpp"

// Diese Datei enthaelt nur die main-Funktion.
// Wichtig: In EINEM Prozess laufen hier ZWEI Nodes:
//   1) visitor_robot    -> Statemachine + Service-Client
//   2) waypoint_driver  -> faehrt die Wegpunkte ab
// Beide werden in denselben Executor gehaengt, damit visitor_robot die
// Fahr-Klasse direkt per Methodenaufruf steuern kann.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // --- Fahr-Node erzeugen ---
  // auto_start=false: der Fahr-Node faehrt NICHT von selbst los.
  // Die Statemachine setzt die Wegpunkte und ruft dann start() auf.
  rclcpp::NodeOptions driver_options;
  driver_options.parameter_overrides({
    rclcpp::Parameter("auto_start", false)
  });
  auto driver = std::make_shared<waypoint_driver::WaypointDriver>(driver_options);

  // --- Besucher-Node erzeugen ---
  auto visitor = std::make_shared<visitor_robot::VisitorRobot>();

  // Fahr-Node beim Besucher bekannt machen (ruft intern auch stop() auf)
  visitor->setDriver(driver);

  // --- Beide Nodes in einem Executor ausfuehren ---
  // SingleThreadedExecutor: alle Callbacks laufen nacheinander in einem Thread.
  // Der Service-Aufruf ist asynchron, deshalb blockiert hier nichts.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(visitor);
  executor.add_node(driver);

  RCLCPP_INFO(visitor->get_logger(), "Besucher-Roboter startet...");

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
