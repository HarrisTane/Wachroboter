#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "guard_robot/guard_robot.hpp"
#include "waypoint_driver/waypoint_driver.hpp"

// Diese Datei enthaelt nur die main-Funktion.
// Wichtig: In EINEM Prozess laufen hier ZWEI Nodes:
//   1) guard_robot      -> Statemachine + Service + Alarm
//   2) waypoint_driver  -> faehrt die Wegpunkte ab
// Beide werden in denselben Executor gehaengt, damit guard_robot die
// Fahr-Klasse direkt per Methodenaufruf steuern kann.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // --- Fahr-Node erzeugen ---
  // auto_start=false: der Fahr-Node faehrt NICHT von selbst los.
  // Erst guard_robot ruft nach dem Alarm start() auf.
  rclcpp::NodeOptions driver_options;
  driver_options.parameter_overrides({
    rclcpp::Parameter("auto_start", false)
  });
  auto driver = std::make_shared<waypoint_driver::WaypointDriver>(driver_options);

  // --- Wachroboter-Node erzeugen ---
  auto guard = std::make_shared<guard_robot::GuardRobot>();

  // Fahr-Node beim Wachroboter bekannt machen (ruft intern auch stop() auf)
  guard->setDriver(driver);

  // --- Beide Nodes in einem Executor ausfuehren ---
  // SingleThreadedExecutor: alle Callbacks laufen nacheinander in einem Thread.
  // Dadurch koennen sich Service-Callback und Timer nicht gegenseitig stoeren
  // (kein Mutex noetig).
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(guard);
  executor.add_node(driver);

  RCLCPP_INFO(guard->get_logger(), "Wachroboter bereit. Warte auf Codewort...");

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
