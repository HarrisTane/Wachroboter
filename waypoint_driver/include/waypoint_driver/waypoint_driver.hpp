#ifndef WAYPOINT_DRIVER__WAYPOINT_DRIVER_HPP_
#define WAYPOINT_DRIVER__WAYPOINT_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <cstddef>

namespace waypoint_driver
{

// Ein einzelner Wegpunkt: x- und y-Koordinate in Metern
struct Waypoint
{
  double x;
  double y;
};

// Fahrzustand pro Wegpunkt: erst drehen, dann fahren
enum class DriveState
{
  ROTATE,
  DRIVE
};

// Wiederverwendbare Node-Klasse. Kann von anderen Paketen direkt
// eingebunden und instanziiert werden (Header + Bibliothek "waypoint_driver_lib").
class WaypointDriver : public rclcpp::Node
{
public:
  explicit WaypointDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // ---------------------------------------------------------------------
  // NEU: Öffentliche Steuer-Schnittstelle für andere Pakete (z.B. guard_robot)
  // ---------------------------------------------------------------------

  // Setzt neue Wegpunkte zur Laufzeit (relativ zur Startpose des Roboters).
  // Setzt gleichzeitig den Fahrfortschritt zurück (Index, Zustand, Fertig-Flag).
  void setWaypoints(const std::vector<Waypoint> & waypoints);

  // Startet das Abfahren der Wegpunkte (Regelschleife gibt ab jetzt Fahrbefehle).
  void start();

  // Stoppt das Fahren sofort (Roboter bekommt Geschwindigkeit 0).
  void stop();

  // true, sobald alle Wegpunkte abgefahren wurden.
  bool isFinished() const {return done_published_;}

  // true, solange der Fahrmodus aktiv ist.
  bool isActive() const {return active_;}

private:
  // --- Callbacks ---
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void controlLoop();

  // --- Hilfsfunktionen ---
  static double quaternionToYaw(const geometry_msgs::msg::Quaternion & q);
  static double normalizeAngle(double angle);
  void loadWaypointParameter();
  void transformWaypointsToOdomFrame();
  void publishStop();
  void publishDoneIfNeeded();

  // --- ROS-Schnittstellen ---
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- Aus ROS-Parametern gelesene Werte ---
  double kp_angular_;
  double kp_linear_;
  double max_linear_speed_;
  double max_angular_speed_;
  double goal_tolerance_;
  double angle_tolerance_;      // Bogenmaß, Schwelle zum Wechsel Drehen->Fahren
  double safety_distance_;
  double safety_angle_rad_;
  double control_frequency_;

  // --- Wegpunkte ---
  std::vector<Waypoint> waypoints_relative_;  // wie im Parameter angegeben (relativ zum Start)
  std::vector<Waypoint> waypoints_odom_;      // umgerechnet ins Odom-Koordinatensystem
  std::size_t current_index_ = 0;
  DriveState state_ = DriveState::ROTATE;

  // --- Odometrie-Zustand ---
  bool has_odom_ = false;
  bool has_start_pose_ = false;
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_yaw_ = 0.0;
  double start_x_ = 0.0;
  double start_y_ = 0.0;
  double start_yaw_ = 0.0;

  // --- Sicherheit ---
  bool obstacle_blocked_ = false;

  // --- Fertig-Status ---
  bool done_published_ = false;

  // NEU: Ist der Fahrmodus aktiv? Wird über den Parameter "auto_start"
  // vorbelegt (Standard true = altes Verhalten, faehrt sofort los).
  // guard_robot setzt "auto_start" auf false und ruft spaeter start() auf.
  bool active_ = true;
};

}  // namespace waypoint_driver

#endif  // WAYPOINT_DRIVER__WAYPOINT_DRIVER_HPP_
