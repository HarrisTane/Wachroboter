#include "waypoint_driver/waypoint_driver.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

namespace waypoint_driver
{

WaypointDriver::WaypointDriver(const rclcpp::NodeOptions & options)
: Node("waypoint_driver", options)
{
  // --- Parameter deklarieren (Standardwerte passend zur Aufgabenstellung) ---
  this->declare_parameter<std::vector<double>>("waypoints", std::vector<double>{});
  this->declare_parameter<double>("kp_angular", 1.5);
  this->declare_parameter<double>("kp_linear", 0.6);
  this->declare_parameter<double>("max_linear_speed", 0.2);
  this->declare_parameter<double>("max_angular_speed", 0.5);
  this->declare_parameter<double>("goal_tolerance", 0.15);
  this->declare_parameter<double>("angle_tolerance_deg", 6.0);
  this->declare_parameter<double>("safety_distance", 0.4);
  this->declare_parameter<double>("safety_angle_deg", 30.0);
  this->declare_parameter<double>("control_frequency", 20.0);
  // NEU: Soll sofort losgefahren werden? true = altes Verhalten.
  // guard_robot startet diese Klasse mit auto_start=false, damit der
  // Wachroboter erst nach dem Alarm losfaehrt.
  this->declare_parameter<bool>("auto_start", true);

  // --- Parameter auslesen ---
  kp_angular_ = this->get_parameter("kp_angular").as_double();
  kp_linear_ = this->get_parameter("kp_linear").as_double();
  max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
  max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  angle_tolerance_ = this->get_parameter("angle_tolerance_deg").as_double() * M_PI / 180.0;
  safety_distance_ = this->get_parameter("safety_distance").as_double();
  safety_angle_rad_ = this->get_parameter("safety_angle_deg").as_double() * M_PI / 180.0;
  control_frequency_ = this->get_parameter("control_frequency").as_double();
  active_ = this->get_parameter("auto_start").as_bool();  // NEU

  // Wegpunkte aus dem Parameter "waypoints" (flache Liste x1,y1,x2,y2,...) laden
  loadWaypointParameter();

  // --- Publisher ---
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // "waypoints_done" mit transient_local: auch später startende Subscriber
  // (z.B. der andere Roboter) bekommen den letzten Wert noch mitgeteilt
  rclcpp::QoS done_qos(1);
  done_qos.transient_local();
  done_pub_ = this->create_publisher<std_msgs::msg::Bool>("waypoints_done", done_qos);

  // --- Subscriber ---
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&WaypointDriver::odomCallback, this, std::placeholders::_1));

  // /scan nutzt BEST_EFFORT -> SensorDataQoS(), sonst kommen keine Nachrichten an
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    std::bind(&WaypointDriver::scanCallback, this, std::placeholders::_1));

  // --- Timer für die Regelschleife ---
  auto period = std::chrono::duration<double>(1.0 / control_frequency_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&WaypointDriver::controlLoop, this));

  RCLCPP_INFO(this->get_logger(),
    "waypoint_driver gestartet mit %zu Wegpunkten (auto_start=%s).",
    waypoints_relative_.size(), active_ ? "true" : "false");
}

// NEU: Wegpunkte zur Laufzeit setzen (z.B. vom Wachroboter nach dem Alarm)
void WaypointDriver::setWaypoints(const std::vector<Waypoint> & waypoints)
{
  waypoints_relative_ = waypoints;   // neue Liste uebernehmen
  current_index_ = 0;                // wieder beim ersten Wegpunkt anfangen
  state_ = DriveState::ROTATE;       // jeder Wegpunkt startet mit Drehen
  done_published_ = false;           // "fertig" darf spaeter erneut gemeldet werden

  // Nur umrechnen, wenn die Startpose schon aus /odom bekannt ist.
  // Sonst passiert das automatisch beim ersten odomCallback().
  if (has_start_pose_) {
    transformWaypointsToOdomFrame();
  }

  RCLCPP_INFO(this->get_logger(), "Neue Wegpunktliste gesetzt: %zu Wegpunkte.",
    waypoints_relative_.size());
}

// NEU: Fahren freigeben
void WaypointDriver::start()
{
  active_ = true;
  RCLCPP_INFO(this->get_logger(), "waypoint_driver: Fahren AKTIVIERT.");
}

// NEU: Fahren sperren und sofort anhalten
void WaypointDriver::stop()
{
  active_ = false;
  publishStop();  // sofort Geschwindigkeit 0 senden
  RCLCPP_INFO(this->get_logger(), "waypoint_driver: Fahren GESTOPPT.");
}

void WaypointDriver::loadWaypointParameter()
{
  auto flat = this->get_parameter("waypoints").as_double_array();

  // Die Liste muss aus x/y-Paaren bestehen -> gerade Anzahl an Werten
  if (flat.size() % 2 != 0) {
    RCLCPP_ERROR(this->get_logger(),
      "Parameter 'waypoints' hat eine ungerade Anzahl an Werten (%zu). "
      "Letzter Wert wird ignoriert.", flat.size());
  }

  waypoints_relative_.clear();
  for (std::size_t i = 0; i + 1 < flat.size(); i += 2) {
    Waypoint wp;
    wp.x = flat[i];
    wp.y = flat[i + 1];
    waypoints_relative_.push_back(wp);
  }

  // Warnung nur, wenn wir auch sofort losfahren sollen. Beim Wachroboter
  // ist eine leere Liste am Anfang normal (Wegpunkte kommen spaeter).
  if (waypoints_relative_.empty() && active_) {
    RCLCPP_WARN(this->get_logger(),
      "Keine Wegpunkte übergeben! Parameter 'waypoints' setzen, "
      "z.B. -p waypoints:=\"[1.0, 0.0, 1.0, 1.0]\"");
  }
}

double WaypointDriver::quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
{
  // Standardformel: aus Quaternion den Gierwinkel (Drehung um Z-Achse) berechnen
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double WaypointDriver::normalizeAngle(double angle)
{
  // Winkel in den Bereich [-pi, pi] bringen (z.B. 350° -> -10° statt +350°)
  while (angle > M_PI) {angle -= 2.0 * M_PI;}
  while (angle < -M_PI) {angle += 2.0 * M_PI;}
  return angle;
}

void WaypointDriver::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Aktuelle Pose aus der Odometrie übernehmen
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
  current_yaw_ = quaternionToYaw(msg->pose.pose.orientation);
  has_odom_ = true;

  // Beim ersten /odom-Wert die Startpose merken.
  // Alle Wegpunkte sind relativ zu dieser Startpose angegeben.
  if (!has_start_pose_) {
    start_x_ = current_x_;
    start_y_ = current_y_;
    start_yaw_ = current_yaw_;
    has_start_pose_ = true;

    transformWaypointsToOdomFrame();

    RCLCPP_INFO(this->get_logger(),
      "Startpose gesetzt: x=%.2f y=%.2f yaw=%.1f°.",
      start_x_, start_y_, start_yaw_ * 180.0 / M_PI);
  }
}

void WaypointDriver::transformWaypointsToOdomFrame()
{
  // Wegpunkte sind relativ zur Startpose (x=vorwärts, y=links aus Robotersicht
  // beim Start) angegeben. Hier einmalig per 2D-Drehmatrix + Verschiebung
  // in feste Odom-Koordinaten umrechnen.
  waypoints_odom_.clear();
  double cos_yaw = std::cos(start_yaw_);
  double sin_yaw = std::sin(start_yaw_);

  for (const auto & wp_rel : waypoints_relative_) {
    Waypoint wp_abs;
    wp_abs.x = start_x_ + wp_rel.x * cos_yaw - wp_rel.y * sin_yaw;
    wp_abs.y = start_y_ + wp_rel.x * sin_yaw + wp_rel.y * cos_yaw;
    waypoints_odom_.push_back(wp_abs);
  }
}

void WaypointDriver::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Bei jeder neuen Scan-Nachricht den Sicherheitsstatus neu bestimmen
  // (nur der aktuellste Scan zählt, kein Mischen mit alten Werten)
  bool blocked = false;

  for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
    float range = msg->ranges[i];

    // Ungültige Messwerte (NaN, Inf, außerhalb des Sensorbereichs) ignorieren
    if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) {
      continue;
    }

    // Winkel dieses Messpunkts relativ zur Fahrzeugfront (Annahme: Sensor
    // schaut geradeaus, Winkel 0 = vorne)
    double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;

    if (std::fabs(angle) > safety_angle_rad_) {
      continue;  // außerhalb des vorderen +/- 30°-Bereichs
    }

    if (range < safety_distance_) {
      blocked = true;
      break;  // ein zu naher Punkt reicht
    }
  }

  obstacle_blocked_ = blocked;
}

void WaypointDriver::publishStop()
{
  geometry_msgs::msg::Twist stop_msg;
  // Alle Felder sind default 0.0 -> Roboter hält an
  cmd_vel_pub_->publish(stop_msg);
}

void WaypointDriver::publishDoneIfNeeded()
{
  if (!done_published_) {
    std_msgs::msg::Bool done_msg;
    done_msg.data = true;
    done_pub_->publish(done_msg);
    done_published_ = true;
    RCLCPP_INFO(this->get_logger(), "Alle Wegpunkte erreicht. waypoints_done = true.");
  }
}

void WaypointDriver::controlLoop()
{
  // NEU: Solange nicht aktiv -> Roboter stehen lassen und nichts regeln.
  // (Wir senden bewusst Geschwindigkeit 0, damit der Roboter garantiert haelt.)
  if (!active_) {
    publishStop();
    return;
  }

  // Ohne Odometrie können wir nicht regeln -> sicherheitshalber anhalten und warten
  if (!has_odom_ || !has_start_pose_) {
    publishStop();
    return;
  }

  // Alle Wegpunkte abgefahren -> anhalten, einmalig "fertig" melden
  if (current_index_ >= waypoints_odom_.size()) {
    publishStop();
    publishDoneIfNeeded();
    return;
  }

  // --- Sicherheitscheck hat höchste Priorität ---
  if (obstacle_blocked_) {
    publishStop();
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Hindernis näher als %.2f m vorne -> Stopp!", safety_distance_);
    return;
  }

  // --- Regelabweichungen zum aktuellen Wegpunkt ---
  const Waypoint & target = waypoints_odom_[current_index_];
  double dx = target.x - current_x_;
  double dy = target.y - current_y_;
  double distance = std::hypot(dx, dy);
  double target_angle = std::atan2(dy, dx);
  double angle_error = normalizeAngle(target_angle - current_yaw_);

  geometry_msgs::msg::Twist cmd;

  // --- Wegpunkt erreicht? ---
  if (distance < goal_tolerance_) {
    RCLCPP_INFO(this->get_logger(), "Wegpunkt %zu erreicht (x=%.2f y=%.2f).",
      current_index_ + 1, target.x, target.y);
    current_index_++;
    state_ = DriveState::ROTATE;  // nächster Wegpunkt startet wieder mit Drehen
    publishStop();
    return;
  }

  // --- Zustandswechsel mit Hysterese (verhindert Flackern Drehen<->Fahren) ---
  if (state_ == DriveState::ROTATE && std::fabs(angle_error) < angle_tolerance_) {
    state_ = DriveState::DRIVE;
  } else if (state_ == DriveState::DRIVE && std::fabs(angle_error) > 2.0 * angle_tolerance_) {
    // Während der Fahrt zu stark vom Kurs abgekommen -> neu ausrichten
    state_ = DriveState::ROTATE;
  }

  if (state_ == DriveState::ROTATE) {
    // Phase 1: Drehen (P-Regler auf Winkelfehler)
    cmd.angular.z = kp_angular_ * angle_error;
    cmd.linear.x = 0.0;
  } else {
    // Phase 2: Geradeausfahren (P-Regler auf Distanz)
    cmd.linear.x = kp_linear_ * distance;
    cmd.angular.z = 0.0;
  }

  // --- Auf Maximalgeschwindigkeiten begrenzen ---
  cmd.linear.x = std::clamp(cmd.linear.x, -max_linear_speed_, max_linear_speed_);
  cmd.angular.z = std::clamp(cmd.angular.z, -max_angular_speed_, max_angular_speed_);

  cmd_vel_pub_->publish(cmd);
}

}  // namespace waypoint_driver
