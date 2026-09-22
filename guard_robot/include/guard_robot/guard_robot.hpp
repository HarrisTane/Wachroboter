#ifndef GUARD_ROBOT__GUARD_ROBOT_HPP_
#define GUARD_ROBOT__GUARD_ROBOT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <wachroboter_interfaces/srv/check_codeword.hpp>
#include "waypoint_driver/waypoint_driver.hpp"

#include <memory>
#include <string>
#include <vector>

namespace guard_robot
{

// Die drei Zustaende der Aufgabenstellung plus ein Endzustand.
// enum class = "starkes" enum: Werte muessen mit GuardState:: angesprochen
// werden, dadurch keine Verwechslung mit anderen Zahlen moeglich.
enum class GuardState
{
  WARTEN,            // Roboter steht und wartet auf eine Codewort-Anfrage
  ALARM,             // Falsches Codewort -> Warnung auf /alarm publizieren
  FAHRE_ZU_POLIZEI,  // Wegpunkte zur Zielposition 2 abfahren
  FERTIG             // Zielposition 2 erreicht, Roboter steht
};

// Wachroboter-Node. Bietet den Service "check_codeword" an und steuert
// ueber die Klasse waypoint_driver::WaypointDriver die Fahrt zur Polizei.
class GuardRobot : public rclcpp::Node
{
public:
  explicit GuardRobot(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Der Fahr-Node wird in main() erzeugt und hier uebergeben.
  // (Eine Node kann keine zweite Node in ihrem eigenen Konstruktor in den
  // Executor haengen, deshalb dieser Zwischenschritt.)
  void setDriver(std::shared_ptr<waypoint_driver::WaypointDriver> driver);

private:
  // --- Service-Callback: wird bei jeder Codewort-Anfrage aufgerufen ---
  void handleCheckCodeword(
    const std::shared_ptr<wachroboter_interfaces::srv::CheckCodeword::Request> request,
    std::shared_ptr<wachroboter_interfaces::srv::CheckCodeword::Response> response);

  // --- Zustandsmaschine, laeuft periodisch per Timer ---
  void stateLoop();

  // --- Hilfsfunktionen ---
  bool isCodewordValid(const std::string & codeword) const;  // Codewort pruefen
  void changeState(GuardState new_state);                    // Zustandswechsel + Log
  static const char * stateToString(GuardState state);       // Zustand als Text
  void publishAlarm();                                       // Warnung auf /alarm

  // --- ROS-Schnittstellen ---
  rclcpp::Service<wachroboter_interfaces::srv::CheckCodeword>::SharedPtr codeword_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alarm_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  // --- Der Fahr-Node aus dem Paket waypoint_driver ---
  std::shared_ptr<waypoint_driver::WaypointDriver> driver_;

  // --- Parameter ---
  std::vector<std::string> valid_codewords_;   // gueltige Codewoerter
  std::vector<double> police_waypoints_flat_;  // Wegpunkte zur Polizei: x1,y1,x2,y2,...
  double alarm_duration_;                      // Alarmdauer in Sekunden
  double alarm_interval_;                      // Abstand der Alarm-Nachrichten in Sekunden
  std::string alarm_text_;                     // Inhalt der Alarm-Nachricht
  bool case_sensitive_;                        // Gross-/Kleinschreibung beachten?

  // --- Interner Zustand ---
  GuardState state_ = GuardState::WARTEN;  // aktueller Zustand der Statemachine
  rclcpp::Time alarm_start_;               // Zeitpunkt, an dem der Alarm begann
  rclcpp::Time last_alarm_pub_;            // Zeitpunkt der letzten Alarm-Nachricht
};

}  // namespace guard_robot

#endif  // GUARD_ROBOT__GUARD_ROBOT_HPP_
