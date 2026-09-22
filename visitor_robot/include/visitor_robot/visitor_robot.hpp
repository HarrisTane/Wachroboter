#ifndef VISITOR_ROBOT__VISITOR_ROBOT_HPP_
#define VISITOR_ROBOT__VISITOR_ROBOT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <wachroboter_interfaces/srv/check_codeword.hpp>
#include "waypoint_driver/waypoint_driver.hpp"

#include <memory>
#include <string>
#include <vector>

namespace visitor_robot
{

// Zustaende des ankommenden Roboters.
// enum class = "starkes" enum: Werte muessen mit VisitorState:: angesprochen
// werden, dadurch keine Verwechslung mit anderen Zahlen moeglich.
enum class VisitorState
{
  FAHRE_ZU_WACHE,      // Wegpunkte bis kurz vor den Wachroboter abfahren
  SENDE_CODEWORT,      // Service /check_codeword aufrufen und auf Antwort warten
  FAHRE_ZU_LAGER,      // granted=true  -> Wegpunkte zur Zielposition 1 (Lager)
  ZUGANG_VERWEIGERT,   // granted=false -> stehen bleiben
  FERTIG               // Lager erreicht, Roboter steht
};

// Besucher-Roboter-Node. Faehrt zur Wache, fragt das Codewort ab und
// faehrt je nach Antwort weiter zum Lager oder bleibt stehen.
class VisitorRobot : public rclcpp::Node
{
public:
  explicit VisitorRobot(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Der Fahr-Node wird in main() erzeugt und hier uebergeben.
  // (Eine Node kann keine zweite Node in ihrem eigenen Konstruktor in den
  // Executor haengen, deshalb dieser Zwischenschritt.)
  void setDriver(std::shared_ptr<waypoint_driver::WaypointDriver> driver);

private:
  // --- Zustandsmaschine, laeuft periodisch per Timer ---
  void stateLoop();

  // --- Einzelne Zustaende ---
  void handleFahreZuWache();
  void handleSendeCodewort();
  void handleFahreZuLager();

  // --- Hilfsfunktionen ---
  void changeState(VisitorState new_state);              // Zustandswechsel + Log
  static const char * stateToString(VisitorState state); // Zustand als Text
  // Flache Parameterliste (x1,y1,x2,y2,...) in Wegpunkte umwandeln
  std::vector<waypoint_driver::Waypoint> toWaypoints(const std::vector<double> & flat) const;
  // Wegpunkte an den Fahr-Node uebergeben und Fahren freigeben
  void startDriving(const std::vector<double> & flat, const std::string & ziel_name);
  void sendRequest();  // einen asynchronen Service-Aufruf absetzen

  // --- ROS-Schnittstellen ---
  rclcpp::Client<wachroboter_interfaces::srv::CheckCodeword>::SharedPtr codeword_client_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  // --- Der Fahr-Node aus dem Paket waypoint_driver ---
  std::shared_ptr<waypoint_driver::WaypointDriver> driver_;

  // --- Parameter ---
  std::string codeword_;                    // Codewort, das gesendet wird
  std::string service_name_;                // Name des Services (/check_codeword)
  std::vector<double> wache_waypoints_;     // Wegpunkte bis vor den Wachroboter
  std::vector<double> lager_waypoints_;     // Wegpunkte zur Zielposition 1 (Lager)
  double service_timeout_;                  // max. Wartezeit auf die Antwort (s)
  double service_retry_interval_;           // Abstand zwischen zwei Versuchen (s)

  // --- Interner Zustand ---
  VisitorState state_ = VisitorState::FAHRE_ZU_WACHE;
  bool drive_started_ = false;      // wurde die aktuelle Fahrt schon gestartet?
  bool request_pending_ = false;    // laeuft gerade ein Service-Aufruf?
  bool response_received_ = false;  // ist eine Antwort eingetroffen?
  bool response_granted_ = false;   // Inhalt der Antwort: Zugang erlaubt?
  std::string response_message_;    // Inhalt der Antwort: Klartext
  rclcpp::Time request_sent_time_;  // wann wurde der Aufruf abgeschickt?
  rclcpp::Time last_retry_log_;     // zuletzt "Service nicht da"-Meldung geloggt
  int64_t pending_request_id_ = 0;  // ID des laufenden Aufrufs (fuer Abbruch)
  int attempt_ = 0;                 // Zaehler der Versuche (nur fuer Logs)
};

}  // namespace visitor_robot

#endif  // VISITOR_ROBOT__VISITOR_ROBOT_HPP_
