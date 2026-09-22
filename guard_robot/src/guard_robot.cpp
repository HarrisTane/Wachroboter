#include "guard_robot/guard_robot.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>

namespace guard_robot
{

GuardRobot::GuardRobot(const rclcpp::NodeOptions & options)
: Node("guard_robot", options)
{
  // ------------------------------------------------------------------
  // 1) Parameter deklarieren (mit sinnvollen Standardwerten)
  // ------------------------------------------------------------------

  // Liste der gueltigen Codewoerter (ROS-Parameter vom Typ string array)
  this->declare_parameter<std::vector<std::string>>(
    "valid_codewords", std::vector<std::string>{"apfelkuchen", "nordlicht"});

  // Wegpunkte zur "Zielposition 2" (Polizei) als flache Liste: x1,y1,x2,y2,...
  // Die Koordinaten sind relativ zur Startpose des Roboters in Metern.
  this->declare_parameter<std::vector<double>>(
    "police_waypoints", std::vector<double>{2.0, 0.0, 2.0, 2.0});

  // Wie lange (Sekunden) soll der Alarm laufen, bevor losgefahren wird?
  this->declare_parameter<double>("alarm_duration", 3.0);

  // Abstand zwischen zwei Alarm-Nachrichten in Sekunden
  this->declare_parameter<double>("alarm_interval", 0.5);

  // Text, der auf dem Topic /alarm publiziert wird
  this->declare_parameter<std::string>(
    "alarm_text", "ALARM: Falsches Codewort! Unbefugte Person erkannt.");

  // Soll beim Codewort-Vergleich Gross-/Kleinschreibung beachtet werden?
  this->declare_parameter<bool>("case_sensitive", false);

  // Frequenz der Zustandsmaschine in Hz
  this->declare_parameter<double>("state_frequency", 10.0);

  // ------------------------------------------------------------------
  // 2) Parameter auslesen
  // ------------------------------------------------------------------
  valid_codewords_ = this->get_parameter("valid_codewords").as_string_array();
  police_waypoints_flat_ = this->get_parameter("police_waypoints").as_double_array();
  alarm_duration_ = this->get_parameter("alarm_duration").as_double();
  alarm_interval_ = this->get_parameter("alarm_interval").as_double();
  alarm_text_ = this->get_parameter("alarm_text").as_string();
  case_sensitive_ = this->get_parameter("case_sensitive").as_bool();
  double state_frequency = this->get_parameter("state_frequency").as_double();

  // Plausibilitaetspruefung: die Wegpunktliste muss aus x/y-Paaren bestehen
  if (police_waypoints_flat_.size() % 2 != 0) {
    RCLCPP_ERROR(this->get_logger(),
      "Parameter 'police_waypoints' hat eine ungerade Anzahl an Werten (%zu). "
      "Der letzte Wert wird ignoriert.", police_waypoints_flat_.size());
  }
  if (police_waypoints_flat_.empty()) {
    RCLCPP_WARN(this->get_logger(),
      "Parameter 'police_waypoints' ist leer - der Roboter wuerde im Alarmfall "
      "nicht losfahren.");
  }
  if (valid_codewords_.empty()) {
    RCLCPP_WARN(this->get_logger(),
      "Parameter 'valid_codewords' ist leer - JEDES Codewort wird abgelehnt.");
  }

  // ------------------------------------------------------------------
  // 3) ROS-Schnittstellen anlegen
  // ------------------------------------------------------------------

  // Service "check_codeword". Relativer Name -> ohne Namespace wird daraus
  // automatisch "/check_codeword" (genau wie in der Aufgabenstellung).
  codeword_service_ = this->create_service<wachroboter_interfaces::srv::CheckCodeword>(
    "check_codeword",
    std::bind(&GuardRobot::handleCheckCodeword, this,
      std::placeholders::_1, std::placeholders::_2));

  // Publisher fuer die Alarm-Warnung (std_msgs/String auf Topic "alarm")
  alarm_pub_ = this->create_publisher<std_msgs::msg::String>("alarm", 10);

  // ------------------------------------------------------------------
  // 4) Timer fuer die Zustandsmaschine starten
  // ------------------------------------------------------------------
  auto period = std::chrono::duration<double>(1.0 / state_frequency);
  state_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&GuardRobot::stateLoop, this));

  // Zeitstempel initialisieren (wird beim Alarm neu gesetzt)
  alarm_start_ = this->now();
  last_alarm_pub_ = this->now();

  RCLCPP_INFO(this->get_logger(),
    "guard_robot gestartet. %zu gueltige Codewoerter, %zu Wegpunkte zur Polizei.",
    valid_codewords_.size(), police_waypoints_flat_.size() / 2);
  RCLCPP_INFO(this->get_logger(), "Zustand: %s", stateToString(state_));
}

// Der Fahr-Node wird von main() nachtraeglich uebergeben
void GuardRobot::setDriver(std::shared_ptr<waypoint_driver::WaypointDriver> driver)
{
  driver_ = driver;
  // Sicherheitshalber: der Fahr-Node darf am Anfang auf keinen Fall fahren
  if (driver_) {
    driver_->stop();
  }
}

// Zustand als lesbarer Text (fuer die Log-Ausgaben)
const char * GuardRobot::stateToString(GuardState state)
{
  switch (state) {
    case GuardState::WARTEN: return "WARTEN";
    case GuardState::ALARM: return "ALARM";
    case GuardState::FAHRE_ZU_POLIZEI: return "FAHRE_ZU_POLIZEI";
    case GuardState::FERTIG: return "FERTIG";
  }
  return "UNBEKANNT";
}

// Einziger Ort, an dem der Zustand geaendert wird -> jeder Wechsel wird geloggt
void GuardRobot::changeState(GuardState new_state)
{
  if (new_state == state_) {
    return;  // kein echter Wechsel, nichts zu tun
  }
  RCLCPP_INFO(this->get_logger(), "Zustandswechsel: %s -> %s",
    stateToString(state_), stateToString(new_state));
  state_ = new_state;
}

// Codewort gegen die Parameterliste pruefen
bool GuardRobot::isCodewordValid(const std::string & codeword) const
{
  // Hilfs-Lambda: wandelt einen String in Kleinbuchstaben um
  auto to_lower = [](std::string s) {
      std::transform(s.begin(), s.end(), s.begin(),
        [](unsigned char c) {return static_cast<char>(std::tolower(c));});
      return s;
    };

  // Bei case_sensitive_=false wird alles klein geschrieben verglichen
  const std::string input = case_sensitive_ ? codeword : to_lower(codeword);

  for (const auto & valid : valid_codewords_) {
    const std::string ref = case_sensitive_ ? valid : to_lower(valid);
    if (input == ref) {
      return true;  // Treffer gefunden
    }
  }
  return false;  // kein Codewort der Liste passt
}

// Warnung auf dem Topic "alarm" publizieren
void GuardRobot::publishAlarm()
{
  std_msgs::msg::String msg;
  msg.data = alarm_text_;
  alarm_pub_->publish(msg);
  RCLCPP_WARN(this->get_logger(), "%s", alarm_text_.c_str());
}

// ---------------------------------------------------------------------
// Service-Callback: hier kommt die Codewort-Anfrage an
// ---------------------------------------------------------------------
void GuardRobot::handleCheckCodeword(
  const std::shared_ptr<wachroboter_interfaces::srv::CheckCodeword::Request> request,
  std::shared_ptr<wachroboter_interfaces::srv::CheckCodeword::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Codewort-Anfrage erhalten: '%s'",
    request->codeword.c_str());

  // Wenn der Alarm schon laeuft oder wir unterwegs sind, wird nicht mehr geprueft.
  if (state_ != GuardState::WARTEN) {
    response->granted = false;
    response->message = "Roboter ist nicht im Zustand WARTEN (aktuell: " +
      std::string(stateToString(state_)) + ").";
    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  // --- Fall 1: Codewort korrekt ---
  if (isCodewordValid(request->codeword)) {
    response->granted = true;
    response->message = "Zugang gewaehrt";
    RCLCPP_INFO(this->get_logger(), "Zugang gewährt");
    // Zustand bleibt WARTEN, der Roboter bleibt stehen.
    return;
  }

  // --- Fall 2: Codewort falsch ---
  response->granted = false;
  response->message = "Zugang verweigert - Codewort falsch. Alarm ausgeloest.";
  RCLCPP_ERROR(this->get_logger(), "Zugang verweigert: Codewort '%s' ist ungueltig.",
    request->codeword.c_str());

  // Alarmphase starten: Startzeit merken und in den Zustand ALARM wechseln
  alarm_start_ = this->now();
  // last_alarm_pub_ so setzen, dass sofort die erste Warnung rausgeht
  last_alarm_pub_ = alarm_start_ - rclcpp::Duration::from_seconds(alarm_interval_);
  changeState(GuardState::ALARM);
}

// ---------------------------------------------------------------------
// Zustandsmaschine: wird periodisch vom Timer aufgerufen
// ---------------------------------------------------------------------
void GuardRobot::stateLoop()
{
  switch (state_) {
    // -----------------------------------------------------------------
    case GuardState::WARTEN:
      // Nichts tun: der Roboter steht und wartet auf eine Service-Anfrage.
      // Der Fahr-Node ist inaktiv und sendet Geschwindigkeit 0.
      break;

    // -----------------------------------------------------------------
    case GuardState::ALARM: {
      // Wie viele Sekunden laeuft der Alarm schon?
      const double elapsed = (this->now() - alarm_start_).seconds();

      // Alarmdauer abgelaufen -> losfahren
      if (elapsed >= alarm_duration_) {
        if (!driver_) {
          RCLCPP_ERROR(this->get_logger(),
            "Kein waypoint_driver vorhanden - kann nicht losfahren!");
          changeState(GuardState::FERTIG);
          break;
        }

        // Flache Parameterliste (x1,y1,x2,y2,...) in Wegpunkte umwandeln
        std::vector<waypoint_driver::Waypoint> wps;
        for (std::size_t i = 0; i + 1 < police_waypoints_flat_.size(); i += 2) {
          waypoint_driver::Waypoint wp;
          wp.x = police_waypoints_flat_[i];
          wp.y = police_waypoints_flat_[i + 1];
          wps.push_back(wp);
        }

        // Wegpunkte an den Fahr-Node uebergeben und Fahren freigeben
        driver_->setWaypoints(wps);
        driver_->start();

        RCLCPP_INFO(this->get_logger(),
          "Alarm beendet (%.1f s). Fahre zur Zielposition 2 (Polizei).", elapsed);
        changeState(GuardState::FAHRE_ZU_POLIZEI);
        break;
      }

      // Alarm laeuft noch -> in festen Abstaenden die Warnung publizieren
      if ((this->now() - last_alarm_pub_).seconds() >= alarm_interval_) {
        publishAlarm();
        last_alarm_pub_ = this->now();
      }
      break;
    }

    // -----------------------------------------------------------------
    case GuardState::FAHRE_ZU_POLIZEI:
      // Das eigentliche Fahren macht der waypoint_driver.
      // Wir warten nur darauf, dass er "fertig" meldet.
      if (driver_ && driver_->isFinished()) {
        driver_->stop();  // Fahren wieder sperren, Roboter bleibt stehen
        RCLCPP_INFO(this->get_logger(), "Zielposition 2 (Polizei) erreicht.");
        changeState(GuardState::FERTIG);
      }
      break;

    // -----------------------------------------------------------------
    case GuardState::FERTIG:
      // Endzustand: Roboter steht an der Polizei, nichts mehr zu tun.
      break;
  }
}

}  // namespace guard_robot
