#include "visitor_robot/visitor_robot.hpp"

#include <chrono>

namespace visitor_robot
{

VisitorRobot::VisitorRobot(const rclcpp::NodeOptions & options)
: Node("visitor_robot", options)
{
  // ------------------------------------------------------------------
  // 1) Parameter deklarieren (mit sinnvollen Standardwerten)
  // ------------------------------------------------------------------

  // Codewort, das an den Wachroboter geschickt wird.
  // Zum Testen einfach umschalten: -p codeword:="banane"
  this->declare_parameter<std::string>("codeword", "apfelkuchen");

  // Name des Services. Absoluter Name (fuehrender /), damit er auch dann
  // gefunden wird, wenn dieser Roboter in einem Namespace laeuft.
  this->declare_parameter<std::string>("service_name", "/check_codeword");

  // Wegpunkte bis kurz vor den Wachroboter, flache Liste: x1,y1,x2,y2,...
  // Koordinaten relativ zur STARTPOSE dieses Roboters, in Metern.
  this->declare_parameter<std::vector<double>>(
    "wache_waypoints", std::vector<double>{1.5, 0.0});

  // Wegpunkte zur Zielposition 1 (Lager), ebenfalls relativ zur STARTPOSE.
  this->declare_parameter<std::vector<double>>(
    "lager_waypoints", std::vector<double>{1.5, 2.0, 0.0, 2.0});

  // Wie lange warten wir maximal auf die Antwort des Services?
  this->declare_parameter<double>("service_timeout", 5.0);

  // Wie oft probieren wir es erneut, wenn der Service noch nicht da ist?
  this->declare_parameter<double>("service_retry_interval", 2.0);

  // Takt der Zustandsmaschine in Hz
  this->declare_parameter<double>("state_frequency", 10.0);

  // ------------------------------------------------------------------
  // 2) Parameter auslesen
  // ------------------------------------------------------------------
  codeword_ = this->get_parameter("codeword").as_string();
  service_name_ = this->get_parameter("service_name").as_string();
  wache_waypoints_ = this->get_parameter("wache_waypoints").as_double_array();
  lager_waypoints_ = this->get_parameter("lager_waypoints").as_double_array();
  service_timeout_ = this->get_parameter("service_timeout").as_double();
  service_retry_interval_ = this->get_parameter("service_retry_interval").as_double();
  double state_frequency = this->get_parameter("state_frequency").as_double();

  // Plausibilitaetspruefungen: beide Listen muessen aus x/y-Paaren bestehen
  if (wache_waypoints_.size() % 2 != 0) {
    RCLCPP_ERROR(this->get_logger(),
      "Parameter 'wache_waypoints' hat eine ungerade Anzahl an Werten (%zu). "
      "Der letzte Wert wird ignoriert.", wache_waypoints_.size());
  }
  if (lager_waypoints_.size() % 2 != 0) {
    RCLCPP_ERROR(this->get_logger(),
      "Parameter 'lager_waypoints' hat eine ungerade Anzahl an Werten (%zu). "
      "Der letzte Wert wird ignoriert.", lager_waypoints_.size());
  }
  if (wache_waypoints_.empty()) {
    RCLCPP_WARN(this->get_logger(),
      "Parameter 'wache_waypoints' ist leer - der Roboter fragt sofort das "
      "Codewort ab, ohne zu fahren.");
  }

  // ------------------------------------------------------------------
  // 3) Service-Client anlegen
  // ------------------------------------------------------------------
  codeword_client_ = this->create_client<wachroboter_interfaces::srv::CheckCodeword>(
    service_name_);

  // ------------------------------------------------------------------
  // 4) Timer fuer die Zustandsmaschine starten
  // ------------------------------------------------------------------
  auto period = std::chrono::duration<double>(1.0 / state_frequency);
  state_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&VisitorRobot::stateLoop, this));

  // Zeitstempel initialisieren
  request_sent_time_ = this->now();
  last_retry_log_ = this->now();

  RCLCPP_INFO(this->get_logger(),
    "visitor_robot gestartet. Codewort='%s', Service='%s', "
    "%zu Wegpunkte zur Wache, %zu Wegpunkte zum Lager.",
    codeword_.c_str(), service_name_.c_str(),
    wache_waypoints_.size() / 2, lager_waypoints_.size() / 2);
  RCLCPP_INFO(this->get_logger(), "Zustand: %s", stateToString(state_));
}

// Der Fahr-Node wird von main() nachtraeglich uebergeben
void VisitorRobot::setDriver(std::shared_ptr<waypoint_driver::WaypointDriver> driver)
{
  driver_ = driver;
  // Sicherheitshalber: der Fahr-Node darf nicht ungefragt losfahren.
  // Die Wegpunkte werden erst im ersten Durchlauf der Statemachine gesetzt.
  if (driver_) {
    driver_->stop();
  }
}

// Zustand als lesbarer Text (fuer die Log-Ausgaben)
const char * VisitorRobot::stateToString(VisitorState state)
{
  switch (state) {
    case VisitorState::FAHRE_ZU_WACHE: return "FAHRE_ZU_WACHE";
    case VisitorState::SENDE_CODEWORT: return "SENDE_CODEWORT";
    case VisitorState::FAHRE_ZU_LAGER: return "FAHRE_ZU_LAGER";
    case VisitorState::ZUGANG_VERWEIGERT: return "ZUGANG_VERWEIGERT";
    case VisitorState::FERTIG: return "FERTIG";
  }
  return "UNBEKANNT";
}

// Einziger Ort, an dem der Zustand geaendert wird -> jeder Wechsel wird geloggt
void VisitorRobot::changeState(VisitorState new_state)
{
  if (new_state == state_) {
    return;  // kein echter Wechsel, nichts zu tun
  }
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "Zustandswechsel: %s -> %s",
    stateToString(state_), stateToString(new_state));
  RCLCPP_INFO(this->get_logger(), "========================================");
  state_ = new_state;
  drive_started_ = false;  // neue Fahrt muss im neuen Zustand neu gestartet werden
}

// Flache Liste x1,y1,x2,y2,... in eine Wegpunktliste umwandeln
std::vector<waypoint_driver::Waypoint> VisitorRobot::toWaypoints(
  const std::vector<double> & flat) const
{
  std::vector<waypoint_driver::Waypoint> wps;
  for (std::size_t i = 0; i + 1 < flat.size(); i += 2) {
    waypoint_driver::Waypoint wp;
    wp.x = flat[i];
    wp.y = flat[i + 1];
    wps.push_back(wp);
  }
  return wps;
}

// Wegpunkte an den Fahr-Node uebergeben und Fahren freigeben
void VisitorRobot::startDriving(const std::vector<double> & flat, const std::string & ziel_name)
{
  if (!driver_) {
    RCLCPP_ERROR(this->get_logger(), "Kein waypoint_driver vorhanden - kann nicht fahren!");
    return;
  }
  auto wps = toWaypoints(flat);
  driver_->setWaypoints(wps);  // setzt Index/Fertig-Flag automatisch zurueck
  driver_->start();            // ab jetzt gibt der Fahr-Node Fahrbefehle aus
  RCLCPP_INFO(this->get_logger(), "Fahre los zu '%s' (%zu Wegpunkte).",
    ziel_name.c_str(), wps.size());
}

// ---------------------------------------------------------------------
// Zustand 1: FAHRE_ZU_WACHE
// ---------------------------------------------------------------------
void VisitorRobot::handleFahreZuWache()
{
  // Beim ersten Durchlauf die Fahrt starten
  if (!drive_started_) {
    startDriving(wache_waypoints_, "Wache");
    drive_started_ = true;
    return;
  }

  // Warten, bis der Fahr-Node alle Wegpunkte abgearbeitet hat
  if (driver_ && driver_->isFinished()) {
    driver_->stop();  // Roboter anhalten, waehrend wir fragen
    RCLCPP_INFO(this->get_logger(), "Position vor dem Wachroboter erreicht.");
    changeState(VisitorState::SENDE_CODEWORT);
  }
}

// Einen asynchronen Service-Aufruf absetzen
void VisitorRobot::sendRequest()
{
  // Anfrage-Objekt bauen und das Codewort eintragen
  auto request = std::make_shared<wachroboter_interfaces::srv::CheckCodeword::Request>();
  request->codeword = codeword_;

  attempt_++;
  RCLCPP_INFO(this->get_logger(), "Sende Codewort '%s' an '%s' (Versuch %d)...",
    codeword_.c_str(), service_name_.c_str(), attempt_);

  // ASYNCHRON senden: der Aufruf blockiert NICHT. Die Antwort kommt spaeter
  // im Lambda unten an. (Blockierendes spin_until_future_complete waere hier
  // ein Deadlock, weil wir uns schon innerhalb eines Timer-Callbacks befinden.)
  auto future_and_id = codeword_client_->async_send_request(
    request,
    [this](rclcpp::Client<wachroboter_interfaces::srv::CheckCodeword>::SharedFuture future) {
      auto response = future.get();
      // Ergebnis nur zwischenspeichern - ausgewertet wird es in stateLoop().
      response_granted_ = response->granted;
      response_message_ = response->message;
      response_received_ = true;
      request_pending_ = false;
      RCLCPP_INFO(this->get_logger(), "Antwort erhalten: granted=%s, message='%s'",
        response_granted_ ? "true" : "false", response_message_.c_str());
    });

  // ID merken, damit wir den Aufruf bei einem Timeout sauber verwerfen koennen
  pending_request_id_ = future_and_id.request_id;
  request_pending_ = true;
  request_sent_time_ = this->now();
}

// ---------------------------------------------------------------------
// Zustand 2: SENDE_CODEWORT (senden + auf Antwort warten + wiederholen)
// ---------------------------------------------------------------------
void VisitorRobot::handleSendeCodewort()
{
  // --- Fall A: Antwort ist da -> auswerten ---
  if (response_received_) {
    response_received_ = false;

    if (response_granted_) {
      RCLCPP_INFO(this->get_logger(), "Zugang gewaehrt vom Wachroboter.");
      changeState(VisitorState::FAHRE_ZU_LAGER);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Zugang verweigert vom Wachroboter: %s",
        response_message_.c_str());
      changeState(VisitorState::ZUGANG_VERWEIGERT);
    }
    return;
  }

  // --- Fall B: Ein Aufruf laeuft gerade -> nur auf Timeout pruefen ---
  if (request_pending_) {
    const double waiting = (this->now() - request_sent_time_).seconds();
    if (waiting >= service_timeout_) {
      // Antwort kam nicht rechtzeitig: laufenden Aufruf verwerfen,
      // damit eine spaete Antwort uns nicht durcheinanderbringt.
      codeword_client_->remove_pending_request(pending_request_id_);
      request_pending_ = false;
      RCLCPP_WARN(this->get_logger(),
        "Timeout nach %.1f s - keine Antwort vom Wachroboter. Neuer Versuch.",
        service_timeout_);
    }
    return;
  }

  // --- Fall C: Kein Aufruf laeuft -> Service verfuegbar? ---
  // wait_for_service mit Dauer 0 fragt nur nach, blockiert also nicht.
  if (!codeword_client_->wait_for_service(std::chrono::seconds(0))) {
    // Service noch nicht da -> in Ruhe erneut probieren (nicht jeden Takt loggen)
    if ((this->now() - last_retry_log_).seconds() >= service_retry_interval_) {
      RCLCPP_WARN(this->get_logger(),
        "Service '%s' noch nicht verfuegbar - warte...", service_name_.c_str());
      last_retry_log_ = this->now();
    }
    return;
  }

  // --- Fall D: Service ist da -> Anfrage abschicken ---
  sendRequest();
}

// ---------------------------------------------------------------------
// Zustand 3: FAHRE_ZU_LAGER
// ---------------------------------------------------------------------
void VisitorRobot::handleFahreZuLager()
{
  // Beim ersten Durchlauf die Fahrt starten
  if (!drive_started_) {
    startDriving(lager_waypoints_, "Lager (Zielposition 1)");
    drive_started_ = true;
    return;
  }

  // Warten, bis alle Wegpunkte abgefahren sind
  if (driver_ && driver_->isFinished()) {
    driver_->stop();
    RCLCPP_INFO(this->get_logger(), "Zielposition 1 (Lager) erreicht.");
    changeState(VisitorState::FERTIG);
  }
}

// ---------------------------------------------------------------------
// Zustandsmaschine: wird periodisch vom Timer aufgerufen
// ---------------------------------------------------------------------
void VisitorRobot::stateLoop()
{
  switch (state_) {
    case VisitorState::FAHRE_ZU_WACHE:
      handleFahreZuWache();
      break;

    case VisitorState::SENDE_CODEWORT:
      handleSendeCodewort();
      break;

    case VisitorState::FAHRE_ZU_LAGER:
      handleFahreZuLager();
      break;

    case VisitorState::ZUGANG_VERWEIGERT:
      // Endzustand: Roboter bleibt stehen. Einmal pro 5 s daran erinnern.
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "ZUGANG_VERWEIGERT - Roboter bleibt stehen.");
      break;

    case VisitorState::FERTIG:
      // Endzustand: Roboter steht im Lager, nichts mehr zu tun.
      break;
  }
}

}  // namespace visitor_robot
