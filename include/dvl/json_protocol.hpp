#pragma once
#include <dvl_msgs/msg/config_result.hpp>
#include <dvl_msgs/msg/dead_reckoning_report.hpp>
#include <dvl_msgs/msg/velocity_transducer_report.hpp>
#include <dvl_msgs/srv/get_config.hpp>
#include <nlohmann/json.hpp>
#include <optional>
#include <std_srvs/srv/trigger.hpp>

namespace dvl {

static constexpr char json_protocol_version[] = "json_v3.1";

namespace cmd {
static constexpr char kResetDeadReckoning[] = "reset_dead_reckoning";
static constexpr char kCalibrateGyro[] = "calibrate_gyro";
static constexpr char kTriggerPing[] = "trigger_ping";
static constexpr char kGetConfig[] = "get_config";
static constexpr char kSetConfig[] = "set_config";
}  // namespace cmd

namespace CmdId {
enum CmdId {
  kResetDeadReckoning = 0,
  kCalibrateGyro,
  kTriggerPing,
  kGetConfig,
  kSetConfig,
};
}

class Cmd {
 public:
  static constexpr Cmd ResetDeadReckoning() {
    return Cmd(CmdId::kResetDeadReckoning);
  }
  static Cmd CalibrateGyro() { return Cmd(CmdId::kCalibrateGyro); }
  static Cmd TriggerPing() { return Cmd(CmdId::kTriggerPing); }
  static Cmd GetConfig() { return Cmd(CmdId::kGetConfig); }
  static Cmd SetConfig() { return Cmd(CmdId::kSetConfig); }
  static Cmd FromName(const char *name) { return Cmd(name_to_id_.at(name)); }

  std::string Name() const { return id_to_name_.at(cmd_id_); }
  inline constexpr CmdId::CmdId CmdID() const { return cmd_id_; }

  bool operator==(const char *rhs) { return Name() == rhs; }
  bool operator==(const Cmd &rhs) { return cmd_id_ == rhs.CmdID(); }
  bool operator==(const int &rhs) { return cmd_id_ == rhs; }

 private:
  constexpr explicit Cmd(CmdId::CmdId cmd) : cmd_id_(cmd) {}
  inline static const std::map<std::string, CmdId::CmdId> name_to_id_ = {
      {"reset_dead_reckoning", CmdId::kResetDeadReckoning},
      {"calibrate_gyro", CmdId::kCalibrateGyro},
      {"trigger_ping", CmdId::kTriggerPing},
      {"get_config", CmdId::kGetConfig},
      {"set_config", CmdId::kSetConfig}};
  inline static const std::map<CmdId::CmdId, std::string> id_to_name_ = {
      {CmdId::kResetDeadReckoning, "reset_dead_reckoning"},
      {CmdId::kCalibrateGyro, "calibrate_gyro"},
      {CmdId::kTriggerPing, "trigger_ping"},
      {CmdId::kGetConfig, "get_config"},
      {CmdId::kSetConfig, "set_config"}};
  const CmdId::CmdId cmd_id_;
};

inline bool check_version(const nlohmann::json &data) {
  return data["format"] == json_protocol_version;
}
inline std::string get_version(const nlohmann::json &data) {
  std::string version = data["format"];
  return version;
}

inline bool is_velocity_transducer_report(const nlohmann::json &data) {
  return data["type"] == "velocity";
}

inline bool is_dead_reckoning_report(const nlohmann::json &data) {
  return data["type"] == "position_local";
}

inline bool is_command_response(const nlohmann::json &data) {
  return data["type"] == "response";
}

inline bool is_command_response(const nlohmann::json &data, std::string cmd) {
  return (data["type"] == "response") && (data["response_to"] == cmd);
}

inline Cmd get_command_type(const nlohmann::json &data) {
  auto type = data["type"];
  if (type == Cmd::SetConfig().Name()) {
    return Cmd::SetConfig();
  } else if (type == Cmd::GetConfig().Name()) {
    return Cmd::GetConfig();
  } else if (type == Cmd::CalibrateGyro().Name()) {
    return Cmd::CalibrateGyro();
  } else if (type == Cmd::ResetDeadReckoning().Name()) {
    return Cmd::ResetDeadReckoning();
  } else if (type == Cmd::TriggerPing().Name()) {
    return Cmd::TriggerPing();
  } else {
    assert(false);
  }
}
inline CmdId::CmdId get_command_id(const nlohmann::json &data) {
  auto type = data["type"];
  if (type == Cmd::SetConfig().Name()) {
    return Cmd::SetConfig().CmdID();
  } else if (type == Cmd::GetConfig().Name()) {
    return Cmd::GetConfig().CmdID();
  } else if (type == Cmd::CalibrateGyro().Name()) {
    return Cmd::CalibrateGyro().CmdID();
  } else if (type == Cmd::ResetDeadReckoning().Name()) {
    return Cmd::ResetDeadReckoning().CmdID();
  } else if (type == Cmd::TriggerPing().Name()) {
    return Cmd::TriggerPing().CmdID();
  } else {
    assert(false);
  }
}

inline bool is_success(const nlohmann::json &data) { return data["success"]; }

inline std::string response_message(const nlohmann::json &data) {
  return data["error_message"];
}

inline dvl_msgs::srv::GetConfig::Response::SharedPtr parse_get_config(
    const nlohmann::json &data) {
  if (!is_command_response(data, cmd::kGetConfig)) {
    return nullptr;
  }
  auto response = std::make_shared<dvl_msgs::srv::GetConfig::Response>();
  response->success = is_success(data);
  response->message = response_message(data);
  if (!response->success) {
    return response;
  }
  auto &json_result = data["result"];
  dvl_msgs::msg::ConfigResult result;
  result.speed_of_sound = json_result["speed_of_sound"];
  result.acoustic_enabled = json_result["acoustic_enabled"];
  result.dark_mode_enabled = json_result["dark_mode_enabled"];
  result.mounting_rotation_offset = json_result["mounting_rotation_offset"];
  result.range_mode = json_result["range_mode"];
  // result.periodic_cyling_enabled = json_result["periodic_cycling_enabled"];
  response->result = result;
  return response;
}

inline std_srvs::srv::Trigger::Response::SharedPtr parse_reset_dead_reckoning(
    const nlohmann::json &data) {
  if (!is_command_response(data, cmd::kResetDeadReckoning)) {
    return nullptr;
  }
  auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
  response->success = is_success(data);
  response->message = response_message(data);
  return response;
}

inline std::optional<dvl_msgs::msg::VelocityTransducerReport>
parse_velocity_transducer_report(const nlohmann::json &data) {
  dvl_msgs::msg::VelocityTransducerReport msg;
  if (!is_velocity_transducer_report(data)) {
    return std::nullopt;
  }
  auto &v = msg.velocity;

  v.velocity.x = data["vx"];
  v.velocity.y = data["vy"];
  v.velocity.z = data["vz"];

  v.figure_of_merit = data["fom"];
  v.valid = data["velocity_valid"];
  v.covariance = data["covariance"];
  v.altitude = data["altitude"];
  v.status = data["status"];
  v.time_of_validity = data["time_of_validity"];
  v.time_of_transmission = data["time_of_transmission"];

  dvl_msgs::msg::Transducer t_report;
  for (const auto &transducer : data["transducers"]) {
    t_report.id = transducer["id"];
    t_report.velocity = transducer["velocity"];
    t_report.distance = transducer["distance"];
    t_report.rssi = transducer["rssi"];
    t_report.nsd = transducer["nsd"];
    msg.transducers.push_back(t_report);
  }
  return msg;
}

inline std::optional<dvl_msgs::msg::DeadReckoningReport>
parse_dead_reckoning_report(const nlohmann::json &data) {
  if (!is_dead_reckoning_report(data)) {
    return std::nullopt;
  }
  dvl_msgs::msg::DeadReckoningReport msg;
  msg.report.time_stamp = data["ts"];
  msg.report.distance.x = data["x"];
  msg.report.distance.y = data["y"];
  msg.report.distance.z = data["z"];

  msg.report.stddev = data["std"];
  msg.report.rpy.x = data["roll"];
  msg.report.rpy.y = data["pitch"];
  msg.report.rpy.z = data["yaw"];
  msg.report.status = data["status"];
  return msg;
}

}  // namespace dvl
