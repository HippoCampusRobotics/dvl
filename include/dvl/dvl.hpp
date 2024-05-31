#pragma once
#include <sys/socket.h>

#include <dvl_msgs/msg/velocity_transducer_report.hpp>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "dvl/json_protocol.hpp"

namespace dvl {
class Dvl {
 public:
  Dvl(const std::string &ip_address, int port);
  ~Dvl();
  std::optional<nlohmann::json> ReadJsonReport();
  bool SendCommand(Cmd);
  bool SendCommand(Cmd, const nlohmann::json &parameters);
  bool ResetDeadReckoning();
  bool SetSpeedOfSound(double);
  bool SetAcousticEnabled(bool);
  bool SendGetConfig();
  bool SendResetDeadReckoning();

 private:
  bool SendJSON(const nlohmann::json &data);
  bool ConnectTCP();
  bool ConnectSerial();
  static constexpr int kBufferSize = 1024;
  std::string ip_address_;
  int port_;
  int socket_;
};
}  // namespace dvl
