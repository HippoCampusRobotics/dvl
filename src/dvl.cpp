#include "dvl/dvl.hpp"

#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <rclcpp/logging.hpp>
#include <thread>

#include "dvl/json_protocol.hpp"

namespace dvl {

static inline rclcpp::Logger get_logger() { return rclcpp::get_logger("Dvl"); }

Dvl::Dvl(const std::string &ip_address, int port)
    : ip_address_(ip_address), port_(port) {}

Dvl::~Dvl() { close(socket_); }

bool Dvl::ConnectTCP(int tries, std::chrono::milliseconds delay) {
  for (int i = 0; i < tries; ++i) {
    if (ConnectTCP()) {
      return true;
    }
    std::this_thread::sleep_for(delay);
  }
  return false;
}

bool Dvl::ConnectTCP() {
  struct addrinfo hints;
  struct addrinfo *info = nullptr;

  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET;  // ipv4
  hints.ai_socktype = SOCK_STREAM;
  int status = getaddrinfo(ip_address_.c_str(), std::to_string(port_).c_str(),
                           &hints, &info);
  if (status != 0) {
    freeaddrinfo(info);
    RCLCPP_DEBUG(get_logger(), "getaddrinfo() failed: %s",
                 gai_strerror(status));
    return false;
  }
  socket_ = socket(info->ai_family, info->ai_socktype, info->ai_protocol);
  // make the socket non blocking to we do not stall our program.
  int opts;
  opts = fcntl(socket_, F_GETFD, NULL);
  fcntl(socket_, F_SETFL, opts | O_NONBLOCK);

  // let's check if connect immediately succeeds.
  if (connect(socket_, info->ai_addr, info->ai_addrlen) == 0) {
    freeaddrinfo(info);
    return true;
  }
  freeaddrinfo(info);

  // let's if we can poll successfully within a given timeout interval
  struct pollfd fd;
  fd.fd = socket_;
  fd.events = POLLOUT;
  int poll_res = poll(&fd, 1, 500);
  if (poll_res > 0) {
    socklen_t value_length;
    int value{0};
    if (getsockopt(socket_, SOL_SOCKET, SO_ERROR, (void *)&value,
                   &value_length) != 0) {
      RCLCPP_DEBUG(get_logger(), "Failed to read socket options.");
      return false;
    }
    if (value) {
      // since we read the socket error, this means we have an error.
      RCLCPP_DEBUG(get_logger(), "Failed to connect().");
      return false;
    }
    opts = fcntl(socket_, F_GETFD, NULL);
    opts &= ~(O_NONBLOCK);
    fcntl(socket_, F_SETFL, opts);
    return true;
  }
  if (poll_res == 0) {
    RCLCPP_DEBUG(get_logger(), "Connect() timed out.");
  }

  return false;
}

bool Dvl::SendCommand(Cmd cmd) {
  nlohmann::json json_data = {
      {"command", cmd.Name()},
  };
  return SendJSON(json_data);
}

bool Dvl::SendCommand(Cmd cmd, const nlohmann::json &parameters) {
  nlohmann::json json_data = {
      {"command", cmd.Name()},
  };
  json_data["parameters"] = parameters;
  return SendJSON(json_data);
}

bool Dvl::ResetDeadReckoning() {
  return SendCommand(Cmd::ResetDeadReckoning());
}

bool Dvl::SetSpeedOfSound(double speed_of_sound) {
  nlohmann::json parameters = {
      {"speed_of_sound", speed_of_sound},
  };
  return SendCommand(Cmd::SetConfig(), parameters);
}

bool Dvl::SetAcousticEnabled(bool enabled) {
  nlohmann::json parameters = {
      {"acoustic_enabled", enabled},
  };
  return SendCommand(Cmd::SetConfig(), parameters);
}

bool Dvl::SendGetConfig() { return SendCommand(Cmd::GetConfig()); }

bool Dvl::SendResetDeadReckoning() {
  return SendCommand(Cmd::ResetDeadReckoning());
}

bool Dvl::SendJSON(const nlohmann::json &_json_data) {
  std::string data = _json_data.dump();
  const char *p = data.c_str();
  ssize_t byte_count = send(socket_, p, data.size(), 0);
  if (byte_count < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to send data: %s", strerror(errno));
    return false;
  }
  if (static_cast<size_t>(byte_count) != data.size()) {
    RCLCPP_ERROR(get_logger(), "Should have sent %zu bytes, but sent %zd",
                 data.size(), byte_count);
    return false;
  }
  return true;
}

std::optional<nlohmann::json> Dvl::ReadJsonReport() {
  char buf = 0;
  std::string data;
  ssize_t n_bytes;
  // blocking receive. hence, zero bytes indicate closed socket. Otherwise the
  // call would block until data is received.
  do {
    n_bytes = recv(socket_, (void *)&buf, sizeof(buf), 0);
    if (n_bytes < 0) {
      if (errno == EBADF || errno == ENOTSOCK) {
        ConnectTCP(3, std::chrono::milliseconds(100));
        continue;
      }
      RCLCPP_DEBUG(get_logger(), "Failed to receive data: %s", strerror(errno));
      return std::nullopt;
    } else if (n_bytes == 0) {
      if (!ConnectTCP(3, std::chrono::milliseconds(100))) {
        RCLCPP_DEBUG(get_logger(), "Failed to open socket.");
        return std::nullopt;
      }
      continue;
    }
    data += buf;
  } while (buf != '\n');
  // we do not need the line break. used only as indicator for completed
  // reports.
  data.pop_back();
  nlohmann::json json_data;
  try {
    json_data = nlohmann::json::parse(data);
  } catch (nlohmann::json::parse_error &) {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to parse data");
    return std::nullopt;
  }
  if (!check_version(json_data)) {
    RCLCPP_FATAL_ONCE(get_logger(), "Expected format <%s> but is <%s>",
                      json_protocol_version, get_version(json_data).c_str());
    return std::nullopt;
  }

  return json_data;
}
}  // namespace dvl
