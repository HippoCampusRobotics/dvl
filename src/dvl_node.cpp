#include "dvl_node.hpp"

namespace dvl {
DvlNode::DvlNode(const rclcpp::NodeOptions &_options)
    : Node("dvl", _options)

{
  InitParams();
  InitPublishers();
  InitServices();

  run_timer_ = create_timer(std::chrono::milliseconds(10), [this]() { Run(); });
}

void DvlNode::InitPublishers() {
  std::string name;
  rclcpp::QoS qos = rclcpp::SensorDataQoS().keep_last(1);

  name = "~/velocity_transducer_report";
  velocity_transducer_pub_ =
      create_publisher<dvl_msgs::msg::VelocityTransducerReport>(name, qos);

  name = "~/dead_reckoning_report";
  dead_reckoning_pub_ =
      create_publisher<dvl_msgs::msg::DeadReckoningReport>(name, qos);
}

void DvlNode::InitServices() {
  std::string name;
  std::string key;

  key = "get_config";
  name = "~/" + key;
  service_handlers_[key] = ServiceHandler<dvl_msgs::srv::GetConfig>(this, name);

  key = "reset_dead_reckoning";
  name = "~/" + key;
  service_handlers_[key] =
      ServiceHandler<dvl_msgs::srv::ResetDeadReckoning>(this, name);

  key = "set_acoustic_enabled";
  name = "~/" + key;
  service_handlers_[key] =
      ServiceHandler<dvl_msgs::srv::SetAcousticEnabled>(this, name);

  key = "set_speed_of_sound";
  name = "~/" + key;
  service_handlers_[key] =
      ServiceHandler<dvl_msgs::srv::SetSpeedOfSound>(this, name);
}

void DvlNode::Run() {
  if (!dvl_) {
    RCLCPP_INFO(get_logger(), "Creating DVL interface at: %s:%d",
                params_.ip_address.c_str(), params_.port);
    dvl_ = std::make_shared<Dvl>(params_.ip_address, params_.port);
    if (!dvl_) {
      RCLCPP_ERROR(get_logger(), "Failed to create instace of DVL");
      return;
    }
    RCLCPP_INFO(get_logger(), "DVL interface created.");
    for (auto const &[key, value] : service_handlers_) {
      std::visit([this](auto &handler) { handler.SetDVL(dvl_); },
                 service_handlers_.at(key));
    }
  }

  auto report = dvl_->ReadJsonReport();
  if (!report) {
    dvl_.reset();
    return;
  }
  if (is_dead_reckoning_report(*report)) {
    auto msg = parse_dead_reckoning_report(*report);
    if (msg) {
      dead_reckoning_pub_->publish(*msg);
    }
  } else if (is_velocity_transducer_report(*report)) {
    auto msg = parse_velocity_transducer_report(*report);
    if (msg) {
      velocity_transducer_pub_->publish(*msg);
    } else {
      RCLCPP_INFO(get_logger(), "Failed to parse velocity report.");
    }
  } else if (is_command_response(*report)) {
    HandleCommandResponse(*report);
  }
}

void DvlNode::HandleCommandResponse(const nlohmann::json &_data) {
  using namespace dvl_msgs::srv;
  const CmdId::CmdId cmd_id = get_command_id(_data);
  switch (cmd_id) {
    case CmdId::kGetConfig: {
      auto response = parse_get_config(_data);
      std::visit(
          [&response](auto &handler) {
            using T = std::decay_t<decltype(handler)>;
            if constexpr (std::is_same_v<ServiceHandler<GetConfig>, T>) {
              handler.OnDVLResponse(response);
            }
          },
          service_handlers_.at("get_config"));
      return;
    }
    case CmdId::kResetDeadReckoning: {
      auto response = parse_reset_dead_reckoning(_data);
      std::visit(
          [&response](auto &handler) {
            using T = std::decay_t<decltype(handler)>;
            if constexpr (std::is_same_v<ServiceHandler<ResetDeadReckoning>,
                                         T>) {
              handler.OnDVLResponse(response);
            }
          },
          service_handlers_.at("reset_dead_reckoning"));
      return;
    }
    case CmdId::kSetConfig: {
      HandleSetConfigResponse(_data);
      return;
    }
  }
  RCLCPP_WARN(get_logger(), "Unhandled command response: \n%s",
              _data.dump(4).c_str());
}
void DvlNode::HandleSetConfigResponse(const nlohmann::json &_data) {
  using namespace dvl_msgs::srv;
  const nlohmann::json &params = _data["parameters"];
  if (params.contains("speed_of_sound")) {
    auto response = parse_set_speed_of_sound(_data);
    std::visit(
        [&response](auto &handler) {
          using T = std::decay_t<decltype(handler)>;
          if constexpr (std::is_same_v<ServiceHandler<SetSpeedOfSound>, T>) {
            handler.OnDVLResponse(response);
          }
        },
        service_handlers_.at("set_speed_of_sound"));
  } else if (params.contains("acoustic_enabled")) {
    auto response = parse_set_acoustic_enabled(_data);
    std::visit(
        [&response](auto &handler) {
          using T = std::decay_t<decltype(handler)>;
          if constexpr (std::is_same_v<ServiceHandler<SetAcousticEnabled>, T>) {
            handler.OnDVLResponse(response);
          }
        },
        service_handlers_.at("set_acoustic_enabled"));
  } else {
    // do nothing?
    RCLCPP_INFO(get_logger(), "Unhandled parameter for set_config response.");
  }
}

}  // namespace dvl

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<dvl::DvlNode>(options);
  rclcpp::spin(node);
}
