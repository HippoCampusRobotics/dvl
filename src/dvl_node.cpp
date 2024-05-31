#include "dvl_node.hpp"

namespace dvl {
DvlNode::DvlNode(const rclcpp::NodeOptions &_options)
    : Node("dvl", _options),
      get_config_handler_(*this, Cmd::GetConfig()),
      reset_dead_reckoning_handler_(*this, Cmd::ResetDeadReckoning()) {
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

void DvlNode::InitServices() {}

void DvlNode::Run() {
  if (!dvl_) {
    RCLCPP_INFO(get_logger(), "Creating DVL interface.");
    dvl_ = std::make_shared<Dvl>("192.168.0.126", 16171);
    if (!dvl_) {
      RCLCPP_ERROR(get_logger(), "Failed to create instace of DVL");
      return;
    }
    RCLCPP_INFO(get_logger(), "DVL interface created.");
    get_config_handler_.SetDVL(dvl_);
    reset_dead_reckoning_handler_.SetDVL(dvl_);
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
  const CmdId::CmdId cmd_id = get_command_id(_data);
  switch (cmd_id) {
    case CmdId::kGetConfig: {
      auto response = parse_get_config(_data);
      get_config_handler_.OnDVLResponse(response);
      return;
    }
    case CmdId::kResetDeadReckoning: {
      auto response = parse_reset_dead_reckoning(_data);
      reset_dead_reckoning_handler_.OnDVLResponse(response);
      return;
    }
  }
  RCLCPP_WARN(get_logger(), "Unhandled command response: \n%s",
              _data.dump(4).c_str());
}

}  // namespace dvl

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<dvl::DvlNode>(options);
  rclcpp::spin(node);
}
