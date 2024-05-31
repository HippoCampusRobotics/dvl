#pragma once
#include <dvl_msgs/msg/dead_reckoning_report.hpp>
#include <dvl_msgs/msg/velocity_transducer_report.hpp>
#include <dvl_msgs/srv/get_config.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dvl/dvl.hpp"
#include "dvl/json_protocol.hpp"
#include "service_handler.hpp"

namespace dvl {

class DvlNode : public rclcpp::Node {
 public:
  explicit DvlNode(const rclcpp::NodeOptions &options);
  void InitPublishers();
  void InitServices();
  void Run();

 private:
  void HandleCommandResponse(const nlohmann::json &data);

  std::shared_ptr<Dvl> dvl_;
  rclcpp::Publisher<dvl_msgs::msg::VelocityTransducerReport>::SharedPtr
      velocity_transducer_pub_;
  rclcpp::Publisher<dvl_msgs::msg::DeadReckoningReport>::SharedPtr
      dead_reckoning_pub_;
  rclcpp::TimerBase::SharedPtr run_timer_;

  ServiceHandler<dvl_msgs::srv::GetConfig> get_config_handler_;
  ServiceHandler<std_srvs::srv::Trigger> reset_dead_reckoning_handler_;
};
}  // namespace dvl
