#pragma once
#include <dvl_msgs/msg/dead_reckoning_report.hpp>
#include <dvl_msgs/msg/velocity_transducer_report.hpp>
#include <dvl_msgs/srv/get_config.hpp>
#include <dvl_msgs/srv/reset_dead_reckoning.hpp>
#include <dvl_msgs/srv/set_acoustic_enabled.hpp>
#include <dvl_msgs/srv/set_speed_of_sound.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dvl/dvl.hpp"
#include "service_handler.hpp"

namespace dvl {

class DvlNode : public rclcpp::Node {
 public:
  explicit DvlNode(const rclcpp::NodeOptions &options);
  void InitPublishers();
  void InitServices();
  void InitParams();
  void Run();

 private:
  struct Params {
    std::string ip_address;
    int port;
  };

  void HandleCommandResponse(const nlohmann::json &data);
  void HandleSetConfigResponse(const nlohmann::json &data);
  void OnConfigServiceCalled();
  bool IsServiceAllowedToRun(const std::string &name);

  std::shared_ptr<Dvl> dvl_;
  rclcpp::Publisher<dvl_msgs::msg::VelocityTransducerReport>::SharedPtr
      velocity_transducer_pub_;
  rclcpp::Publisher<dvl_msgs::msg::DeadReckoningReport>::SharedPtr
      dead_reckoning_pub_;
  rclcpp::TimerBase::SharedPtr run_timer_;

  ServiceHandler<dvl_msgs::srv::GetConfig> get_config_handler_;
  ServiceHandler<dvl_msgs::srv::ResetDeadReckoning>
      reset_dead_reckoning_handler_;
  ServiceHandler<dvl_msgs::srv::SetAcousticEnabled>
      set_acoustic_enabled_handler_;
  ServiceHandler<dvl_msgs::srv::SetSpeedOfSound> set_speed_of_sound_handler_;
  Params params_;
  bool initial_run_{true};
};
}  // namespace dvl
