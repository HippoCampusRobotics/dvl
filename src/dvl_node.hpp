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
  std::map<std::string, ServiceHandlerVariant> service_handlers_;

  struct ServiceHandlers {
    ServiceHandler<dvl_msgs::srv::GetConfig> get_config;
    ServiceHandler<dvl_msgs::srv::ResetDeadReckoning> reset_dead_reckoning;
    ServiceHandler<dvl_msgs::srv::SetAcousticEnabled> set_acoustic_enabled;
    ServiceHandler<dvl_msgs::srv::SetSpeedOfSound> set_speed_of_sound;
  };
  void HandleCommandResponse(const nlohmann::json &data);
  void HandleSetConfigResponse(const nlohmann::json &data);

  std::shared_ptr<Dvl> dvl_;
  rclcpp::Publisher<dvl_msgs::msg::VelocityTransducerReport>::SharedPtr
      velocity_transducer_pub_;
  rclcpp::Publisher<dvl_msgs::msg::DeadReckoningReport>::SharedPtr
      dead_reckoning_pub_;
  rclcpp::TimerBase::SharedPtr run_timer_;

  Params params_;
};
}  // namespace dvl
