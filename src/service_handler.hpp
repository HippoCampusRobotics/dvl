#pragma once
#include <dvl/dvl.hpp>
#include <dvl/json_protocol.hpp>
#include <dvl_msgs/srv/get_config.hpp>
#include <dvl_msgs/srv/reset_dead_reckoning.hpp>
#include <dvl_msgs/srv/set_acoustic_enabled.hpp>
#include <dvl_msgs/srv/set_speed_of_sound.hpp>
#include <ios>
#include <rclcpp/node.hpp>

namespace dvl {

// needed for static_assert(false) equivalent
template <class T>
constexpr std::false_type always_false{};

template <typename ServiceT>
class ServiceHandler {
 public:
  typedef typename ServiceT::Response ResponseT;

  ServiceHandler() {}

  ServiceHandler(rclcpp::Node *parent_node, const std::string &service_name)
      : node_(parent_node), service_name_(service_name) {
    Initialize();
  }

  void Initialize(rclcpp::Node *parent_node, const std::string &service_name) {
    node_ = parent_node;
    service_name_ = service_name;
    Initialize();
  }

  void Serve(const std::shared_ptr<rmw_request_id_t> _header,
             const typename ServiceT::Request::SharedPtr _request) {
    std::cout << "Node address on serve: " << static_cast<void *>(node_)
              << std::endl;
    bool success = true;
    std::string error_message{};
    // should never happen
    if (!ServiceOkay()) {
      RCLCPP_FATAL(node_->get_logger(),
                   "Service callback called, but service not available?! "
                   "This should NEVER happen.");
      req_id_.reset();
      return;
    }
    // reject if we already serve a request
    if (req_id_) {
      typename ServiceT::Response resp;
      resp.success = false;
      resp.message = "Busy handling another request. Try again later.";
      RCLCPP_INFO(node_->get_logger(),
                  "Ignoring service request while handling another request");
      service_->send_response(*_header, resp);
      return;
    }

    // header is required to send the response to the correct client
    req_id_ = _header;
    std::shared_ptr<Dvl> dvl = dvl_weak_.lock();
    if (!dvl) {
      error_message = "DVL not available.";
      success = false;
    } else if (!SendRequest(_request, dvl)) {
      // TODO: optionally write response dependent on service type if
      // required.
      error_message = "Could not send request to DVL";
      success = false;
    }

    if (!success) {
      SendFailResponse(error_message);
      RCLCPP_ERROR_STREAM(node_->get_logger(), error_message);
      req_id_.reset();
      return;
    }
    // we have handled everything that would cause the service to fail
    // immediately. now take care of the defered response
    timer_->reset();
  }

  void Initialize() {
    assert(node_ != nullptr &&
           "Service handler requires the parent node pointer to be set.");
    std::cout << "Node address on init: " << static_cast<void *>(node_)
              << std::endl;
    std::chrono::milliseconds timeout_ms = std::chrono::milliseconds(3000);
    timer_ = node_->create_timer(timeout_ms, [this]() { OnTimeout(); });
    timer_->cancel();

    RCLCPP_INFO(node_->get_logger(), "Providing service: %s",
                service_name_.c_str());
    service_ = node_->create_service<ServiceT>(
        service_name_,
        [this](const std::shared_ptr<rmw_request_id_t> header,
               const typename ServiceT::Request::SharedPtr _request) {
          Serve(header, _request);
        });
  }
  void SetDVL(std::shared_ptr<Dvl> _dvl) { dvl_weak_ = _dvl; }

  void OnDVLResponse(typename ServiceT::Response::SharedPtr _response) {
    if (timer_) {
      timer_->cancel();
    }
    if (!ServiceOkay()) {
      return;
    }
    if (!RequestIDOkay()) {
      return;
    }
    if (!_response) {
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "Trying to respond to request.");
    service_->send_response(*req_id_, *_response);
    req_id_.reset();
  }

  void OnTimeout() {
    const std::string text = "Response from DVL timed out.";
    SendFailResponse(text);
    RCLCPP_ERROR_STREAM(node_->get_logger(), text);
    req_id_.reset();
    if (timer_) {
      timer_->cancel();
    }
  }

  bool IsWaitingForResponse() {
    if (!timer_) {
      return false;
    }
    if (timer_->is_canceled()) {
      return false;
    }
    return req_id_ != nullptr;
  }

 private:
  bool SendRequest(const typename ServiceT::Request::SharedPtr _req,
                   std::shared_ptr<Dvl> _dvl) {
    using namespace dvl_msgs::srv;
    if constexpr (std::is_same_v<ServiceT, GetConfig>) {
      return _dvl->SendGetConfig();
    } else if constexpr (std::is_same_v<ServiceT, ResetDeadReckoning>) {
      return _dvl->SendResetDeadReckoning();
    } else if constexpr (std::is_same_v<ServiceT, SetAcousticEnabled>) {
      return _dvl->SetAcousticEnabled(_req->acoustic_enabled);
    } else if constexpr (std::is_same_v<ServiceT, SetSpeedOfSound>) {
      return _dvl->SetSpeedOfSound(_req->speed_of_sound);
    } else {
      // means this template is not prepared for this
      static_assert(always_false<ServiceT>,
                    "Not implemented for the provided service type.");
    }
    return false;
  }
  void SendFailResponse(std::string message) {
    if (!ServiceOkay()) {
      return;
    }
    RCLCPP_ERROR(node_->get_logger(), "Service failed: %s", message.c_str());
    typename ServiceT::Response resp;
    resp.success = false;
    resp.message = message;
    service_->send_response(*req_id_, resp);
  }
  bool ServiceOkay() {
    if (!service_) {
      RCLCPP_FATAL(node_->get_logger(),
                   "Cannot response to service because we lost the pointer?!");
      return false;
    }
    return true;
  }
  bool RequestIDOkay() {
    if (!req_id_) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Cannot send response, because no valid request header.");
      return false;
    }
    return true;
  }
  std::shared_ptr<rmw_request_id_t> req_id_;
  std::weak_ptr<Dvl> dvl_weak_;
  typename rclcpp::Service<ServiceT>::SharedPtr service_;
  rclcpp::Node *node_ = nullptr;
  std::string service_name_;
  rclcpp::TimerBase::SharedPtr timer_;
};

typedef std::variant<ServiceHandler<dvl_msgs::srv::GetConfig>,
                     ServiceHandler<dvl_msgs::srv::ResetDeadReckoning>,
                     ServiceHandler<dvl_msgs::srv::SetAcousticEnabled>,
                     ServiceHandler<dvl_msgs::srv::SetSpeedOfSound>>
    ServiceHandlerVariant;
}  // namespace dvl
