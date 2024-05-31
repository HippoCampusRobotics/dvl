#pragma once
#include <dvl/dvl.hpp>
#include <dvl/json_protocol.hpp>
#include <dvl_msgs/srv/get_config.hpp>
#include <rclcpp/node.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace dvl {

// needed for static_assert(false) equivalent
template <class T>
constexpr std::false_type always_false{};

template <typename ServiceT>
class ServiceHandler {
 public:
  typedef typename ServiceT::Response ResponseT;
  ServiceHandler(rclcpp::Node &node, Cmd cmd) : cmd_(cmd), node_(node) {
    std::chrono::milliseconds timeout_ms = std::chrono::milliseconds(3000);
    if (cmd == Cmd::CalibrateGyro()) {
      timeout_ms = std::chrono::milliseconds(20000);
    }
    timer_ = node_.create_timer(timeout_ms, [this]() { OnTimeout(); });
    timer_->cancel();
    std::string service_name = "~/" + cmd_.Name();
    RCLCPP_INFO(node_.get_logger(), "Providing service: %s",
                service_name.c_str());
    service_ = node_.create_service<ServiceT>(
        service_name,
        [this](const std::shared_ptr<rmw_request_id_t> header,
               const typename ServiceT::Request::SharedPtr _request) {
          bool success = true;
          std::string error_message{};
          // should never happen
          if (!ServiceOkay()) {
            RCLCPP_FATAL(node_.get_logger(),
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
            service_->send_response(*header, resp);
            return;
          }

          // header is required to send the response to the correct client
          req_id_ = header;
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
            RCLCPP_ERROR_STREAM(node_.get_logger(), error_message);
            req_id_.reset();
            return;
          }
          // we have handled everything that would cause the service to fail
          // immediately. now take care of the defered response
          timer_->reset();
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
    service_->send_response(*req_id_, *_response);
    req_id_.reset();
  }

  void OnTimeout() {
    const std::string text = "Response from DVL timed out.";
    SendFailResponse(text);
    RCLCPP_ERROR_STREAM(node_.get_logger(), text);
    req_id_.reset();
    if (timer_) {
      timer_->cancel();
    }
  }

 private:
  bool SendRequest(const typename ServiceT::Request::SharedPtr,
                   std::shared_ptr<Dvl> _dvl) {
    if constexpr (std::is_same_v<ServiceT, dvl_msgs::srv::GetConfig>) {
      assert(cmd_ == Cmd::GetConfig());
      return _dvl->SendGetConfig();
    } else if constexpr (std::is_same_v<ServiceT, std_srvs::srv::Trigger>) {
      if (cmd_ == Cmd::ResetDeadReckoning()) {
        return _dvl->SendResetDeadReckoning();
      } else {
        assert(false && "Unhandled command for trigger service.");
      }
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
    typename ServiceT::Response resp;
    resp.success = false;
    resp.message = message;
    service_->send_response(*req_id_, resp);
  }
  bool ServiceOkay() {
    if (!service_) {
      RCLCPP_FATAL(node_.get_logger(),
                   "Cannot response to service because we lost the pointer?!");
      return false;
    }
    return true;
  }
  bool RequestIDOkay() {
    if (!req_id_) {
      RCLCPP_ERROR(node_.get_logger(),
                   "Cannot send response, because no valid request header.");
      return false;
    }
    return true;
  }
  Cmd cmd_;
  std::shared_ptr<rmw_request_id_t> req_id_;
  std::weak_ptr<Dvl> dvl_weak_;
  typename rclcpp::Service<ServiceT>::SharedPtr service_;
  rclcpp::Node &node_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dvl
