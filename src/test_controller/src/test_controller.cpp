#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_managers/controller.h>
#include <mrs_lib/quadratic_throttle_model.h>

namespace test_controller
{

// open-loop hover: every motor gets the throttle that balances the UAV's weight,
// derived from the mass and throttle model of whatever drone is loaded
class TestController : public mrs_uav_managers::Controller {

public:
  bool initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  [[maybe_unused]] std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) override {
    node_            = node;
    common_handlers_ = common_handlers;
    return true;
  }

  void destroy() override {
  }

  bool activate(const ControlOutput &last_control_output) override {
    last_control_output_ = last_control_output;
    is_active_           = true;
    return true;
  }

  void deactivate() override {
    is_active_ = false;
  }

  void updateInactive([[maybe_unused]] const mrs_msgs::msg::UavState &uav_state,
                      [[maybe_unused]] const std::optional<mrs_msgs::msg::TrackerCommand> &tracker_command) override {
  }

  ControlOutput updateActive([[maybe_unused]] const mrs_msgs::msg::UavState       &uav_state,
                             [[maybe_unused]] const mrs_msgs::msg::TrackerCommand &tracker_command) override {

    last_control_output_.control_output = {};

    if (!is_active_) {
      return last_control_output_;
    }

    const double hover_force = common_handlers_->getMass() * common_handlers_->g;
    const double throttle    = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, hover_force, *node_);

    mrs_msgs::msg::HwApiActuatorCmd actuator_cmd;
    actuator_cmd.stamp = node_->get_clock()->now();
    actuator_cmd.motors.assign(common_handlers_->throttle_model.n_motors, throttle);

    last_control_output_.control_output         = actuator_cmd;
    last_control_output_.diagnostics.controller = "TestController";

    return last_control_output_;
  }

  const mrs_msgs::msg::ControllerStatus getStatus() override {
    mrs_msgs::msg::ControllerStatus status;
    status.active = is_active_;
    return status;
  }

  void switchOdometrySource([[maybe_unused]] const mrs_msgs::msg::UavState &new_uav_state) override {
  }

  void resetDisturbanceEstimators() override {
  }

  const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>
  setConstraints([[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request> &constraints) override {
    auto res     = std::make_shared<mrs_msgs::srv::DynamicsConstraintsSrv::Response>();
    res->success = true;
    return res;
  }

private:
  rclcpp::Node::SharedPtr                                              node_;
  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers_;

  bool          is_active_ = false;
  ControlOutput last_control_output_;
};

}  // namespace test_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(test_controller::TestController, mrs_uav_managers::Controller)
