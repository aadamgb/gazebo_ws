#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_managers/controller.h>
#include <mrs_lib/param_loader.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <torch/script.h>
#include <ATen/Parallel.h>

#include <algorithm>
#include <array>

namespace rl_goto_controller
{

// policy I/O -- must match the trained network, verified in initialize()
constexpr int OBS_SIZE = 1;  // TODO: set to the observation size
constexpr int ACT_SIZE = 4;  // one command per motor, in [0, 1]

class RLGoto : public mrs_uav_managers::Controller {

public:
  bool initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) override;

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

  ControlOutput updateActive(const mrs_msgs::msg::UavState &uav_state, const mrs_msgs::msg::TrackerCommand &tracker_command) override;

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
  std::array<float, OBS_SIZE> buildObservation(const mrs_msgs::msg::UavState &uav_state, const mrs_msgs::msg::TrackerCommand &tracker_command);

  rclcpp::Node::SharedPtr node_;
  bool                    is_active_ = false;

  torch::jit::Module policy_;

  ControlOutput last_control_output_;
};

/* initialize() //{ */

bool RLGoto::initialize(const rclcpp::Node::SharedPtr &node, [[maybe_unused]] std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                        std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  node_ = node;

  // resolved under mrs_uav_controllers/<namespace from custom_config>/
  private_handlers->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory("rl_goto_controller") + "/config/rl_goto.yaml");

  std::string policy_path;
  private_handlers->param_loader->loadParam("policy", policy_path);

  if (!private_handlers->param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load all parameters!");
    return false;
  }

  // a small MLP gains nothing from intra-op threads
  at::set_num_threads(1);

  try {
    policy_ = torch::jit::load(policy_path, torch::kCPU);
    policy_.eval();

    // warm up and check the network's I/O
    torch::NoGradGuard  no_grad;
    const torch::Tensor out = policy_.forward({torch::zeros({1, OBS_SIZE})}).toTensor();

    if (out.dim() != 2 || out.size(0) != 1 || out.size(1) != ACT_SIZE || out.scalar_type() != torch::kFloat) {
      RCLCPP_ERROR(node_->get_logger(), "policy '%s' has the wrong output, expected float [1, %d]", policy_path.c_str(), ACT_SIZE);
      return false;
    }
  }
  catch (const c10::Error &e) {
    RCLCPP_ERROR(node_->get_logger(), "failed to load policy '%s': %s", policy_path.c_str(), e.what_without_backtrace());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "loaded policy '%s'", policy_path.c_str());

  return true;
}

//}

/* buildObservation() //{ */

std::array<float, OBS_SIZE> RLGoto::buildObservation([[maybe_unused]] const mrs_msgs::msg::UavState       &uav_state,
                                                     [[maybe_unused]] const mrs_msgs::msg::TrackerCommand &tracker_command) {

  std::array<float, OBS_SIZE> obs{};

  // TODO: fill in the observation, e.g.
  //   uav_state.pose.position / .orientation, uav_state.velocity.linear / .angular
  //   goto reference: tracker_command.position (+ tracker_command.heading)

  return obs;
}

//}

/* updateActive() //{ */

RLGoto::ControlOutput RLGoto::updateActive(const mrs_msgs::msg::UavState &uav_state, const mrs_msgs::msg::TrackerCommand &tracker_command) {

  last_control_output_.control_output = {};

  if (!is_active_) {
    return last_control_output_;
  }

  auto obs = buildObservation(uav_state, tracker_command);

  torch::NoGradGuard  no_grad;
  const torch::Tensor out = policy_.forward({torch::from_blob(obs.data(), {1, OBS_SIZE})}).toTensor().contiguous();
  const float        *act = out.data_ptr<float>();

  mrs_msgs::msg::HwApiActuatorCmd actuator_cmd;
  actuator_cmd.stamp = node_->get_clock()->now();

  for (int i = 0; i < ACT_SIZE; i++) {
    actuator_cmd.motors.push_back(std::clamp(static_cast<double>(act[i]), 0.0, 1.0));
  }

  last_control_output_.control_output         = actuator_cmd;
  last_control_output_.diagnostics.controller = "RLGoto";

  return last_control_output_;
}

//}

}  // namespace rl_goto_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rl_goto_controller::RLGoto, mrs_uav_managers::Controller)
