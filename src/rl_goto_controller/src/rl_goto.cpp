#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_managers/controller.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_msgs/msg/reference_stamped.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <Eigen/Dense>

#include <torch/script.h>

#include <algorithm>
#include <array>
#include <deque>
#include <mutex>
#include <optional>
#include <vector>

namespace rl_goto_controller
{

constexpr int OBS_SIZE = 17;
constexpr int ACT_SIZE = 4;

class RLGoto : public mrs_uav_managers::Controller {

public:
  bool initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) override;

  bool activate([[maybe_unused]] const ControlOutput &last_control_output) override {
    last_action_.fill(0.0f);
    next_policy_time_.reset();
    goal_.reset();
    history_.clear();
    is_active_ = true;
    return true;
  }

  void deactivate() override {
    is_active_ = false;
  }

  ControlOutput updateActive(const mrs_msgs::msg::UavState &uav_state, const mrs_msgs::msg::TrackerCommand &tracker_command) override;

  void updateInactive([[maybe_unused]] const mrs_msgs::msg::UavState &uav_state,
                      [[maybe_unused]] const std::optional<mrs_msgs::msg::TrackerCommand> &tracker_command) override {
  }

  const mrs_msgs::msg::ControllerStatus getStatus() override {
    mrs_msgs::msg::ControllerStatus status;
    status.active = is_active_;
    return status;
  }

  void destroy() override {
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
  struct Layer {
    Eigen::MatrixXf W;
    Eigen::VectorXf b;
  };

  bool            loadPolicy(const std::string &path);
  Eigen::VectorXf buildObservation(const mrs_msgs::msg::UavState &uav_state);
  Eigen::VectorXf runPolicy(Eigen::VectorXf x) const;

  rclcpp::Node::SharedPtr                                              node_;
  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers_;
  bool                                                                 is_active_ = false;

  std::vector<Layer>          layers_;
  std::array<float, ACT_SIZE> last_action_{};
  int                         obs_history_ = 1;
  std::deque<Eigen::VectorXf> history_;
  bool                        hover_center_throttle_ = true;
  float                       hover_cmd_             = 0.0f;
  float                       action_scale_          = 1.0f;

  rclcpp::Duration            policy_period_{0, 0};
  std::optional<rclcpp::Time> next_policy_time_;

  rclcpp::Subscription<mrs_msgs::msg::ReferenceStamped>::SharedPtr sub_goal_;
  std::mutex                                                       mutex_goal_msg_;
  std::optional<mrs_msgs::msg::ReferenceStamped>                   goal_msg_;
  std::optional<Eigen::Vector3d>                                   goal_;
};

// --------------------------------------------------------------------------------------------------- //

/**
 * @brief loads the parameters and the policy and subscribes to the goal topic
 *
 * @return true on success, false makes the control manager refuse the controller
 */
bool RLGoto::initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                        std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  node_            = node;
  common_handlers_ = common_handlers;

  private_handlers->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory("rl_goto_controller") + "/config/rl_goto.yaml");

  std::string policy_path;
  double      policy_rate, hover_cmd, action_scale;
  private_handlers->param_loader->loadParam("policy", policy_path);
  private_handlers->param_loader->loadParam("policy_rate", policy_rate);
  private_handlers->param_loader->loadParam("obs_history", obs_history_);
  private_handlers->param_loader->loadParam("hover_center_throttle", hover_center_throttle_);
  private_handlers->param_loader->loadParam("hover_cmd", hover_cmd);
  private_handlers->param_loader->loadParam("action_scale", action_scale);

  if (!private_handlers->param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load all parameters!");
    return false;
  }

  policy_period_ = rclcpp::Duration::from_seconds(1.0 / policy_rate);
  hover_cmd_     = static_cast<float>(hover_cmd);
  action_scale_  = static_cast<float>(action_scale);

  if (!loadPolicy(policy_path)) {
    return false;
  }

  sub_goal_ = node_->create_subscription<mrs_msgs::msg::ReferenceStamped>("~/rl_goto/goal", 10, [this](const mrs_msgs::msg::ReferenceStamped::SharedPtr msg) {
    std::scoped_lock lock(mutex_goal_msg_);
    goal_msg_ = *msg;
  });

  return true;
}

/**
 * @brief copies the weights of a TorchScript MLP (Linear -> tanh -> ... -> Linear) into layers_
 *
 * @param path the scripted policy (.pt)
 *
 * @return true on success
 */
bool RLGoto::loadPolicy(const std::string &path) {

  try {
    torch::jit::Module policy = torch::jit::load(path, torch::kCPU);
    for (const auto &p : policy.named_parameters()) {
      const torch::Tensor t = p.value.detach().contiguous();
      if (p.name.find("weight") != std::string::npos) {
        layers_.push_back({Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(t.data_ptr<float>(), t.size(0), t.size(1)), {}});
      } else if (p.name.find("bias") != std::string::npos) {
        layers_.back().b = Eigen::Map<const Eigen::VectorXf>(t.data_ptr<float>(), t.size(0));
      }
    }
  }
  catch (const c10::Error &e) {
    RCLCPP_ERROR(node_->get_logger(), "failed to load policy '%s': %s", path.c_str(), e.what_without_backtrace());
    return false;
  }

  if (layers_.empty() || layers_.front().W.cols() != OBS_SIZE * obs_history_) {
    RCLCPP_ERROR(node_->get_logger(), "policy '%s' does not take %d x %d observations, check obs_history", path.c_str(), obs_history_, OBS_SIZE);
    return false;
  }

  return true;
}

/**
 * @brief evaluates the MLP
 *
 * @param x the observation
 *
 * @return the unclipped action, one per motor
 */
Eigen::VectorXf RLGoto::runPolicy(Eigen::VectorXf x) const {

  for (size_t i = 0; i < layers_.size(); i++) {
    x = layers_[i].W * x + layers_[i].b;
    if (i + 1 < layers_.size()) {
      x = x.array().tanh();
    }
  }

  return x;
}

/**
 * @brief builds the policy observation as in the genesis env
 *
 * @param uav_state the estimated UAV state
 *
 * @return obs_history frames (newest first) of [goal - position, orientation (w, x, y, z), body velocity, body rates,
 *         last action], scaled and clipped
 */
Eigen::VectorXf RLGoto::buildObservation(const mrs_msgs::msg::UavState &uav_state) {

  const Eigen::Vector3d    p(uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z);
  const Eigen::Vector3d    v(uav_state.velocity.linear.x, uav_state.velocity.linear.y, uav_state.velocity.linear.z);
  const Eigen::Vector3d    w(uav_state.velocity.angular.x, uav_state.velocity.angular.y, uav_state.velocity.angular.z);
  const Eigen::Quaterniond q(uav_state.pose.orientation.w, uav_state.pose.orientation.x, uav_state.pose.orientation.y, uav_state.pose.orientation.z);

  if (!goal_) {
    goal_ = p;
  }

  std::optional<mrs_msgs::msg::ReferenceStamped> goal_msg;
  {
    std::scoped_lock lock(mutex_goal_msg_);
    goal_msg = goal_msg_;
  }
  if (goal_msg) {
    if (const auto goal = common_handlers_->transformer->transformSingle(*goal_msg, uav_state.header.frame_id)) {
      goal_ = Eigen::Vector3d(goal->reference.position.x, goal->reference.position.y, goal->reference.position.z);
    }
  }

  const Eigen::Vector3f rel_pos = ((*goal_ - p) / 3.0).cast<float>().cwiseMax(-1.0f).cwiseMin(1.0f);
  const Eigen::Vector3f v_body  = (q.normalized().conjugate() * v / 3.0).cast<float>().cwiseMax(-1.0f).cwiseMin(1.0f);
  const Eigen::Vector3f w_body  = (w / M_PI).cast<float>().cwiseMax(-1.0f).cwiseMin(1.0f);

  Eigen::VectorXf frame(OBS_SIZE);
  frame << rel_pos, q.w(), q.x(), q.y(), q.z(), v_body, w_body, Eigen::Map<const Eigen::Vector4f>(last_action_.data());

  if (history_.empty()) {
    history_.assign(obs_history_, frame);
  } else {
    history_.push_front(frame);
    history_.pop_back();
  }

  Eigen::VectorXf x(OBS_SIZE * obs_history_);
  for (int i = 0; i < obs_history_; i++) {
    x.segment(i * OBS_SIZE, OBS_SIZE) = history_[i];
  }
  return x;
}


/**
 * @brief the main routine, runs the policy at policy_rate and holds its output in between
 *
 * @param uav_state the estimated UAV state
 * @param tracker_command unused, the goal comes from the goal topic
 *
 * @return the motor commands in [0, 1]
 */
RLGoto::ControlOutput RLGoto::updateActive(const mrs_msgs::msg::UavState &uav_state, [[maybe_unused]] const mrs_msgs::msg::TrackerCommand &tracker_command) {

  ControlOutput output;

  if (!is_active_) {
    return output;
  }

  const rclcpp::Time now(uav_state.header.stamp, RCL_ROS_TIME);

  if (!next_policy_time_ || now >= *next_policy_time_) {

    next_policy_time_ = (!next_policy_time_ || now - *next_policy_time_ >= policy_period_) ? now + policy_period_ : *next_policy_time_ + policy_period_;

    const Eigen::VectorXf action = runPolicy(buildObservation(uav_state));

    for (int i = 0; i < ACT_SIZE; i++) {
      last_action_[i] = std::clamp(action[i], hover_center_throttle_ ? -1.0f : 0.0f, 1.0f);
    }
  }

  mrs_msgs::msg::HwApiActuatorCmd actuator_cmd;
  actuator_cmd.stamp = node_->get_clock()->now();
  for (const float a : last_action_) {
    actuator_cmd.motors.push_back(hover_center_throttle_ ? std::clamp(hover_cmd_ * (1.0f + action_scale_ * a), 0.0f, 1.0f) : a);
  }

  output.control_output         = actuator_cmd;
  output.diagnostics.controller = "RLGoto";

  return output;
}

}  // namespace rl_goto_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rl_goto_controller::RLGoto, mrs_uav_managers::Controller)
