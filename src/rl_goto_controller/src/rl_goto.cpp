#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_managers/controller.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_msgs/msg/reference_stamped.hpp>
#include <mrs_msgs/msg/float64_array_stamped.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <map>
#include <mutex>
#include <optional>
#include <vector>

namespace rl_goto_controller
{

constexpr int OBS_SIZE    = 17;
constexpr int ACT_SIZE    = 4;
constexpr int PARAMS_SIZE = 17;  // drone parameters of the adapt_goto encoder policies
constexpr int PARAMS_REF  = 12;  // reference values of their log ratios, stored in the policy file

class RLGoto : public mrs_uav_managers::Controller {

public:
  bool initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) override;

  bool activate([[maybe_unused]] const ControlOutput &last_control_output) override {
    last_action_.fill(0.0f);
    next_policy_time_.reset();
    goal_.reset();
    history_.clear();
    {
      // drop goals received while inactive, the UAV must hold its position until a new goal arrives
      std::scoped_lock lock(mutex_goal_msg_);
      goal_msg_.reset();
    }
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
  bool            loadDroneParams(mrs_lib::ParamLoader &param_loader);
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

  // adapt_goto encoder policies: the drone parameters are encoded into a latent appended to the observation
  bool               adapt_encoder_ = false;
  std::vector<Layer> encoder_layers_;
  bool               latent_tanh_ = false;
  Eigen::VectorXf    params_ref_;
  Eigen::VectorXf    params_;

  rclcpp::Duration            policy_period_{0, 0};
  std::optional<rclcpp::Time> next_policy_time_;

  rclcpp::Subscription<mrs_msgs::msg::ReferenceStamped>::SharedPtr sub_goal_;
  std::mutex                                                       mutex_goal_msg_;
  std::optional<mrs_msgs::msg::ReferenceStamped>                   goal_msg_;
  std::optional<Eigen::Vector3d>                                   goal_;

  // what the policy saw and did, to compare simulation and real flights from the bags
  rclcpp::Publisher<mrs_msgs::msg::Float64ArrayStamped>::SharedPtr pub_observation_;
  rclcpp::Publisher<mrs_msgs::msg::Float64ArrayStamped>::SharedPtr pub_action_;
  rclcpp::Publisher<mrs_msgs::msg::Float64ArrayStamped>::SharedPtr pub_goal_;
  rclcpp::Publisher<mrs_msgs::msg::Float64ArrayStamped>::SharedPtr pub_drone_params_;
};

/**
 * @brief an Eigen vector as a stamped array, stamped like the UAV state it was computed from
 */
template <typename Vector>
mrs_msgs::msg::Float64ArrayStamped toArrayMsg(const std_msgs::msg::Header &header, const Vector &v) {
  mrs_msgs::msg::Float64ArrayStamped msg;
  msg.header = header;
  msg.values.assign(v.data(), v.data() + v.size());
  return msg;
}

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
  private_handlers->param_loader->loadParam("adapt_encoder", adapt_encoder_);

  if (!private_handlers->param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load all parameters!");
    return false;
  }

  policy_period_ = rclcpp::Duration::from_seconds(1.0 / policy_rate);
  hover_cmd_     = static_cast<float>(hover_cmd);
  action_scale_  = static_cast<float>(action_scale);

  // relative paths point into the installed policies/ of this package
  if (!policy_path.empty() && policy_path.front() != '/') {
    policy_path = ament_index_cpp::get_package_share_directory("rl_goto_controller") + "/policies/" + policy_path;
  }

  if (!loadPolicy(policy_path)) {
    return false;
  }

  if (adapt_encoder_ && !loadDroneParams(*private_handlers->param_loader)) {
    return false;
  }

  sub_goal_ = node_->create_subscription<mrs_msgs::msg::ReferenceStamped>("~/rl_goto/goal", 10, [this](const mrs_msgs::msg::ReferenceStamped::SharedPtr msg) {
    std::scoped_lock lock(mutex_goal_msg_);
    goal_msg_ = *msg;
  });

  // published at every policy step, stamped with the UAV state the step used:
  //   observation: the policy input without the latent, obs_history frames (newest first) of OBS_SIZE values
  //   action:      the raw policy output, before clipping and scaling to the motor commands (hw_api/actuator_cmd)
  //   goal:        the goal position in the frame of the UAV state
  pub_observation_ = node_->create_publisher<mrs_msgs::msg::Float64ArrayStamped>("~/rl_goto/observation", 100);
  pub_action_      = node_->create_publisher<mrs_msgs::msg::Float64ArrayStamped>("~/rl_goto/action", 100);
  pub_goal_        = node_->create_publisher<mrs_msgs::msg::Float64ArrayStamped>("~/rl_goto/goal_used", 100);

  // the normalized drone parameters fed to the encoder, latched so a bag started later still gets them
  pub_drone_params_ = node_->create_publisher<mrs_msgs::msg::Float64ArrayStamped>("~/rl_goto/drone_params", rclcpp::QoS(1).transient_local());
  if (adapt_encoder_) {
    std_msgs::msg::Header header;
    header.stamp = node_->get_clock()->now();
    pub_drone_params_->publish(toArrayMsg(header, params_));
  }

  return true;
}

/**
 * @brief copies the weights of an MLP (Linear -> tanh -> ... -> Linear) into layers_
 *
 * @param path the policy exported by utils/export_policy.py (.rlp): "RLP1", uint32 tensor count, then per tensor
 *             uint32 name length, name, uint32 dims, uint32 shape[dims], float32 data (row-major), all little-endian
 *
 * @return true on success
 */
bool RLGoto::loadPolicy(const std::string &path) {

  std::ifstream file(path, std::ios::binary);

  const auto read_u32 = [&file]() {
    uint32_t v = 0;
    file.read(reinterpret_cast<char *>(&v), sizeof(v));
    return v;
  };

  char magic[4] = {};
  file.read(magic, sizeof(magic));
  if (!file || std::string(magic, sizeof(magic)) != "RLP1") {
    RCLCPP_ERROR(node_->get_logger(), "failed to load policy '%s': missing or not an .rlp file (utils/export_policy.py)", path.c_str());
    return false;
  }

  std::map<std::string, Eigen::VectorXf> extras;

  const uint32_t n_tensors = read_u32();
  for (uint32_t i = 0; i < n_tensors && file; i++) {

    std::string name(read_u32(), '\0');
    file.read(name.data(), name.size());

    std::vector<uint32_t> shape(read_u32());
    for (auto &s : shape) {
      s = read_u32();
    }

    if (!file || shape.empty() || shape.size() > 2) {
      break;
    }

    const Eigen::Index rows = shape[0];
    const Eigen::Index cols = shape.size() == 2 ? shape[1] : 1;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> t(rows, cols);
    file.read(reinterpret_cast<char *>(t.data()), t.size() * sizeof(float));

    std::vector<Layer> &layers = name.rfind("encoder.", 0) == 0 ? encoder_layers_ : layers_;
    if (name.find("weight") != std::string::npos && shape.size() == 2) {
      layers.push_back({t, {}});
    } else if (name.find("bias") != std::string::npos && !layers.empty()) {
      layers.back().b = t.reshaped();
    } else {
      extras[name] = t.reshaped();
    }
  }

  if (!file) {
    RCLCPP_ERROR(node_->get_logger(), "failed to load policy '%s': truncated or corrupted file", path.c_str());
    return false;
  }

  if (!encoder_layers_.empty()) {
    if (!extras.count("latent_tanh") || !extras.count("params_ref")) {
      RCLCPP_ERROR(node_->get_logger(), "policy '%s' has an encoder but no latent_tanh / params_ref", path.c_str());
      return false;
    }
    latent_tanh_ = extras["latent_tanh"](0) != 0.0f;
    params_ref_  = extras["params_ref"];
  }

  if (adapt_encoder_ != !encoder_layers_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "policy '%s' %s an encoder but adapt_encoder is %s", path.c_str(), encoder_layers_.empty() ? "has no" : "has",
                 adapt_encoder_ ? "true" : "false");
    return false;
  }

  const int latent_dim = adapt_encoder_ ? encoder_layers_.back().W.rows() : 0;
  if (layers_.empty() || layers_.front().W.cols() != OBS_SIZE * obs_history_ + latent_dim) {
    RCLCPP_ERROR(node_->get_logger(), "policy '%s' does not take %d x %d observations, check obs_history", path.c_str(), obs_history_, OBS_SIZE);
    return false;
  }

  if (adapt_encoder_ && (encoder_layers_.front().W.cols() != PARAMS_SIZE || params_ref_.size() != PARAMS_REF)) {
    RCLCPP_ERROR(node_->get_logger(), "policy '%s' does not take the %d drone parameters, or was exported without them (test.py export=true)",
                 path.c_str(), PARAMS_SIZE);
    return false;
  }

  return true;
}

/**
 * @brief loads the parameters of the flown drone and normalizes them as the genesis env (env_adapt_goto.py _update_params)
 *
 * @return true on success
 */
bool RLGoto::loadDroneParams(mrs_lib::ParamLoader &param_loader) {

  double              mass, arm, kf, max_rpm, action_delay;
  std::vector<double> inertia, motor_tau, motor_eff;
  param_loader.loadParam("drone/mass", mass);
  param_loader.loadParam("drone/inertia", inertia);
  param_loader.loadParam("drone/arm", arm);
  param_loader.loadParam("drone/kf", kf);
  param_loader.loadParam("drone/max_rpm", max_rpm);
  param_loader.loadParam("drone/motor_tau", motor_tau);
  param_loader.loadParam("drone/motor_eff", motor_eff);
  param_loader.loadParam("drone/action_delay", action_delay);

  if (!param_loader.loadedSuccessfully() || inertia.size() != 3 || motor_tau.size() != 2 || motor_eff.size() != ACT_SIZE) {
    RCLCPP_ERROR(node_->get_logger(), "could not load the drone parameters (inertia: 3 values, motor_tau: 2, motor_eff: 4)");
    return false;
  }

  // physical parameters as log ratios to params_ref: mass, inertia (3), arm (4), kf, max_rpm, motor_tau (2)
  const std::array<double, PARAMS_REF> physical = {mass, inertia[0], inertia[1], inertia[2], arm, arm, arm, arm, kf, max_rpm, motor_tau[0], motor_tau[1]};

  params_.resize(PARAMS_SIZE);
  for (int i = 0; i < PARAMS_REF; i++) {
    params_[i] = static_cast<float>(std::log(physical[i] / params_ref_[i]));
  }
  for (int i = 0; i < ACT_SIZE; i++) {
    params_[PARAMS_REF + i] = static_cast<float>((motor_eff[i] - 1.0) * 10.0);
  }
  params_[PARAMS_SIZE - 1] = static_cast<float>(action_delay / 0.02);

  return true;
}

/**
 * @brief evaluates the MLP, with adapt_encoder first the encoder on the drone parameters, whose latent is appended to x
 *
 * @param x the observation
 *
 * @return the unclipped action, one per motor
 */
Eigen::VectorXf RLGoto::runPolicy(Eigen::VectorXf x) const {

  if (adapt_encoder_) {
    Eigen::VectorXf z = params_;
    for (size_t i = 0; i < encoder_layers_.size(); i++) {
      z = encoder_layers_[i].W * z + encoder_layers_[i].b;
      if (i + 1 < encoder_layers_.size() || latent_tanh_) {
        z = z.array().tanh();
      }
    }
    x.conservativeResize(x.size() + z.size());
    x.tail(z.size()) = z;
  }

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

    const Eigen::VectorXf observation = buildObservation(uav_state);
    const Eigen::VectorXf action      = runPolicy(observation);

    pub_observation_->publish(toArrayMsg(uav_state.header, observation));
    pub_action_->publish(toArrayMsg(uav_state.header, action));
    pub_goal_->publish(toArrayMsg(uav_state.header, *goal_));

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
