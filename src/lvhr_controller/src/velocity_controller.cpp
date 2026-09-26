/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_managers/controller.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>

#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>

#include <torch/script.h>
#include <ATen/Parallel.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <sstream>

//}

namespace lvhr_controller
{

// policy I/O -- must match the trained network, verified against it in initialize()
constexpr int OBS_SIZE = 20;
constexpr int ACT_SIZE = 4;

// only used to report whether a gate crossing went through the opening
constexpr double GATE_HALF_WIDTH  = 0.6;
constexpr double GATE_HALF_HEIGHT = 0.6;

/* RacingTrack //{ */

struct RacingTrack
{
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Matrix3d> rotmats;
};

// intrinsic Z-Y-X from degrees
Eigen::Matrix3d rpyToRotmat(const Eigen::Vector3d &rpy_deg) {

  const Eigen::Vector3d rpy = rpy_deg * (M_PI / 180.0);

  return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

bool loadGatesFromYaml(const std::string &path, RacingTrack &track, const rclcpp::Logger &logger) {

  try {

    const YAML::Node data = YAML::LoadFile(path);

    if (!data["gates"] || data["gates"].size() == 0) {
      RCLCPP_ERROR(logger, "[VelocityController]: no gates under the 'gates' key in '%s'", path.c_str());
      return false;
    }

    track.positions.clear();
    track.rotmats.clear();

    for (const auto &g : data["gates"]) {

      const auto pos = g["position"].as<std::vector<double>>();
      const auto rpy = g["rpy"].as<std::vector<double>>();

      if (pos.size() != 3 || rpy.size() != 3) {
        RCLCPP_ERROR(logger, "[VelocityController]: each gate needs a 3-element 'position' and 'rpy' in '%s'", path.c_str());
        return false;
      }

      track.positions.emplace_back(pos[0], pos[1], pos[2]);
      track.rotmats.emplace_back(rpyToRotmat(Eigen::Vector3d(rpy[0], rpy[1], rpy[2])));
    }

    return true;
  }
  catch (const YAML::Exception &e) {
    RCLCPP_ERROR(logger, "[VelocityController]: failed to parse racing track '%s': %s", path.c_str(), e.what());
    return false;
  }
}

//}

/* //{ class VelocityController */

class VelocityController : public mrs_uav_managers::Controller {

public:
  bool initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) override;

  void destroy() override;

  bool activate(const ControlOutput &last_control_output) override;

  void deactivate(void) override;

  void updateInactive(const mrs_msgs::msg::UavState &uav_state, const std::optional<mrs_msgs::msg::TrackerCommand> &tracker_command) override;

  ControlOutput updateActive(const mrs_msgs::msg::UavState &uav_state, const mrs_msgs::msg::TrackerCommand &tracker_command) override;

  const mrs_msgs::msg::ControllerStatus getStatus() override;

  void switchOdometrySource(const mrs_msgs::msg::UavState &new_uav_state) override;

  void resetDisturbanceEstimators(void) override;

  const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>
  setConstraints(const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request> &constraints) override;

private:
  void checkGatePassed(const Eigen::Vector3d &curr_pos);

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  double _uav_mass_;

  // | ------------------------- params ------------------------- |

  Eigen::Vector3d omega_output_min_;
  Eigen::Vector3d omega_output_max_;
  double          throttle_clamp_min_;
  double          throttle_clamp_max_;
  double          collective_thrust_max_;  // [N], maps collective force to throttle

  Eigen::Vector3d v_max_;        // [m/s], scales the policy's velocity action
  double          yawrate_max_;  // [rad/s], scales the policy's heading-rate action
  Eigen::Vector3d k_v_;          // velocity-error gain
  Eigen::Vector3d k_R_;          // SO(3) attitude-error gain

  // | ------------------------- policy ------------------------- |

  torch::jit::Module policy_;
  std::mutex         mutex_policy_;

  // | ---------------------- racing track ---------------------- |

  RacingTrack     track_;
  size_t          gate_idx_ = 0;
  Eigen::Vector3d prev_pos_ = Eigen::Vector3d::Zero();

  // | ----------------------- lap timing ----------------------- |

  bool         lap_started_    = false;
  size_t       lap_start_gate_ = 0;
  uint32_t     lap_count_      = 0;
  rclcpp::Time lap_start_time_;

  // | ----------------------- constraints ---------------------- |

  mrs_msgs::msg::DynamicsConstraints constraints_;
  std::mutex                         mutex_constraints_;

  // | ------------------------- output ------------------------- |

  ControlOutput     last_control_output_;
  std::atomic<bool> first_iteration_ = true;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

bool VelocityController::initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                                    std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  node_  = node;
  clock_ = node_->get_clock();

  common_handlers_  = common_handlers;
  private_handlers_ = private_handlers;

  _uav_mass_ = common_handlers->getMass();

  // | --------------------- load parameters -------------------- |

  // resolved under mrs_uav_controllers/<namespace from custom_config>/
  private_handlers->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory("lvhr_controller") + "/config/velocity_controller.yaml");

  std::string         track_path, policy_path;
  std::vector<double> omega_min, omega_max, v_max, k_v, k_R;

  private_handlers->param_loader->loadParam("track", track_path);
  private_handlers->param_loader->loadParam("policy", policy_path);
  private_handlers->param_loader->loadParam("omega_output_min", omega_min);
  private_handlers->param_loader->loadParam("omega_output_max", omega_max);
  private_handlers->param_loader->loadParam("throttle_clamp_min", throttle_clamp_min_);
  private_handlers->param_loader->loadParam("throttle_clamp_max", throttle_clamp_max_);
  private_handlers->param_loader->loadParam("collective_thrust_max", collective_thrust_max_);
  private_handlers->param_loader->loadParam("v_max", v_max);
  private_handlers->param_loader->loadParam("wz_max", yawrate_max_);
  private_handlers->param_loader->loadParam("k_v", k_v);
  private_handlers->param_loader->loadParam("k_R", k_R);

  if (!private_handlers->param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load all parameters!");
    return false;
  }

  const auto toVec3 = [this](const std::vector<double> &in, const std::string &name, Eigen::Vector3d &out) {
    if (in.size() != 3) {
      RCLCPP_ERROR(node_->get_logger(), "parameter '%s' must have 3 elements, got %zu", name.c_str(), in.size());
      return false;
    }
    out = Eigen::Vector3d(in[0], in[1], in[2]);
    return true;
  };

  if (!toVec3(omega_min, "omega_output_min", omega_output_min_) || !toVec3(omega_max, "omega_output_max", omega_output_max_) ||
      !toVec3(v_max, "v_max", v_max_) || !toVec3(k_v, "k_v", k_v_) || !toVec3(k_R, "k_R", k_R_)) {
    return false;
  }

  // | -------------------- load racing track ------------------- |

  if (!loadGatesFromYaml(track_path, track_, node_->get_logger())) {
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "loaded %zu gates from '%s'", track_.positions.size(), track_path.c_str());

  // | ----------------------- load policy ---------------------- |

  // a small MLP evaluated at the controller rate gains nothing from intra-op
  // threads, which only add latency and fight the simulator for cores
  at::set_num_threads(1);

  try {

    policy_ = torch::jit::load(policy_path, torch::kCPU);
    policy_.eval();

    // warm up, and check the network's I/O matches what updateActive() builds and decodes;
    // e.g. the adaptive-gain so3a policies output more than ACT_SIZE values
    torch::NoGradGuard no_grad;

    const torch::Tensor out = policy_.forward({torch::zeros({1, OBS_SIZE})}).toTensor();

    if (out.dim() != 2 || out.size(0) != 1 || out.size(1) != ACT_SIZE || out.scalar_type() != torch::kFloat) {
      std::ostringstream shape;
      shape << out.sizes();
      RCLCPP_ERROR(node_->get_logger(), "policy '%s' returns %s %s, expected float [1, %d]", policy_path.c_str(), c10::toString(out.scalar_type()),
                   shape.str().c_str(), ACT_SIZE);
      return false;
    }
  }
  catch (const c10::Error &e) {
    RCLCPP_ERROR(node_->get_logger(), "failed to load policy '%s': %s", policy_path.c_str(), e.what_without_backtrace());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "loaded policy '%s'", policy_path.c_str());

  // | ----------------------- finish init ---------------------- |

  RCLCPP_INFO(node_->get_logger(), "initialized");

  is_initialized_ = true;

  return true;
}

//}

/* //{ destroy() */

void VelocityController::destroy() {

  is_active_ = false;

  RCLCPP_INFO(node_->get_logger(), "destroyed");
}

//}

/* //{ activate() */

bool VelocityController::activate(const ControlOutput &last_control_output) {

  last_control_output_ = last_control_output;

  first_iteration_ = true;
  is_active_       = true;
  gate_idx_        = 0;
  lap_started_     = false;
  lap_count_       = 0;

  RCLCPP_INFO(node_->get_logger(), "activated");

  return true;
}

//}

/* //{ deactivate() */

void VelocityController::deactivate(void) {

  is_active_ = false;

  RCLCPP_INFO(node_->get_logger(), "deactivated");
}

//}

/* updateInactive() //{ */

void VelocityController::updateInactive([[maybe_unused]] const mrs_msgs::msg::UavState                   &uav_state,
                                        [[maybe_unused]] const std::optional<mrs_msgs::msg::TrackerCommand> &tracker_command) {
}

//}

/* //{ updateActive() */

VelocityController::ControlOutput VelocityController::updateActive(const mrs_msgs::msg::UavState                        &uav_state,
                                                                   [[maybe_unused]] const mrs_msgs::msg::TrackerCommand &tracker_command) {

  last_control_output_.control_output                = {};
  last_control_output_.desired_heading_rate          = {};
  last_control_output_.desired_orientation           = {};
  last_control_output_.desired_unbiased_acceleration = {};

  if (!is_active_) {
    return last_control_output_;
  }

  const Eigen::Vector3d p(uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z);
  const Eigen::Vector3d v(uav_state.velocity.linear.x, uav_state.velocity.linear.y, uav_state.velocity.linear.z);  // world frame

  // | -------------------- gate progression -------------------- |

  // needs a previous position to detect a crossing
  if (!first_iteration_) {
    checkGatePassed(p);
  }

  // | ------------------- build observation -------------------- |

  const Eigen::Vector3d &gate_pos = track_.positions[gate_idx_];
  const Eigen::Matrix3d &gate_R   = track_.rotmats[gate_idx_];
  const Eigen::Matrix3d  RT       = gate_R.transpose();

  const Eigen::Vector3d rel_pos   = RT * (p - gate_pos);
  const Eigen::Vector3d rel_vel   = RT * v;
  const Eigen::Vector3d gate_norm = gate_R.col(1);  // gate y axis = forward

  float obs[OBS_SIZE];

  // [0:3] relative position (gate frame)
  obs[0] = static_cast<float>(rel_pos.x());
  obs[1] = static_cast<float>(rel_pos.y());
  obs[2] = static_cast<float>(rel_pos.z());

  // [3:6] relative velocity (gate frame)
  obs[3] = static_cast<float>(rel_vel.x());
  obs[4] = static_cast<float>(rel_vel.y());
  obs[5] = static_cast<float>(rel_vel.z());

  // [6:10] orientation quaternion [w, x, y, z]
  obs[6] = static_cast<float>(uav_state.pose.orientation.w);
  obs[7] = static_cast<float>(uav_state.pose.orientation.x);
  obs[8] = static_cast<float>(uav_state.pose.orientation.y);
  obs[9] = static_cast<float>(uav_state.pose.orientation.z);

  // [10:13] body-frame angular rates [p, q, r]
  obs[10] = static_cast<float>(uav_state.velocity.angular.x);
  obs[11] = static_cast<float>(uav_state.velocity.angular.y);
  obs[12] = static_cast<float>(uav_state.velocity.angular.z);

  // [13:16] gate position (world frame)
  obs[13] = static_cast<float>(gate_pos.x());
  obs[14] = static_cast<float>(gate_pos.y());
  obs[15] = static_cast<float>(gate_pos.z());

  // [16:19] gate normal (world frame)
  obs[16] = static_cast<float>(gate_norm.x());
  obs[17] = static_cast<float>(gate_norm.y());
  obs[18] = static_cast<float>(gate_norm.z());

  // [19] gate index
  obs[19] = static_cast<float>(gate_idx_);

  // | ----------------------- run policy ----------------------- |

  float action[ACT_SIZE];

  {
    torch::NoGradGuard no_grad;

    const torch::Tensor input = torch::from_blob(obs, {1, OBS_SIZE}).clone();

    std::scoped_lock lock(mutex_policy_);

    const torch::Tensor out = policy_.forward({input}).toTensor().contiguous();

    std::copy_n(out.data_ptr<float>(), ACT_SIZE, action);
  }

  // | ----- action decoding: SO(3) velocity + heading rate ----- |

  // policy outputs are all in [0, 1]: [0:3] velocity command (world), [3] heading rate
  const Eigen::Vector3d v_cmd((std::clamp(action[0], 0.0f, 1.0f) * 2.0f - 1.0f) * v_max_.x(),
                              (std::clamp(action[1], 0.0f, 1.0f) * 2.0f - 1.0f) * v_max_.y(),
                              (std::clamp(action[2], 0.0f, 1.0f) * 2.0f - 1.0f) * v_max_.z());

  const double yaw_rate = (std::clamp(action[3], 0.0f, 1.0f) * 2.0 - 1.0) * yawrate_max_;

  const Eigen::Quaterniond q(uav_state.pose.orientation.w, uav_state.pose.orientation.x, uav_state.pose.orientation.y, uav_state.pose.orientation.z);
  const Eigen::Matrix3d    R  = q.normalized().toRotationMatrix();
  const Eigen::Vector3d    e3 = Eigen::Vector3d::UnitZ();

  // translational: velocity error -> desired specific thrust (world frame)
  const Eigen::Vector3d e_v     = v_cmd - v;
  const Eigen::Vector3d acc_des = k_v_.cwiseProduct(e_v) + common_handlers_->g * e3;

  // desired body z + collective force (Lee projection onto the current b3)
  const Eigen::Vector3d b3_des = acc_des.normalized();
  const Eigen::Vector3d b3     = R.col(2);
  const double          f_coll = std::max(0.0, _uav_mass_ * acc_des.dot(b3));

  // desired attitude: b1 from the CURRENT heading (decoupled yaw)
  const double          yaw = std::atan2(R(1, 0), R(0, 0));
  const Eigen::Vector3d b1_c(std::cos(yaw), std::sin(yaw), 0.0);
  const Eigen::Vector3d b2_des = b3_des.cross(b1_c).normalized();
  const Eigen::Vector3d b1_des = b2_des.cross(b3_des);

  Eigen::Matrix3d R_des;
  R_des.col(0) = b1_des;
  R_des.col(1) = b2_des;
  R_des.col(2) = b3_des;

  // SO(3) attitude error -> body-rate command
  const Eigen::Matrix3d e_R_mat = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
  const Eigen::Vector3d e_R(e_R_mat(2, 1), e_R_mat(0, 2), e_R_mat(1, 0));  // vee

  // heading-rate feedforward: world yaw rate rotated into the body frame
  const Eigen::Vector3d w_ff = R.transpose() * R_des * Eigen::Vector3d(0.0, 0.0, yaw_rate);

  const Eigen::Vector3d omega_cmd = -k_R_.cwiseProduct(e_R) + w_ff;  // [rad/s], FLU

  // | ---- encode as CTBR (PX4 runs the rate loop + mixer) ----- |

  const double throttle = std::sqrt(std::clamp(f_coll / collective_thrust_max_, 0.0, 1.0));

  mrs_msgs::msg::HwApiAttitudeRateCmd attitude_rate_cmd;

  attitude_rate_cmd.stamp       = clock_->now();
  attitude_rate_cmd.throttle    = std::clamp(throttle, throttle_clamp_min_, throttle_clamp_max_);
  attitude_rate_cmd.body_rate.x = std::clamp(omega_cmd.x(), omega_output_min_.x(), omega_output_max_.x());
  attitude_rate_cmd.body_rate.y = std::clamp(omega_cmd.y(), omega_output_min_.y(), omega_output_max_.y());
  attitude_rate_cmd.body_rate.z = std::clamp(omega_cmd.z(), omega_output_min_.z(), omega_output_max_.z());

  // | --------- unbiased desired acceleration (fcu frame) ------- |

  {
    const Eigen::Vector3d acc_world = (f_coll / _uav_mass_) * b3 - common_handlers_->g * e3;

    geometry_msgs::msg::Vector3Stamped world_accel;

    world_accel.header.stamp    = clock_->now();
    world_accel.header.frame_id = uav_state.header.frame_id;
    world_accel.vector.x        = acc_world.x();
    world_accel.vector.y        = acc_world.y();
    world_accel.vector.z        = acc_world.z();

    auto res = common_handlers_->transformer->transformSingle(world_accel, "fcu");

    if (res) {
      last_control_output_.desired_unbiased_acceleration = Eigen::Vector3d(res->vector.x, res->vector.y, res->vector.z);
    }
  }

  last_control_output_.control_output         = attitude_rate_cmd;
  last_control_output_.desired_heading_rate   = 0.0;
  last_control_output_.diagnostics.controller = "VelocityController";

  prev_pos_        = p;
  first_iteration_ = false;

  return last_control_output_;
}

//}

/* //{ getStatus() */

const mrs_msgs::msg::ControllerStatus VelocityController::getStatus() {

  mrs_msgs::msg::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void VelocityController::switchOdometrySource([[maybe_unused]] const mrs_msgs::msg::UavState &new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void VelocityController::resetDisturbanceEstimators(void) {
}

//}

/* setConstraints() //{ */

const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>
VelocityController::setConstraints(const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request> &constraints) {

  auto res = std::make_shared<mrs_msgs::srv::DynamicsConstraintsSrv::Response>();

  if (!is_initialized_) {
    return res;
  }

  mrs_lib::set_mutexed(mutex_constraints_, constraints->constraints, constraints_);

  res->success = true;
  res->message = "constraints updated";

  return res;
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* checkGatePassed() //{ */

void VelocityController::checkGatePassed(const Eigen::Vector3d &curr_pos) {

  const Eigen::Vector3d &gate_pos = track_.positions[gate_idx_];
  const Eigen::Matrix3d  RT       = track_.rotmats[gate_idx_].transpose();

  const Eigen::Vector3d prev_rel = RT * (prev_pos_ - gate_pos);
  const Eigen::Vector3d curr_rel = RT * (curr_pos - gate_pos);

  // a pass is the gate plane (its y = 0) being crossed front-wards
  if (!(prev_rel.y() < 0.0 && curr_rel.y() >= 0.0)) {
    return;
  }

  const bool inside_gate = std::abs(curr_rel.x()) < GATE_HALF_WIDTH && std::abs(curr_rel.z()) < GATE_HALF_HEIGHT;

  // a crossing outside the opening still advances to the next gate
  const size_t passed_gate = gate_idx_;
  gate_idx_                = (gate_idx_ + 1) % track_.positions.size();

  if (inside_gate) {
    RCLCPP_INFO(node_->get_logger(), "gate %zu passed ✅", passed_gate);
  } else {
    RCLCPP_WARN(node_->get_logger(), "gate %zu crossed outside the opening ❌", passed_gate);
  }

  if (!lap_started_) {

    lap_started_    = true;
    lap_start_gate_ = passed_gate;
    lap_start_time_ = clock_->now();

    RCLCPP_INFO(node_->get_logger(), "lap clock started at gate %zu", passed_gate);

  } else if (passed_gate == lap_start_gate_) {

    const double lap_time = (clock_->now() - lap_start_time_).seconds();

    lap_count_++;
    lap_start_time_ = clock_->now();

    RCLCPP_INFO(node_->get_logger(), "lap %u finished 🏁 lap time %.3f s", lap_count_, lap_time);
  }
}

//}

}  // namespace lvhr_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(lvhr_controller::VelocityController, mrs_uav_managers::Controller)
