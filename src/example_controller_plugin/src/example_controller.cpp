/* includes //{*/

#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_managers/controller.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/dynparam_mgr.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

//}

namespace example_controller_plugin
{

/* DrsParams_t //{ */

struct DrsParams_t
{
  double gain_position;
  double gain_velocity;
  double gain_attitude;
  double gain_attitude_rate;
};

//}

/* //{ class ExampleController */

class ExampleController : public mrs_uav_managers::Controller {

public:
  bool initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);

  bool activate(const ControlOutput &last_control_output);

  void deactivate(void);

  void destroy();

  void updateInactive(const mrs_msgs::msg::UavState &uav_state, const std::optional<mrs_msgs::msg::TrackerCommand> &tracker_command);

  ControlOutput updateActive(const mrs_msgs::msg::UavState &uav_state, const mrs_msgs::msg::TrackerCommand &tracker_command);

  const mrs_msgs::msg::ControllerStatus getStatus();

  void switchOdometrySource(const mrs_msgs::msg::UavState &new_uav_state);

  void resetDisturbanceEstimators(void);

  Eigen::Vector3d orientationError(const Eigen::Matrix3d &R, const Eigen::Matrix3d &Rd);

  Eigen::Matrix3d so3transform(const rclcpp::Node::SharedPtr &node, const Eigen::Vector3d &body_z, const ::Eigen::Vector3d &heading,
                               const bool &preserve_heading);

  std::optional<mrs_msgs::msg::HwApiAttitudeRateCmd> attitudeController(const rclcpp::Node::SharedPtr &node, const mrs_msgs::msg::UavState &uav_state,
                                                                        const mrs_msgs::msg::HwApiAttitudeCmd &reference, const Eigen::Vector3d &ff_rate,
                                                                        const Eigen::Vector3d &rate_saturation, const Eigen::Vector3d &gains,
                                                                        const bool &parasitic_heading_rate_compensation);

  std::optional<mrs_msgs::msg::HwApiControlGroupCmd> attitudeRateController(const rclcpp::Node::SharedPtr &node, const mrs_msgs::msg::UavState &uav_state,
                                                                            const mrs_msgs::msg::HwApiAttitudeRateCmd &reference, const Eigen::Vector3d &gains);

  mrs_msgs::msg::HwApiActuatorCmd actuatorMixer(const rclcpp::Node::SharedPtr &node, const mrs_msgs::msg::HwApiControlGroupCmd &ctrl_group_cmd,
                                                const Eigen::MatrixXd &mixer);

  const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>
  setConstraints(const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request> &constraints);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::msg::UavState uav_state_;
  std::mutex              mutex_uav_state_;

  // | --------------- dynamic reconfigure server --------------- |

  std::shared_ptr<mrs_lib::DynparamMgr> dynparam_mgr_;
  std::mutex                            mutex_dynparam_mgr_;
  DrsParams_t                           drs_params_;

  // | ----------------------- constraints ---------------------- |

  mrs_msgs::msg::DynamicsConstraints constraints_;
  std::mutex                         mutex_constraints_;

  // | --------- throttle generation and mass estimation -------- |

  double _uav_mass_;

  // | ------------------ activation and output ----------------- |

  ControlOutput last_control_output_;
  ControlOutput activation_control_output_;

  rclcpp::Time      last_update_time_;
  std::atomic<bool> first_iteration_ = true;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

bool ExampleController::initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                                   std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  node_  = node;
  clock_ = node_->get_clock();

  common_handlers_  = common_handlers;
  private_handlers_ = private_handlers;

  _uav_mass_ = common_handlers->getMass();

  last_update_time_ = clock_->now();

  // | --------------------- load parameters -------------------- |

  // add yaml config from the package's config folder
  private_handlers->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory("example_controller_plugin") + "/config/example_controller.yaml");

  // setup the dynamical reconfigure parameter manager
  dynparam_mgr_ = std::make_shared<mrs_lib::DynparamMgr>(node_, mutex_dynparam_mgr_);

  // copy loaded yaml files from the param loader to the dynparam mgr
  dynparam_mgr_->get_param_provider().copyYamls(private_handlers->param_loader->getParamProvider());

  // the parameter is loaded from the namespace of the plugin
  //    ... that is mrs_uav_managers/example_controller

  // setup dynamically-reconfigurable parameters (gains)
  dynparam_mgr_->register_param("gains/position", &drs_params_.gain_position, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));
  dynparam_mgr_->register_param("gains/velocity", &drs_params_.gain_velocity, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));
  dynparam_mgr_->register_param("gains/attitude", &drs_params_.gain_attitude, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));
  dynparam_mgr_->register_param("gains/attitude_rate", &drs_params_.gain_attitude_rate, mrs_lib::DynparamMgr::range_t<double>(0.0, 100.0));

  // | ------------------ finish loading params ----------------- |

  if (!private_handlers->param_loader->loadedSuccessfully() || !dynparam_mgr_->loaded_successfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load all parameters!");
    return false;
  }

  // | ----------------------- finish init ---------------------- |

  RCLCPP_INFO(node_->get_logger(), "initialized");

  is_initialized_ = true;

  return true;
}

//}

/* //{ activate() */

bool ExampleController::activate(const ControlOutput &last_control_output) {

  activation_control_output_ = last_control_output;

  first_iteration_ = true;
  is_active_       = true;

  RCLCPP_INFO(node_->get_logger(), "activated");

  return true;
}

//}

/* //{ deactivate() */

void ExampleController::deactivate(void) {

  is_active_       = false;
  first_iteration_ = false;

  RCLCPP_INFO(node_->get_logger(), "deactivated");
}

//}

/* //{ destroy() */

void ExampleController::destroy(void) {

  is_active_       = false;
  first_iteration_ = false;

  RCLCPP_INFO(node_->get_logger(), "destroyed");
}

//}

/* updateInactive() //{ */

void ExampleController::updateInactive(const mrs_msgs::msg::UavState                                       &uav_state,
                                       [[maybe_unused]] const std::optional<mrs_msgs::msg::TrackerCommand> &tracker_command) {

  mrs_lib::set_mutexed(mutex_dynparam_mgr_, uav_state, uav_state_);

  last_update_time_ = uav_state.header.stamp;

  first_iteration_ = false;
}

//}

/* //{ updateActive() */

ExampleController::ControlOutput ExampleController::updateActive(const mrs_msgs::msg::UavState       &uav_state,
                                                                 const mrs_msgs::msg::TrackerCommand &tracker_command) {

  auto drs_params  = mrs_lib::get_mutexed(mutex_dynparam_mgr_, drs_params_);
  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  // clear all the optional parts of the result
  last_control_output_.desired_heading_rate          = {};
  last_control_output_.desired_orientation           = {};
  last_control_output_.desired_unbiased_acceleration = {};
  last_control_output_.control_output                = {};

  if (!is_active_) {
    return last_control_output_;
  }

  // | ---------- calculate dt from the last iteration ---------- |

  double dt;

  if (first_iteration_) {
    dt               = 0.01;
    first_iteration_ = false;
  } else {
    dt = rclcpp::Time(uav_state.header.stamp).seconds() - last_update_time_.seconds();
  }

  last_update_time_ = rclcpp::Time(uav_state.header.stamp);

  if (fabs(dt) < 0.001) {

    RCLCPP_WARN(node_->get_logger(), "the last odometry message came too close (%.2f s)!", dt);
    dt = 0.01;
  }

  // | -------- check for the available output modalities ------- |

  // you can decide what to return, but it needs to be available
  if (common_handlers_->control_output_modalities.attitude) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "desired attitude output modality is available");
  }

  // | ---------- extract the detailed model parameters --------- |

  if (common_handlers_->detailed_model_params) {

    mrs_uav_managers::control_manager::DetailedModelParams_t detailed_model_params = common_handlers_->detailed_model_params.value();

    RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "UAV inertia is: " << detailed_model_params.inertia);
  }

  // | -------------- prepare the control reference ------------- |

  geometry_msgs::msg::PoseStamped position_reference;

  position_reference.header           = tracker_command.header;
  position_reference.pose.position    = tracker_command.position;
  position_reference.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(tracker_command.heading);

  // | ----------------- prepare control errors ----------------- |

  double uav_heading = 0;

  {
    try {
      uav_heading = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
    }
    catch (...) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "failed to calculate UAV's heading");

      // let's set this to the reference, this will cause 0 control error and therefore no wierd control action
      uav_heading = tracker_command.heading;
    }
  }

  Eigen::Vector3d ep;
  ep << tracker_command.position.x - uav_state.pose.position.x, tracker_command.position.y - uav_state.pose.position.y,
      tracker_command.position.z - uav_state.pose.position.z;

  Eigen::Vector3d ev;
  ev << tracker_command.velocity.x - uav_state.velocity.linear.x, tracker_command.velocity.y - uav_state.velocity.linear.y,
      tracker_command.velocity.z - uav_state.velocity.linear.z;

  // | ------------- position and velocity feedback ------------- |

  Eigen::Vector3d desired_a = ep * drs_params.gain_position * _uav_mass_ + ev * drs_params.gain_velocity * _uav_mass_;

  // | --------------------- add feedforward -------------------- |

  if (tracker_command.use_acceleration) {
    desired_a(0) += tracker_command.acceleration.x;
    desired_a(1) += tracker_command.acceleration.y;
    desired_a(2) += tracker_command.acceleration.z;
  }

  // | ---------------------- desired force --------------------- |

  Eigen::Vector3d desired_f = (desired_a + Eigen::Vector3d(0, 0, common_handlers_->g)) * _uav_mass_;

  Eigen::Vector3d desired_f_normed = desired_f;
  desired_f_normed.normalized();

  // | ----------------------- so3 control ---------------------- |

  Eigen::Vector3d bxd; // desired heading vector

  if (tracker_command.use_heading) {
    bxd << cos(tracker_command.heading), sin(tracker_command.heading), 0;
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 10000, "[Se3Controller]: desired heading was not specified, using current heading instead!");
    bxd << cos(uav_heading), sin(uav_heading), 0;
  }

  Eigen::Matrix3d Rd = so3transform(node_, desired_f_normed, bxd, true);

  // | -------------------- desired throttle -------------------- |

  // current orientation
  const Eigen::Matrix3d R = mrs_lib::AttitudeConverter(uav_state.pose.orientation);

  const double desired_thrust_force = desired_f.dot(R.col(2));

  double throttle = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, desired_thrust_force, *node_);

  // --------------------------------------------------------------
  // |                      attitude control                      |
  // --------------------------------------------------------------

  Eigen::Vector3d attitude_rate_saturation(constraints.roll_rate, constraints.pitch_rate, constraints.yaw_rate);
  Eigen::Vector3d attitude_p_gain(drs_params.gain_attitude * _uav_mass_, drs_params.gain_attitude * _uav_mass_, drs_params.gain_attitude * _uav_mass_);
  Eigen::Vector3d rate_feedforward(0, 0, 0);

  mrs_msgs::msg::HwApiAttitudeCmd attitude_cmd;

  attitude_cmd.stamp       = clock_->now();
  attitude_cmd.orientation = mrs_lib::AttitudeConverter(Rd);
  attitude_cmd.throttle    = throttle;

  auto attitude_rate_command = attitudeController(node_, uav_state, attitude_cmd, rate_feedforward, attitude_rate_saturation, attitude_p_gain, false);

  // --------------------------------------------------------------
  // |                    Attitude rate control                   |
  // --------------------------------------------------------------

  Eigen::Array3d attitude_rate_p_gain = common_handlers_->detailed_model_params->inertia.diagonal().array() * drs_params.gain_attitude_rate;

  auto control_group_command = attitudeRateController(node_, uav_state, attitude_rate_command.value(), attitude_rate_p_gain);

  // --------------------------------------------------------------
  // |                        output mixer                        |
  // --------------------------------------------------------------

  mrs_msgs::msg::HwApiActuatorCmd actuator_cmd =
      actuatorMixer(node_, control_group_command.value(), common_handlers_->detailed_model_params->control_group_mixer);

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 100, "[ExampleController]: motor output:\nmotor1: %.2f\nmotor2: %.2f\nmotor3: %.2f\nmotor4: %.2f", actuator_cmd.motors[0], actuator_cmd.motors[1],
                       actuator_cmd.motors[2], actuator_cmd.motors[3]);

  // | ----------------- set the control output ----------------- |

  // the following types can be set as control_output
  // ------------------------------------------------------
  // mrs_msgs::msg::HwApiPositionCmd
  // mrs_msgs::msg::HwApiVelocityHdgCmd
  // mrs_msgs::msg::HwApiVelocityHdgRateCmd
  // mrs_msgs::msg::HwApiAccelerationHdgCmd
  // mrs_msgs::msg::HwApiAccelerationHdgRateCmd
  // mrs_msgs::msg::HwApiAttitudeCmd
  // mrs_msgs::msg::HwApiAttitudeRateCmd
  // mrs_msgs::msg::HwApiControlGroupCmd
  // mrs_msgs::msg::HwApiActuatorCmd
  // ------------------------------------------------------
  // - the controller can check the "capabilities" of the underlying
  //   hardware in common_handlers_->control_output_modalities
  // - the controller can output any of the available modalities based
  //   on your choice
  // - the modalities that your controller is aable to output should be
  //   listed in it's configuration in the custom_config
  // - if the controller's modalities listed in custom_config do have
  //   zero overlap with the HW API's offered modalities, the controller
  //   won't be loaded
  // ------------------------------------------------------

  last_control_output_.control_output = actuator_cmd;

  // --------------------------------------------------------------
  // |                 fill in the optional parts                 |
  // --------------------------------------------------------------

  // it is recommended to fill the optional parts if you know them

  // | ------------------- desired orientation ------------------ |

  /// the "desired_orientation" is used for:
  // * plotting the orientation in the control_manager/control_refence topic
  // * checking for attitude control error, which can trigger eland/failsafe
  // last_control_output_.desired_orientation = mrs_lib::AttitudeConverter(...);

  // | -------------- unbiased desired acceleration ------------- |

  /// IMPORANT
  // The acceleration and heading rate in 3D (expressed in the "fcu" frame of reference)
  // that the UAV will actually undergo due to the control action.
  Eigen::Vector3d unbiased_des_acc(0, 0, 0);

  {
    Eigen::Vector3d unbiased_des_acc_world(desired_a(0), desired_a(1), desired_a(2));

    geometry_msgs::msg::Vector3Stamped world_accel;

    world_accel.header.stamp    = clock_->now();
    world_accel.header.frame_id = uav_state.header.frame_id;
    world_accel.vector.x        = unbiased_des_acc_world(0);
    world_accel.vector.y        = unbiased_des_acc_world(1);
    world_accel.vector.z        = unbiased_des_acc_world(2);

    auto res = common_handlers_->transformer->transformSingle(world_accel, "fcu");

    if (res) {
      unbiased_des_acc << res.value().vector.x, res.value().vector.y, res.value().vector.z;
    }
  }

  // fill the unbiased desired acceleration
  last_control_output_.desired_unbiased_acceleration = unbiased_des_acc;

  // | ----------------- fill in the diagnostics ---------------- |

  last_control_output_.diagnostics.controller = "ExampleController";

  return last_control_output_;
}

//}

/* //{ getStatus() */

const mrs_msgs::msg::ControllerStatus ExampleController::getStatus() {

  mrs_msgs::msg::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void ExampleController::switchOdometrySource([[maybe_unused]] const mrs_msgs::msg::UavState &new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void ExampleController::resetDisturbanceEstimators(void) {
}

//}

/* setConstraints() //{ */

const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>
ExampleController::setConstraints([[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request> &constraints) {

  if (!is_initialized_) {
    return std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>(new mrs_msgs::srv::DynamicsConstraintsSrv::Response());
  }

  mrs_lib::set_mutexed(mutex_constraints_, constraints->constraints, constraints_);

  RCLCPP_INFO(node_->get_logger(), "updating constraints");

  mrs_msgs::srv::DynamicsConstraintsSrv::Response res;
  res.success = true;
  res.message = "constraints updated";

  return std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>(new mrs_msgs::srv::DynamicsConstraintsSrv::Response(res));
}

//}

/* orientationError() //{ */

Eigen::Vector3d ExampleController::orientationError(const Eigen::Matrix3d &R, const Eigen::Matrix3d &Rd) {

  // orientation error
  Eigen::Matrix3d R_error = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);

  // vectorize the orientation error
  // clang-format off
    Eigen::Vector3d R_error_vec;
    R_error_vec << (R_error(1, 2) - R_error(2, 1)) / 2.0,
                   (R_error(2, 0) - R_error(0, 2)) / 2.0,
                   (R_error(0, 1) - R_error(1, 0)) / 2.0;
  // clang-format on

  return R_error_vec;
}

//}

/* so3transform() //{ */

Eigen::Matrix3d ExampleController::so3transform(const rclcpp::Node::SharedPtr &node, const Eigen::Vector3d &body_z, const ::Eigen::Vector3d &heading,
                                                const bool &preserve_heading) {

  Eigen::Vector3d body_z_normed = body_z.normalized();

  Eigen::Matrix3d Rd;

  if (preserve_heading) {

    RCLCPP_DEBUG_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "[SO3Transform]: using Baca's method");

    // | ------------------------- body z ------------------------- |
    Rd.col(2) = body_z_normed;

    // | ------------------------- body x ------------------------- |

    // construct the oblique projection
    Eigen::Matrix3d projector_body_z_compl = (Eigen::Matrix3d::Identity(3, 3) - body_z_normed * body_z_normed.transpose());

    // create a basis of the body-z complement subspace
    Eigen::MatrixXd A = Eigen::MatrixXd(3, 2);
    A.col(0)          = projector_body_z_compl.col(0);
    A.col(1)          = projector_body_z_compl.col(1);

    // create the basis of the projection null-space complement
    Eigen::MatrixXd B = Eigen::MatrixXd(3, 2);
    B.col(0)          = Eigen::Vector3d(1, 0, 0);
    B.col(1)          = Eigen::Vector3d(0, 1, 0);

    // oblique projector to <range_basis>
    Eigen::MatrixXd Bt_A               = B.transpose() * A;
    Eigen::MatrixXd Bt_A_pseudoinverse = ((Bt_A.transpose() * Bt_A).inverse()) * Bt_A.transpose();
    Eigen::MatrixXd oblique_projector  = A * Bt_A_pseudoinverse * B.transpose();

    Rd.col(0) = oblique_projector * heading;
    Rd.col(0).normalize();

    // | ------------------------- body y ------------------------- |

    Rd.col(1) = Rd.col(2).cross(Rd.col(0));
    Rd.col(1).normalize();

  } else {

    RCLCPP_DEBUG_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "[SO3Transform]: using Lee's method");

    Rd.col(2) = body_z_normed;
    Rd.col(1) = Rd.col(2).cross(heading);
    Rd.col(1).normalize();
    Rd.col(0) = Rd.col(1).cross(Rd.col(2));
    Rd.col(0).normalize();
  }

  return Rd;
}

//}

/* attitudeController() //{ */

std::optional<mrs_msgs::msg::HwApiAttitudeRateCmd>
ExampleController::attitudeController(const rclcpp::Node::SharedPtr &node, const mrs_msgs::msg::UavState &uav_state,
                                      const mrs_msgs::msg::HwApiAttitudeCmd &reference, const Eigen::Vector3d &ff_rate, const Eigen::Vector3d &rate_saturation,
                                      const Eigen::Vector3d &gains, const bool &parasitic_heading_rate_compensation) {

  Eigen::Matrix3d R  = mrs_lib::AttitudeConverter(uav_state.pose.orientation);
  Eigen::Matrix3d Rd = mrs_lib::AttitudeConverter(reference.orientation);

  // calculate the orientation error
  Eigen::Vector3d E = orientationError(R, Rd);

  Eigen::Vector3d rate_feedback = gains.array() * E.array() + ff_rate.array();

  // | ----------- parasitic heading rate compensation ---------- |

  if (parasitic_heading_rate_compensation) {

    RCLCPP_DEBUG_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "[AttitudeController]: parasitic heading rate compensation enabled");

    // compensate for the parasitic heading rate created by the desired pitch and roll rate
    Eigen::Vector3d rp_heading_rate_compensation(0, 0, 0);

    Eigen::Vector3d q_feedback_yawless = rate_feedback;
    q_feedback_yawless(2)              = 0; // nullyfy the effect of the original yaw feedback

    double parasitic_heading_rate = 0;

    try {
      parasitic_heading_rate = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeadingRate(q_feedback_yawless);
    }
    catch (...) {
      RCLCPP_ERROR(node->get_logger(), "[AttitudeController]: exception caught while calculating the parasitic heading rate!");
    }

    try {
      rp_heading_rate_compensation(2) = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getYawRateIntrinsic(-parasitic_heading_rate);
    }
    catch (...) {
      RCLCPP_ERROR(node->get_logger(), "[AttitudeController]: exception caught while calculating the parasitic heading rate compensation!");
    }

    rate_feedback += rp_heading_rate_compensation;
  }

  // | --------------- saturate the attitude rate --------------- |

  if (rate_feedback(0) > rate_saturation(0)) {
    rate_feedback(0) = rate_saturation(0);
  } else if (rate_feedback(0) < -rate_saturation(0)) {
    rate_feedback(0) = -rate_saturation(0);
  }

  if (rate_feedback(1) > rate_saturation(1)) {
    rate_feedback(1) = rate_saturation(1);
  } else if (rate_feedback(1) < -rate_saturation(1)) {
    rate_feedback(1) = -rate_saturation(1);
  }

  if (rate_feedback(2) > rate_saturation(2)) {
    rate_feedback(2) = rate_saturation(2);
  } else if (rate_feedback(2) < -rate_saturation(2)) {
    rate_feedback(2) = -rate_saturation(2);
  }

  // | ------------ fill in the attitude rate command ----------- |

  mrs_msgs::msg::HwApiAttitudeRateCmd cmd;

  cmd.stamp = node->get_clock()->now();

  cmd.body_rate.x = rate_feedback(0);
  cmd.body_rate.y = rate_feedback(1);
  cmd.body_rate.z = rate_feedback(2);

  cmd.throttle = reference.throttle;

  return cmd;
}

//}

/* attitudeRateController() //{ */

std::optional<mrs_msgs::msg::HwApiControlGroupCmd> ExampleController::attitudeRateController(const rclcpp::Node::SharedPtr             &node,
                                                                                             const mrs_msgs::msg::UavState             &uav_state,
                                                                                             const mrs_msgs::msg::HwApiAttitudeRateCmd &reference,
                                                                                             const Eigen::Vector3d                     &gains) {

  Eigen::Vector3d des_rate(reference.body_rate.x, reference.body_rate.y, reference.body_rate.z);
  Eigen::Vector3d cur_rate(uav_state.velocity.angular.x, uav_state.velocity.angular.y, uav_state.velocity.angular.z);

  Eigen::Vector3d ctrl_group_action = (des_rate - cur_rate).array() * gains.array();

  mrs_msgs::msg::HwApiControlGroupCmd cmd;

  cmd.stamp = node->get_clock()->now();

  cmd.throttle = reference.throttle;

  cmd.roll  = ctrl_group_action(0);
  cmd.pitch = ctrl_group_action(1);
  cmd.yaw   = ctrl_group_action(2);

  return {cmd};
}

//}

/* actuatorMixer() //{ */

mrs_msgs::msg::HwApiActuatorCmd ExampleController::actuatorMixer(const rclcpp::Node::SharedPtr &node, const mrs_msgs::msg::HwApiControlGroupCmd &ctrl_group_cmd,
                                                                 const Eigen::MatrixXd &mixer) {

  Eigen::Vector4d ctrl_group(ctrl_group_cmd.roll, ctrl_group_cmd.pitch, ctrl_group_cmd.yaw, ctrl_group_cmd.throttle);

  Eigen::VectorXd motors = mixer * ctrl_group;

  double min = motors.minCoeff();

  if (min < 0.0) {
    motors.array() += abs(min);
  }

  double max = motors.maxCoeff();

  if (max > 1.0) {

    if (ctrl_group_cmd.throttle > 1e-2) {

      // scale down roll/pitch/yaw actions to maintain desired throttle
      for (int i = 0; i < 3; i++) {
        ctrl_group(i) = ctrl_group(i) / (motors.mean() / ctrl_group_cmd.throttle);
      }

      motors = mixer * ctrl_group;

    } else {
      motors /= max;
    }
  }

  // | --------------------- fill in the msg -------------------- |

  mrs_msgs::msg::HwApiActuatorCmd actuator_msg;

  actuator_msg.stamp = node->get_clock()->now();

  for (int i = 0; i < motors.size(); i++) {
    actuator_msg.motors.push_back(motors(i));
  }

  return actuator_msg;
}

//}

} // namespace example_controller_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(example_controller_plugin::ExampleController, mrs_uav_managers::Controller)
