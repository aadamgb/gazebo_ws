
#include <utilities.hpp>

#include <uxr/client/client.h>
#include <ucdr/microcdr.h>

#include <mathlib/mathlib.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/uORB.h>
#include <uORB/ucdr/actuator_motors.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/ucdr/actuator_servos.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/ucdr/airspeed_validated.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/ucdr/arming_check_reply.h>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/ucdr/arming_check_request.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/ucdr/battery_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/ucdr/collision_constraints.h>
#include <uORB/topics/collision_constraints.h>
#include <uORB/ucdr/config_overrides.h>
#include <uORB/topics/config_overrides.h>
#include <uORB/ucdr/distance_sensor.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/ucdr/estimator_status_flags.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/ucdr/failsafe_flags.h>
#include <uORB/topics/failsafe_flags.h>
#include <uORB/ucdr/fixed_wing_lateral_setpoint.h>
#include <uORB/topics/fixed_wing_lateral_setpoint.h>
#include <uORB/ucdr/fixed_wing_longitudinal_setpoint.h>
#include <uORB/topics/fixed_wing_longitudinal_setpoint.h>
#include <uORB/ucdr/gimbal_device_attitude_status.h>
#include <uORB/topics/gimbal_device_attitude_status.h>
#include <uORB/ucdr/goto_setpoint.h>
#include <uORB/topics/goto_setpoint.h>
#include <uORB/ucdr/home_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/ucdr/landing_gear.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/ucdr/lateral_control_configuration.h>
#include <uORB/topics/lateral_control_configuration.h>
#include <uORB/ucdr/longitudinal_control_configuration.h>
#include <uORB/topics/longitudinal_control_configuration.h>
#include <uORB/ucdr/manual_control_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/ucdr/message_format_request.h>
#include <uORB/topics/message_format_request.h>
#include <uORB/ucdr/message_format_response.h>
#include <uORB/topics/message_format_response.h>
#include <uORB/ucdr/mode_completed.h>
#include <uORB/topics/mode_completed.h>
#include <uORB/ucdr/obstacle_distance.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/ucdr/offboard_control_mode.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/ucdr/onboard_computer_status.h>
#include <uORB/topics/onboard_computer_status.h>
#include <uORB/ucdr/position_setpoint_triplet.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/ucdr/register_ext_component_reply.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/ucdr/register_ext_component_request.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/ucdr/rover_attitude_setpoint.h>
#include <uORB/topics/rover_attitude_setpoint.h>
#include <uORB/ucdr/rover_position_setpoint.h>
#include <uORB/topics/rover_position_setpoint.h>
#include <uORB/ucdr/rover_rate_setpoint.h>
#include <uORB/topics/rover_rate_setpoint.h>
#include <uORB/ucdr/rover_speed_setpoint.h>
#include <uORB/topics/rover_speed_setpoint.h>
#include <uORB/ucdr/rover_steering_setpoint.h>
#include <uORB/topics/rover_steering_setpoint.h>
#include <uORB/ucdr/rover_throttle_setpoint.h>
#include <uORB/topics/rover_throttle_setpoint.h>
#include <uORB/ucdr/sensor_combined.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/ucdr/sensor_gps.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/ucdr/sensor_optical_flow.h>
#include <uORB/topics/sensor_optical_flow.h>
#include <uORB/ucdr/telemetry_status.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/ucdr/timesync_status.h>
#include <uORB/topics/timesync_status.h>
#include <uORB/ucdr/trajectory_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/ucdr/transponder_report.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/ucdr/unregister_ext_component.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/ucdr/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/ucdr/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/ucdr/vehicle_command.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/ucdr/vehicle_command_ack.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/ucdr/vehicle_control_mode.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/ucdr/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/ucdr/vehicle_land_detected.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/ucdr/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/ucdr/vehicle_odometry.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/ucdr/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/ucdr/vehicle_status.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/ucdr/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/ucdr/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/ucdr/vtol_vehicle_status.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/ucdr/wind.h>
#include <uORB/topics/wind.h>

#define UXRCE_DEFAULT_POLL_INTERVAL_MS 10

typedef bool (*UcdrSerializeMethod)(const void* data, ucdrBuffer& buf, int64_t time_offset);

static constexpr int max_topic_size = 512;
static_assert(sizeof(register_ext_component_reply_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(arming_check_request_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(mode_completed_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(battery_status_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(collision_constraints_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(estimator_status_flags_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(failsafe_flags_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(manual_control_setpoint_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(message_format_response_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(position_setpoint_triplet_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(sensor_combined_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(timesync_status_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(transponder_report_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(vehicle_land_detected_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(vehicle_attitude_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(vehicle_control_mode_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(vehicle_command_ack_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(vehicle_global_position_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(sensor_gps_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(vehicle_local_position_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(vehicle_odometry_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(vehicle_status_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(airspeed_validated_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(vtol_vehicle_status_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(home_position_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(wind_s) <= max_topic_size, "topic too large, increase max_topic_size");
static_assert(sizeof(gimbal_device_attitude_status_s) <= max_topic_size, "topic too large, increase max_topic_size");



// SFINAE to use R::MESSAGE_VERSION if it exists, and 0 otherwise
template <typename R>
class MessageVersionHelper
{
	template <typename C>
	static constexpr uint32_t get(decltype(&C::MESSAGE_VERSION)) { return C::MESSAGE_VERSION; }
	template <typename C>
	static constexpr uint32_t get(...) { return 0; }
public:
	static constexpr uint32_t m = get<R>(0);
};

template <typename T>
static constexpr uint32_t get_message_version() {
	return MessageVersionHelper<T>::m;
}

struct SendSubscription {
	const struct orb_metadata *orb_meta;
	uxrObjectId data_writer;
	const char* dds_type_name;
	const char* topic;
	uint32_t message_version;
	uint32_t topic_size;
	UcdrSerializeMethod ucdr_serialize_method;
	uint64_t publish_interval_ms;
};

// Subscribers for messages to send
struct SendTopicsSubs {
	SendSubscription send_subscriptions[27] = {
			{ ORB_ID(register_ext_component_reply),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::RegisterExtComponentReply_",
			  "/fmu/out/register_ext_component_reply",
			  get_message_version<register_ext_component_reply_s>(),
			  ucdr_topic_size_register_ext_component_reply(),
			  &ucdr_serialize_register_ext_component_reply,
			  static_cast<uint64_t>((0 > 0) ? (1e3 / 1000.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(arming_check_request),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::ArmingCheckRequest_",
			  "/fmu/out/arming_check_request",
			  get_message_version<arming_check_request_s>(),
			  ucdr_topic_size_arming_check_request(),
			  &ucdr_serialize_arming_check_request,
			  static_cast<uint64_t>((5.0 > 0) ? (1e3 / 5.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(mode_completed),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::ModeCompleted_",
			  "/fmu/out/mode_completed",
			  get_message_version<mode_completed_s>(),
			  ucdr_topic_size_mode_completed(),
			  &ucdr_serialize_mode_completed,
			  static_cast<uint64_t>((50.0 > 0) ? (1e3 / 50.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(battery_status),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::BatteryStatus_",
			  "/fmu/out/battery_status",
			  get_message_version<battery_status_s>(),
			  ucdr_topic_size_battery_status(),
			  &ucdr_serialize_battery_status,
			  static_cast<uint64_t>((1.0 > 0) ? (1e3 / 1.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(collision_constraints),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::CollisionConstraints_",
			  "/fmu/out/collision_constraints",
			  get_message_version<collision_constraints_s>(),
			  ucdr_topic_size_collision_constraints(),
			  &ucdr_serialize_collision_constraints,
			  static_cast<uint64_t>((50.0 > 0) ? (1e3 / 50.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(estimator_status_flags),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::EstimatorStatusFlags_",
			  "/fmu/out/estimator_status_flags",
			  get_message_version<estimator_status_flags_s>(),
			  ucdr_topic_size_estimator_status_flags(),
			  &ucdr_serialize_estimator_status_flags,
			  static_cast<uint64_t>((5.0 > 0) ? (1e3 / 5.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(failsafe_flags),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::FailsafeFlags_",
			  "/fmu/out/failsafe_flags",
			  get_message_version<failsafe_flags_s>(),
			  ucdr_topic_size_failsafe_flags(),
			  &ucdr_serialize_failsafe_flags,
			  static_cast<uint64_t>((5.0 > 0) ? (1e3 / 5.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(manual_control_setpoint),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::ManualControlSetpoint_",
			  "/fmu/out/manual_control_setpoint",
			  get_message_version<manual_control_setpoint_s>(),
			  ucdr_topic_size_manual_control_setpoint(),
			  &ucdr_serialize_manual_control_setpoint,
			  static_cast<uint64_t>((25.0 > 0) ? (1e3 / 25.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(message_format_response),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::MessageFormatResponse_",
			  "/fmu/out/message_format_response",
			  get_message_version<message_format_response_s>(),
			  ucdr_topic_size_message_format_response(),
			  &ucdr_serialize_message_format_response,
			  static_cast<uint64_t>((0 > 0) ? (1e3 / 1000.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(position_setpoint_triplet),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::PositionSetpointTriplet_",
			  "/fmu/out/position_setpoint_triplet",
			  get_message_version<position_setpoint_triplet_s>(),
			  ucdr_topic_size_position_setpoint_triplet(),
			  &ucdr_serialize_position_setpoint_triplet,
			  static_cast<uint64_t>((5.0 > 0) ? (1e3 / 5.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(sensor_combined),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::SensorCombined_",
			  "/fmu/out/sensor_combined",
			  get_message_version<sensor_combined_s>(),
			  ucdr_topic_size_sensor_combined(),
			  &ucdr_serialize_sensor_combined,
			  static_cast<uint64_t>((0 > 0) ? (1e3 / 1000.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(timesync_status),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::TimesyncStatus_",
			  "/fmu/out/timesync_status",
			  get_message_version<timesync_status_s>(),
			  ucdr_topic_size_timesync_status(),
			  &ucdr_serialize_timesync_status,
			  static_cast<uint64_t>((10.0 > 0) ? (1e3 / 10.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(transponder_report),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::TransponderReport_",
			  "/fmu/out/transponder_report",
			  get_message_version<transponder_report_s>(),
			  ucdr_topic_size_transponder_report(),
			  &ucdr_serialize_transponder_report,
			  static_cast<uint64_t>((0 > 0) ? (1e3 / 1000.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(vehicle_land_detected),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::VehicleLandDetected_",
			  "/fmu/out/vehicle_land_detected",
			  get_message_version<vehicle_land_detected_s>(),
			  ucdr_topic_size_vehicle_land_detected(),
			  &ucdr_serialize_vehicle_land_detected,
			  static_cast<uint64_t>((5.0 > 0) ? (1e3 / 5.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(vehicle_attitude),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::VehicleAttitude_",
			  "/fmu/out/vehicle_attitude",
			  get_message_version<vehicle_attitude_s>(),
			  ucdr_topic_size_vehicle_attitude(),
			  &ucdr_serialize_vehicle_attitude,
			  static_cast<uint64_t>((0 > 0) ? (1e3 / 1000.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(vehicle_control_mode),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::VehicleControlMode_",
			  "/fmu/out/vehicle_control_mode",
			  get_message_version<vehicle_control_mode_s>(),
			  ucdr_topic_size_vehicle_control_mode(),
			  &ucdr_serialize_vehicle_control_mode,
			  static_cast<uint64_t>((50.0 > 0) ? (1e3 / 50.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(vehicle_command_ack),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::VehicleCommandAck_",
			  "/fmu/out/vehicle_command_ack",
			  get_message_version<vehicle_command_ack_s>(),
			  ucdr_topic_size_vehicle_command_ack(),
			  &ucdr_serialize_vehicle_command_ack,
			  static_cast<uint64_t>((0 > 0) ? (1e3 / 1000.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(vehicle_global_position),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::VehicleGlobalPosition_",
			  "/fmu/out/vehicle_global_position",
			  get_message_version<vehicle_global_position_s>(),
			  ucdr_topic_size_vehicle_global_position(),
			  &ucdr_serialize_vehicle_global_position,
			  static_cast<uint64_t>((50.0 > 0) ? (1e3 / 50.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(vehicle_gps_position),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::SensorGps_",
			  "/fmu/out/vehicle_gps_position",
			  get_message_version<sensor_gps_s>(),
			  ucdr_topic_size_sensor_gps(),
			  &ucdr_serialize_sensor_gps,
			  static_cast<uint64_t>((50.0 > 0) ? (1e3 / 50.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(vehicle_local_position),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::VehicleLocalPosition_",
			  "/fmu/out/vehicle_local_position",
			  get_message_version<vehicle_local_position_s>(),
			  ucdr_topic_size_vehicle_local_position(),
			  &ucdr_serialize_vehicle_local_position,
			  static_cast<uint64_t>((50.0 > 0) ? (1e3 / 50.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(vehicle_odometry),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::VehicleOdometry_",
			  "/fmu/out/vehicle_odometry",
			  get_message_version<vehicle_odometry_s>(),
			  ucdr_topic_size_vehicle_odometry(),
			  &ucdr_serialize_vehicle_odometry,
			  static_cast<uint64_t>((0 > 0) ? (1e3 / 1000.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(vehicle_status),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::VehicleStatus_",
			  "/fmu/out/vehicle_status",
			  get_message_version<vehicle_status_s>(),
			  ucdr_topic_size_vehicle_status(),
			  &ucdr_serialize_vehicle_status,
			  static_cast<uint64_t>((5.0 > 0) ? (1e3 / 5.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(airspeed_validated),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::AirspeedValidated_",
			  "/fmu/out/airspeed_validated",
			  get_message_version<airspeed_validated_s>(),
			  ucdr_topic_size_airspeed_validated(),
			  &ucdr_serialize_airspeed_validated,
			  static_cast<uint64_t>((50.0 > 0) ? (1e3 / 50.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(vtol_vehicle_status),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::VtolVehicleStatus_",
			  "/fmu/out/vtol_vehicle_status",
			  get_message_version<vtol_vehicle_status_s>(),
			  ucdr_topic_size_vtol_vehicle_status(),
			  &ucdr_serialize_vtol_vehicle_status,
			  static_cast<uint64_t>((0 > 0) ? (1e3 / 1000.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(home_position),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::HomePosition_",
			  "/fmu/out/home_position",
			  get_message_version<home_position_s>(),
			  ucdr_topic_size_home_position(),
			  &ucdr_serialize_home_position,
			  static_cast<uint64_t>((5.0 > 0) ? (1e3 / 5.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(wind),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::Wind_",
			  "/fmu/out/wind",
			  get_message_version<wind_s>(),
			  ucdr_topic_size_wind(),
			  &ucdr_serialize_wind,
			  static_cast<uint64_t>((1.0 > 0) ? (1e3 / 1.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
			{ ORB_ID(gimbal_device_attitude_status),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "px4_msgs::msg::dds_::GimbalDeviceAttitudeStatus_",
			  "/fmu/out/gimbal_device_attitude_status",
			  get_message_version<gimbal_device_attitude_status_s>(),
			  ucdr_topic_size_gimbal_device_attitude_status(),
			  &ucdr_serialize_gimbal_device_attitude_status,
			  static_cast<uint64_t>((20.0 > 0) ? (1e3 / 20.0) : UXRCE_DEFAULT_POLL_INTERVAL_MS),
			},
	};

	px4_pollfd_struct_t fds[27] {};

	uint32_t num_payload_sent{};

	bool init(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId reliable_in_stream_id, uxrStreamId best_effort_in_stream_id, uxrObjectId participant_id, const char *client_namespace);
	void update(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId best_effort_stream_id, uxrObjectId participant_id, const char *client_namespace);
	void reset();
};

bool SendTopicsSubs::init(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId reliable_in_stream_id, uxrStreamId best_effort_in_stream_id, uxrObjectId participant_id, const char *client_namespace) {
	bool ret = true;
	for (unsigned idx = 0; idx < sizeof(send_subscriptions)/sizeof(send_subscriptions[0]); ++idx) {
		if (fds[idx].events == 0) {
			fds[idx].fd = orb_subscribe(send_subscriptions[idx].orb_meta);
			fds[idx].events = POLLIN;
			orb_set_interval(fds[idx].fd, send_subscriptions[idx].publish_interval_ms);
		}

		if (!create_data_writer(session, reliable_out_stream_id, participant_id, static_cast<ORB_ID>(send_subscriptions[idx].orb_meta->o_id), client_namespace, send_subscriptions[idx].topic,
								   send_subscriptions[idx].message_version,
								   send_subscriptions[idx].dds_type_name, send_subscriptions[idx].data_writer)) {
			ret = false;
		}
	}
	return ret;
}

void SendTopicsSubs::reset() {
	num_payload_sent = 0;
	for (unsigned idx = 0; idx < sizeof(send_subscriptions)/sizeof(send_subscriptions[0]); ++idx) {
		send_subscriptions[idx].data_writer = uxr_object_id(0, UXR_INVALID_ID);
		orb_unsubscribe(fds[idx].fd);
		fds[idx].fd = -1;
	}
};

void SendTopicsSubs::update(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId best_effort_stream_id, uxrObjectId participant_id, const char *client_namespace)
{
	int64_t time_offset_us = session->time_offset / 1000; // ns -> us

	alignas(sizeof(uint64_t)) char topic_data[max_topic_size];

	for (unsigned idx = 0; idx < sizeof(send_subscriptions)/sizeof(send_subscriptions[0]); ++idx) {
		if (fds[idx].revents & POLLIN) {
			// Topic updated, copy data and send
			orb_copy(send_subscriptions[idx].orb_meta, fds[idx].fd, &topic_data);

			if (send_subscriptions[idx].data_writer.id != UXR_INVALID_ID) {

				ucdrBuffer ub;
				uint32_t topic_size = send_subscriptions[idx].topic_size;
				if (uxr_prepare_output_stream(session, best_effort_stream_id, send_subscriptions[idx].data_writer, &ub, topic_size) != UXR_INVALID_REQUEST_ID) {
					send_subscriptions[idx].ucdr_serialize_method(&topic_data, ub, time_offset_us);
					// TODO: fill up the MTU and then flush, which reduces the packet overhead
					uxr_flash_output_streams(session);
					num_payload_sent += topic_size;

				} else {
					//PX4_ERR("Error uxr_prepare_output_stream UXR_INVALID_REQUEST_ID %s", send_subscriptions[idx].subscription.get_topic()->o_name);
				}

			} else {
				//PX4_ERR("Error UXR_INVALID_ID %s", send_subscriptions[idx].subscription.get_topic()->o_name);
			}

		}
	}
}

// Publishers for received messages
struct RcvTopicsPubs {
	uORB::Publication<register_ext_component_request_s> register_ext_component_request_pub{ORB_ID(register_ext_component_request)};
	uORB::Publication<unregister_ext_component_s> unregister_ext_component_pub{ORB_ID(unregister_ext_component)};
	uORB::Publication<config_overrides_s> config_overrides_request_pub{ORB_ID(config_overrides_request)};
	uORB::Publication<arming_check_reply_s> arming_check_reply_pub{ORB_ID(arming_check_reply)};
	uORB::Publication<message_format_request_s> message_format_request_pub{ORB_ID(message_format_request)};
	uORB::Publication<mode_completed_s> mode_completed_pub{ORB_ID(mode_completed)};
	uORB::Publication<vehicle_control_mode_s> config_control_setpoints_pub{ORB_ID(config_control_setpoints)};
	uORB::Publication<distance_sensor_s> distance_sensor_pub{ORB_ID(distance_sensor)};
	uORB::Publication<manual_control_setpoint_s> manual_control_input_pub{ORB_ID(manual_control_input)};
	uORB::Publication<offboard_control_mode_s> offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<onboard_computer_status_s> onboard_computer_status_pub{ORB_ID(onboard_computer_status)};
	uORB::Publication<obstacle_distance_s> obstacle_distance_pub{ORB_ID(obstacle_distance)};
	uORB::Publication<sensor_optical_flow_s> sensor_optical_flow_pub{ORB_ID(sensor_optical_flow)};
	uORB::Publication<goto_setpoint_s> goto_setpoint_pub{ORB_ID(goto_setpoint)};
	uORB::Publication<telemetry_status_s> telemetry_status_pub{ORB_ID(telemetry_status)};
	uORB::Publication<trajectory_setpoint_s> trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s> vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_odometry_s> vehicle_mocap_odometry_pub{ORB_ID(vehicle_mocap_odometry)};
	uORB::Publication<vehicle_rates_setpoint_s> vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_odometry_s> vehicle_visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};
	uORB::Publication<vehicle_command_s> vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_s> vehicle_command_mode_executor_pub{ORB_ID(vehicle_command_mode_executor)};
	uORB::Publication<vehicle_thrust_setpoint_s> vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s> vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<actuator_motors_s> actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s> actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<vehicle_global_position_s> aux_global_position_pub{ORB_ID(aux_global_position)};
	uORB::Publication<fixed_wing_longitudinal_setpoint_s> fixed_wing_longitudinal_setpoint_pub{ORB_ID(fixed_wing_longitudinal_setpoint)};
	uORB::Publication<fixed_wing_lateral_setpoint_s> fixed_wing_lateral_setpoint_pub{ORB_ID(fixed_wing_lateral_setpoint)};
	uORB::Publication<longitudinal_control_configuration_s> longitudinal_control_configuration_pub{ORB_ID(longitudinal_control_configuration)};
	uORB::Publication<lateral_control_configuration_s> lateral_control_configuration_pub{ORB_ID(lateral_control_configuration)};
	uORB::Publication<rover_position_setpoint_s> rover_position_setpoint_pub{ORB_ID(rover_position_setpoint)};
	uORB::Publication<rover_speed_setpoint_s> rover_speed_setpoint_pub{ORB_ID(rover_speed_setpoint)};
	uORB::Publication<rover_attitude_setpoint_s> rover_attitude_setpoint_pub{ORB_ID(rover_attitude_setpoint)};
	uORB::Publication<rover_rate_setpoint_s> rover_rate_setpoint_pub{ORB_ID(rover_rate_setpoint)};
	uORB::Publication<rover_throttle_setpoint_s> rover_throttle_setpoint_pub{ORB_ID(rover_throttle_setpoint)};
	uORB::Publication<rover_steering_setpoint_s> rover_steering_setpoint_pub{ORB_ID(rover_steering_setpoint)};
	uORB::Publication<landing_gear_s> landing_gear_pub{ORB_ID(landing_gear)};


	uint32_t num_payload_received{};

	bool init(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId reliable_in_stream_id, uxrStreamId best_effort_in_stream_id, uxrObjectId participant_id, const char *client_namespace);
};

static void on_topic_update(uxrSession *session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id,
		     struct ucdrBuffer *ub, uint16_t length, void *args)
{
	RcvTopicsPubs *pubs = (RcvTopicsPubs *)args;
	const int64_t time_offset_us = session->time_offset / 1000; // ns -> us
	pubs->num_payload_received += length;

	switch (object_id.id) {
	case 0+ (65535U / 32U) + 1: {
			register_ext_component_request_s data;

			if (ucdr_deserialize_register_ext_component_request(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(register_ext_component_request), data);
				pubs->register_ext_component_request_pub.publish(data);
			}
		}
		break;

	case 1+ (65535U / 32U) + 1: {
			unregister_ext_component_s data;

			if (ucdr_deserialize_unregister_ext_component(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(unregister_ext_component), data);
				pubs->unregister_ext_component_pub.publish(data);
			}
		}
		break;

	case 2+ (65535U / 32U) + 1: {
			config_overrides_s data;

			if (ucdr_deserialize_config_overrides(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(config_overrides), data);
				pubs->config_overrides_request_pub.publish(data);
			}
		}
		break;

	case 3+ (65535U / 32U) + 1: {
			arming_check_reply_s data;

			if (ucdr_deserialize_arming_check_reply(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(arming_check_reply), data);
				pubs->arming_check_reply_pub.publish(data);
			}
		}
		break;

	case 4+ (65535U / 32U) + 1: {
			message_format_request_s data;

			if (ucdr_deserialize_message_format_request(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(message_format_request), data);
				pubs->message_format_request_pub.publish(data);
			}
		}
		break;

	case 5+ (65535U / 32U) + 1: {
			mode_completed_s data;

			if (ucdr_deserialize_mode_completed(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(mode_completed), data);
				pubs->mode_completed_pub.publish(data);
			}
		}
		break;

	case 6+ (65535U / 32U) + 1: {
			vehicle_control_mode_s data;

			if (ucdr_deserialize_vehicle_control_mode(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(vehicle_control_mode), data);
				pubs->config_control_setpoints_pub.publish(data);
			}
		}
		break;

	case 7+ (65535U / 32U) + 1: {
			distance_sensor_s data;

			if (ucdr_deserialize_distance_sensor(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(distance_sensor), data);
				pubs->distance_sensor_pub.publish(data);
			}
		}
		break;

	case 8+ (65535U / 32U) + 1: {
			manual_control_setpoint_s data;

			if (ucdr_deserialize_manual_control_setpoint(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(manual_control_setpoint), data);
				pubs->manual_control_input_pub.publish(data);
			}
		}
		break;

	case 9+ (65535U / 32U) + 1: {
			offboard_control_mode_s data;

			if (ucdr_deserialize_offboard_control_mode(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(offboard_control_mode), data);
				pubs->offboard_control_mode_pub.publish(data);
			}
		}
		break;

	case 10+ (65535U / 32U) + 1: {
			onboard_computer_status_s data;

			if (ucdr_deserialize_onboard_computer_status(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(onboard_computer_status), data);
				pubs->onboard_computer_status_pub.publish(data);
			}
		}
		break;

	case 11+ (65535U / 32U) + 1: {
			obstacle_distance_s data;

			if (ucdr_deserialize_obstacle_distance(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(obstacle_distance), data);
				pubs->obstacle_distance_pub.publish(data);
			}
		}
		break;

	case 12+ (65535U / 32U) + 1: {
			sensor_optical_flow_s data;

			if (ucdr_deserialize_sensor_optical_flow(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(sensor_optical_flow), data);
				pubs->sensor_optical_flow_pub.publish(data);
			}
		}
		break;

	case 13+ (65535U / 32U) + 1: {
			goto_setpoint_s data;

			if (ucdr_deserialize_goto_setpoint(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(goto_setpoint), data);
				pubs->goto_setpoint_pub.publish(data);
			}
		}
		break;

	case 14+ (65535U / 32U) + 1: {
			telemetry_status_s data;

			if (ucdr_deserialize_telemetry_status(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(telemetry_status), data);
				pubs->telemetry_status_pub.publish(data);
			}
		}
		break;

	case 15+ (65535U / 32U) + 1: {
			trajectory_setpoint_s data;

			if (ucdr_deserialize_trajectory_setpoint(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(trajectory_setpoint), data);
				pubs->trajectory_setpoint_pub.publish(data);
			}
		}
		break;

	case 16+ (65535U / 32U) + 1: {
			vehicle_attitude_setpoint_s data;

			if (ucdr_deserialize_vehicle_attitude_setpoint(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(vehicle_attitude_setpoint), data);
				pubs->vehicle_attitude_setpoint_pub.publish(data);
			}
		}
		break;

	case 17+ (65535U / 32U) + 1: {
			vehicle_odometry_s data;

			if (ucdr_deserialize_vehicle_odometry(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(vehicle_odometry), data);
				pubs->vehicle_mocap_odometry_pub.publish(data);
			}
		}
		break;

	case 18+ (65535U / 32U) + 1: {
			vehicle_rates_setpoint_s data;

			if (ucdr_deserialize_vehicle_rates_setpoint(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(vehicle_rates_setpoint), data);
				pubs->vehicle_rates_setpoint_pub.publish(data);
			}
		}
		break;

	case 19+ (65535U / 32U) + 1: {
			vehicle_odometry_s data;

			if (ucdr_deserialize_vehicle_odometry(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(vehicle_odometry), data);
				pubs->vehicle_visual_odometry_pub.publish(data);
			}
		}
		break;

	case 20+ (65535U / 32U) + 1: {
			vehicle_command_s data;

			if (ucdr_deserialize_vehicle_command(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(vehicle_command), data);
				pubs->vehicle_command_pub.publish(data);
			}
		}
		break;

	case 21+ (65535U / 32U) + 1: {
			vehicle_command_s data;

			if (ucdr_deserialize_vehicle_command(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(vehicle_command), data);
				pubs->vehicle_command_mode_executor_pub.publish(data);
			}
		}
		break;

	case 22+ (65535U / 32U) + 1: {
			vehicle_thrust_setpoint_s data;

			if (ucdr_deserialize_vehicle_thrust_setpoint(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(vehicle_thrust_setpoint), data);
				pubs->vehicle_thrust_setpoint_pub.publish(data);
			}
		}
		break;

	case 23+ (65535U / 32U) + 1: {
			vehicle_torque_setpoint_s data;

			if (ucdr_deserialize_vehicle_torque_setpoint(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(vehicle_torque_setpoint), data);
				pubs->vehicle_torque_setpoint_pub.publish(data);
			}
		}
		break;

	case 24+ (65535U / 32U) + 1: {
			actuator_motors_s data;

			if (ucdr_deserialize_actuator_motors(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(actuator_motors), data);
				pubs->actuator_motors_pub.publish(data);
			}
		}
		break;

	case 25+ (65535U / 32U) + 1: {
			actuator_servos_s data;

			if (ucdr_deserialize_actuator_servos(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(actuator_servos), data);
				pubs->actuator_servos_pub.publish(data);
			}
		}
		break;

	case 26+ (65535U / 32U) + 1: {
			vehicle_global_position_s data;

			if (ucdr_deserialize_vehicle_global_position(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(vehicle_global_position), data);
				pubs->aux_global_position_pub.publish(data);
			}
		}
		break;

	case 27+ (65535U / 32U) + 1: {
			fixed_wing_longitudinal_setpoint_s data;

			if (ucdr_deserialize_fixed_wing_longitudinal_setpoint(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(fixed_wing_longitudinal_setpoint), data);
				pubs->fixed_wing_longitudinal_setpoint_pub.publish(data);
			}
		}
		break;

	case 28+ (65535U / 32U) + 1: {
			fixed_wing_lateral_setpoint_s data;

			if (ucdr_deserialize_fixed_wing_lateral_setpoint(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(fixed_wing_lateral_setpoint), data);
				pubs->fixed_wing_lateral_setpoint_pub.publish(data);
			}
		}
		break;

	case 29+ (65535U / 32U) + 1: {
			longitudinal_control_configuration_s data;

			if (ucdr_deserialize_longitudinal_control_configuration(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(longitudinal_control_configuration), data);
				pubs->longitudinal_control_configuration_pub.publish(data);
			}
		}
		break;

	case 30+ (65535U / 32U) + 1: {
			lateral_control_configuration_s data;

			if (ucdr_deserialize_lateral_control_configuration(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(lateral_control_configuration), data);
				pubs->lateral_control_configuration_pub.publish(data);
			}
		}
		break;

	case 31+ (65535U / 32U) + 1: {
			rover_position_setpoint_s data;

			if (ucdr_deserialize_rover_position_setpoint(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(rover_position_setpoint), data);
				pubs->rover_position_setpoint_pub.publish(data);
			}
		}
		break;

	case 32+ (65535U / 32U) + 1: {
			rover_speed_setpoint_s data;

			if (ucdr_deserialize_rover_speed_setpoint(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(rover_speed_setpoint), data);
				pubs->rover_speed_setpoint_pub.publish(data);
			}
		}
		break;

	case 33+ (65535U / 32U) + 1: {
			rover_attitude_setpoint_s data;

			if (ucdr_deserialize_rover_attitude_setpoint(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(rover_attitude_setpoint), data);
				pubs->rover_attitude_setpoint_pub.publish(data);
			}
		}
		break;

	case 34+ (65535U / 32U) + 1: {
			rover_rate_setpoint_s data;

			if (ucdr_deserialize_rover_rate_setpoint(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(rover_rate_setpoint), data);
				pubs->rover_rate_setpoint_pub.publish(data);
			}
		}
		break;

	case 35+ (65535U / 32U) + 1: {
			rover_throttle_setpoint_s data;

			if (ucdr_deserialize_rover_throttle_setpoint(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(rover_throttle_setpoint), data);
				pubs->rover_throttle_setpoint_pub.publish(data);
			}
		}
		break;

	case 36+ (65535U / 32U) + 1: {
			rover_steering_setpoint_s data;

			if (ucdr_deserialize_rover_steering_setpoint(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(rover_steering_setpoint), data);
				pubs->rover_steering_setpoint_pub.publish(data);
			}
		}
		break;

	case 37+ (65535U / 32U) + 1: {
			landing_gear_s data;

			if (ucdr_deserialize_landing_gear(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(landing_gear), data);
				pubs->landing_gear_pub.publish(data);
			}
		}
		break;


	default:
		PX4_ERR("unknown object id: %i", object_id.id);
		break;
	}
}

bool RcvTopicsPubs::init(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId reliable_in_stream_id, uxrStreamId best_effort_in_stream_id, uxrObjectId participant_id, const char *client_namespace)
{
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(register_ext_component_request)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<register_ext_component_request_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 0, client_namespace, "/fmu/in/register_ext_component_request", message_version, "px4_msgs::msg::dds_::RegisterExtComponentRequest_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(unregister_ext_component)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<unregister_ext_component_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 1, client_namespace, "/fmu/in/unregister_ext_component", message_version, "px4_msgs::msg::dds_::UnregisterExtComponent_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(config_overrides)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<config_overrides_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 2, client_namespace, "/fmu/in/config_overrides_request", message_version, "px4_msgs::msg::dds_::ConfigOverrides_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(arming_check_reply)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<arming_check_reply_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 3, client_namespace, "/fmu/in/arming_check_reply", message_version, "px4_msgs::msg::dds_::ArmingCheckReply_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(message_format_request)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<message_format_request_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 4, client_namespace, "/fmu/in/message_format_request", message_version, "px4_msgs::msg::dds_::MessageFormatRequest_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(mode_completed)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<mode_completed_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 5, client_namespace, "/fmu/in/mode_completed", message_version, "px4_msgs::msg::dds_::ModeCompleted_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(vehicle_control_mode)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<vehicle_control_mode_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 6, client_namespace, "/fmu/in/config_control_setpoints", message_version, "px4_msgs::msg::dds_::VehicleControlMode_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(distance_sensor)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<distance_sensor_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 7, client_namespace, "/fmu/in/distance_sensor", message_version, "px4_msgs::msg::dds_::DistanceSensor_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(manual_control_setpoint)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<manual_control_setpoint_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 8, client_namespace, "/fmu/in/manual_control_input", message_version, "px4_msgs::msg::dds_::ManualControlSetpoint_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(offboard_control_mode)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<offboard_control_mode_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 9, client_namespace, "/fmu/in/offboard_control_mode", message_version, "px4_msgs::msg::dds_::OffboardControlMode_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(onboard_computer_status)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<onboard_computer_status_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 10, client_namespace, "/fmu/in/onboard_computer_status", message_version, "px4_msgs::msg::dds_::OnboardComputerStatus_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(obstacle_distance)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<obstacle_distance_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 11, client_namespace, "/fmu/in/obstacle_distance", message_version, "px4_msgs::msg::dds_::ObstacleDistance_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(sensor_optical_flow)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<sensor_optical_flow_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 12, client_namespace, "/fmu/in/sensor_optical_flow", message_version, "px4_msgs::msg::dds_::SensorOpticalFlow_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(goto_setpoint)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<goto_setpoint_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 13, client_namespace, "/fmu/in/goto_setpoint", message_version, "px4_msgs::msg::dds_::GotoSetpoint_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(telemetry_status)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<telemetry_status_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 14, client_namespace, "/fmu/in/telemetry_status", message_version, "px4_msgs::msg::dds_::TelemetryStatus_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(trajectory_setpoint)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<trajectory_setpoint_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 15, client_namespace, "/fmu/in/trajectory_setpoint", message_version, "px4_msgs::msg::dds_::TrajectorySetpoint_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(vehicle_attitude_setpoint)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<vehicle_attitude_setpoint_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 16, client_namespace, "/fmu/in/vehicle_attitude_setpoint", message_version, "px4_msgs::msg::dds_::VehicleAttitudeSetpoint_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(vehicle_odometry)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<vehicle_odometry_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 17, client_namespace, "/fmu/in/vehicle_mocap_odometry", message_version, "px4_msgs::msg::dds_::VehicleOdometry_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(vehicle_rates_setpoint)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<vehicle_rates_setpoint_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 18, client_namespace, "/fmu/in/vehicle_rates_setpoint", message_version, "px4_msgs::msg::dds_::VehicleRatesSetpoint_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(vehicle_odometry)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<vehicle_odometry_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 19, client_namespace, "/fmu/in/vehicle_visual_odometry", message_version, "px4_msgs::msg::dds_::VehicleOdometry_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(vehicle_command)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<vehicle_command_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 20, client_namespace, "/fmu/in/vehicle_command", message_version, "px4_msgs::msg::dds_::VehicleCommand_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(vehicle_command)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<vehicle_command_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 21, client_namespace, "/fmu/in/vehicle_command_mode_executor", message_version, "px4_msgs::msg::dds_::VehicleCommand_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(vehicle_thrust_setpoint)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<vehicle_thrust_setpoint_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 22, client_namespace, "/fmu/in/vehicle_thrust_setpoint", message_version, "px4_msgs::msg::dds_::VehicleThrustSetpoint_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(vehicle_torque_setpoint)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<vehicle_torque_setpoint_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 23, client_namespace, "/fmu/in/vehicle_torque_setpoint", message_version, "px4_msgs::msg::dds_::VehicleTorqueSetpoint_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(actuator_motors)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<actuator_motors_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 24, client_namespace, "/fmu/in/actuator_motors", message_version, "px4_msgs::msg::dds_::ActuatorMotors_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(actuator_servos)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<actuator_servos_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 25, client_namespace, "/fmu/in/actuator_servos", message_version, "px4_msgs::msg::dds_::ActuatorServos_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(vehicle_global_position)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<vehicle_global_position_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 26, client_namespace, "/fmu/in/aux_global_position", message_version, "px4_msgs::msg::dds_::VehicleGlobalPosition_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(fixed_wing_longitudinal_setpoint)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<fixed_wing_longitudinal_setpoint_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 27, client_namespace, "/fmu/in/fixed_wing_longitudinal_setpoint", message_version, "px4_msgs::msg::dds_::FixedWingLongitudinalSetpoint_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(fixed_wing_lateral_setpoint)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<fixed_wing_lateral_setpoint_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 28, client_namespace, "/fmu/in/fixed_wing_lateral_setpoint", message_version, "px4_msgs::msg::dds_::FixedWingLateralSetpoint_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(longitudinal_control_configuration)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<longitudinal_control_configuration_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 29, client_namespace, "/fmu/in/longitudinal_control_configuration", message_version, "px4_msgs::msg::dds_::LongitudinalControlConfiguration_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(lateral_control_configuration)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<lateral_control_configuration_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 30, client_namespace, "/fmu/in/lateral_control_configuration", message_version, "px4_msgs::msg::dds_::LateralControlConfiguration_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(rover_position_setpoint)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<rover_position_setpoint_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 31, client_namespace, "/fmu/in/rover_position_setpoint", message_version, "px4_msgs::msg::dds_::RoverPositionSetpoint_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(rover_speed_setpoint)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<rover_speed_setpoint_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 32, client_namespace, "/fmu/in/rover_speed_setpoint", message_version, "px4_msgs::msg::dds_::RoverSpeedSetpoint_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(rover_attitude_setpoint)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<rover_attitude_setpoint_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 33, client_namespace, "/fmu/in/rover_attitude_setpoint", message_version, "px4_msgs::msg::dds_::RoverAttitudeSetpoint_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(rover_rate_setpoint)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<rover_rate_setpoint_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 34, client_namespace, "/fmu/in/rover_rate_setpoint", message_version, "px4_msgs::msg::dds_::RoverRateSetpoint_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(rover_throttle_setpoint)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<rover_throttle_setpoint_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 35, client_namespace, "/fmu/in/rover_throttle_setpoint", message_version, "px4_msgs::msg::dds_::RoverThrottleSetpoint_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(rover_steering_setpoint)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<rover_steering_setpoint_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 36, client_namespace, "/fmu/in/rover_steering_setpoint", message_version, "px4_msgs::msg::dds_::RoverSteeringSetpoint_", queue_depth);
	}
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(landing_gear)) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<landing_gear_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, 37, client_namespace, "/fmu/in/landing_gear", message_version, "px4_msgs::msg::dds_::LandingGear_", queue_depth);
	}

	uxr_set_topic_callback(session, on_topic_update, this);

	return true;
}
