#!/usr/bin/env python3
"""
Send the UAV to random positions inside a box. When the UAV gets within
`radius` of the current goal (or `timeout` expires), a new goal is sampled.
Goals go straight to RLGoto (/<uav>/control_manager/rl_goto/goal), bypassing the
MRS tracker like the genesis env; `-p direct:=false` sends them through the tracker.
A fixed `seed` makes the goal sequence reproducible.

By default (relative:=true) the box is centred on the position of the UAV when this
script starts: a 1 x 1 m square in the plane of the UAV, so the goals stay close to
wherever the UAV hovers when RLGoto is switched on. The x/y/z bounds are then offsets
from that position; with relative:=false they are absolute positions as before.

Run (from a shell with the same env as the tmux panes):
  ./random_goto.py
  ./random_goto.py --ros-args -p seed:=0
  ./random_goto.py --ros-args -p radius:=0.5 -p x_min:=-1.0 -p x_max:=1.0
  ./random_goto.py --ros-args -p relative:=false -p z_min:=4.0 -p z_max:=4.0
  ./random_goto.py --ros-args -p ground_truth:=true   # simulation only

Safety guard: a ⚠️ warning is printed while the UAV is further than max_distance (4 m)
from the centre of the goal box, e.g. when the controller overshoots. If RLGoto is the
active controller at that moment, the UAV is switched to safety_controller
(ActuatorsController, which holds the MRS tracker reference) and no more goals are sent;
restart this script to continue. safety_controller:="" only warns.

The UAV position comes from the MRS estimate (/<uav>/estimation_manager/odom_main), which
also exists on the real UAV; ground_truth:=true uses the simulator ground truth instead.

The goal and threshold radius are shown as a green sphere (visual only, no
collisions) in RViz (/<uav>/random_goto/goal_marker) and in Gazebo (simulation only,
shifted by the median ground truth - estimate offset).
"""

import bisect
import math
import os
import random
import statistics
from collections import deque

import rclpy
import rclpy.duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from mrs_msgs.msg import ControlManagerDiagnostics, ReferenceStamped
from mrs_msgs.srv import ReferenceStampedSrv
from mrs_msgs.srv import String as StringSrv

try:
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.entity_pb2 import Entity
    from gz.msgs10.pose_pb2 import Pose
    from gz.transport13 import Node as GzNode
except ImportError:
    GzNode = None

# static, visual-only model: no collision element, so nothing can hit it
GZ_SPHERE_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <cast_shadows>true</cast_shadows>
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        <material>
          <ambient>0.2 1.0 0.2 1</ambient>
          <diffuse>0.3 1.0 0.3 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
          <emissive>0.1 0.5 0.1 1</emissive>
        </material>
        <transparency>0.0</transparency>
      </visual>
    </link>
  </model>
</sdf>"""

# default box, offsets from the starting position of the UAV: 1 x 1 m square in its plane
X_RANGE = 1.0
Y_RANGE = 1.0
Z_RANGE = 0.0

class RandomGoto(Node):

    def __init__(self):
        super().__init__("random_goto")

        # follow the simulation clock like the rest of the stack (USE_SIM_TIME is exported in session.yaml)
        if os.environ.get("USE_SIM_TIME", "").lower() == "true":
            self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.uav_name = self.declare_parameter("uav_name", "uav1").value

        # relative: the bounds are offsets from the UAV position when this node starts (taken from the first odometry)
        self.relative = self.declare_parameter("relative", True).value
        self.center = None

        # sampling bounds [m], the goals must lie inside the safety area in world_config.yaml
        self.x_bounds = (self.declare_parameter("x_min", -X_RANGE).value, self.declare_parameter("x_max", X_RANGE).value)
        self.y_bounds = (self.declare_parameter("y_min", -Y_RANGE).value, self.declare_parameter("y_max", Y_RANGE).value)
        self.z_bounds = (self.declare_parameter("z_min", -Z_RANGE).value, self.declare_parameter("z_max", Z_RANGE).value)

        self.radius = self.declare_parameter("radius", 0.2).value            # [m] goal reached threshold
        self.timeout = self.declare_parameter("timeout", 10.0).value         # [s] resample if goal not reached
        self.random_heading = self.declare_parameter("random_heading", False).value
        # [m] warn when the UAV is further than this from the centre of the goal box (the start position if relative)
        self.max_distance = self.declare_parameter("max_distance", 4.0).value
        # ... and then switch from RLGoto to this controller ("" = only warn)
        self.safety_controller = self.declare_parameter("safety_controller", "ActuatorsController").value
        self.active_controller = None
        self.safety_triggered = False  # goals stopped
        self.safety_switch = None      # "pending" while a switch request is in flight
        # position used for the start position and the "goal reached" check: by default the MRS estimate
        # (works on the real UAV), ground_truth:=true uses the simulator ground truth instead
        self.ground_truth = self.declare_parameter("ground_truth", False).value
        if self.ground_truth:
            default_odom_topic = f"/{self.uav_name}/hw_api/ground_truth"
            # The Gazebo ground truth is stamped "<uav>/world_origin", but that is the Gazebo world,
            # NOT MRS's GNSS-based world_origin (they differ by a drifting offset).
            # MRS's ground_truth_origin coincides with the Gazebo world, so goals sent there match odom_topic.
            default_frame_id = f"{self.uav_name}/ground_truth_origin"
        else:
            default_odom_topic = f"/{self.uav_name}/estimation_manager/odom_main"
            default_frame_id = ""  # empty = the frame of odom_topic
        self.odom_topic = self.declare_parameter("odom_topic", default_odom_topic).value
        # frame of the goal
        self.frame_id = self.declare_parameter("frame_id", default_frame_id).value

        # visualization of the goal sphere, in Gazebo only in simulation (RUN_TYPE is exported in the sessions)
        self.gz_marker = self.declare_parameter("gz_marker", os.environ.get("RUN_TYPE", "") == "simulation").value
        # the Gazebo ground truth, to place goals given in the estimator frame into the Gazebo world
        self.gz_ground_truth_topic = self.declare_parameter("gz_ground_truth_topic", f"/{self.uav_name}/hw_api/ground_truth").value
        self.gz_world = self.declare_parameter("gz_world", "default").value
        self.gz_model = f"{self.uav_name}_goto_goal"

        # direct: publish the goal straight to RLGoto (like the genesis env), bypassing the MRS tracker;
        # otherwise send it to control_manager/reference and the tracker plans a trajectory to it
        self.direct = self.declare_parameter("direct", True).value

        seed = self.declare_parameter("seed", -1).value
        if seed >= 0:
            random.seed(seed)

        self.odom = None
        self.goal = None
        self.goal_time = None
        self.pending = False
        self.retry_time = None

        self.create_subscription(
            Odometry, self.odom_topic, self.odom_cb, qos_profile_sensor_data)
        self.client = self.create_client(ReferenceStampedSrv, f"/{self.uav_name}/control_manager/reference")
        self.goal_pub = self.create_publisher(ReferenceStamped, f"/{self.uav_name}/control_manager/rl_goto/goal", 10)
        self.marker_pub = self.create_publisher(Marker, f"/{self.uav_name}/random_goto/goal_marker", 1)
        self.create_subscription(
            ControlManagerDiagnostics, f"/{self.uav_name}/control_manager/diagnostics", self.diagnostics_cb, 10)
        self.switch_client = self.create_client(StringSrv, f"/{self.uav_name}/control_manager/switch_controller")
        # republish so RViz picks up the marker even if it starts after this node,
        # and retry the Gazebo sphere if the goal could not be transformed yet
        self.create_timer(1.0, self.refresh_markers)

        self.gz = None
        self.gz_spawned = False
        self.gz_placed = False
        if self.gz_marker:
            if GzNode is None:
                self.get_logger().warn("gz python bindings not found, goal sphere only shown in RViz")
            else:
                self.gz = GzNode()
                if not self.ground_truth:
                    # ground truth - estimate offsets of time-matched samples; their median places the sphere.
                    # (The TF between the estimator and ground truth origins jitters by ~0.5 m in agile flight.)
                    self.est_samples = deque(maxlen=200)
                    self.gz_offsets = deque(maxlen=500)
                    self.create_subscription(
                        Odometry, self.gz_ground_truth_topic, self.gz_ground_truth_cb, qos_profile_sensor_data)

        self.create_timer(0.1, self.loop)
        self.get_logger().info(
            f"{'offsets from the start position' if self.relative else 'bounds'} "
            f"x{self.x_bounds} y{self.y_bounds} z{self.z_bounds}, radius {self.radius} m, "
            f"position from {self.odom_topic}, goals {'direct to RLGoto' if self.direct else 'via the MRS tracker'}")

    def odom_cb(self, msg):
        if self.odom is None:
            self.get_logger().info(f"receiving {self.odom_topic} in frame '{msg.header.frame_id}'")
            if self.relative:
                p = msg.pose.pose.position
                self.center = (p.x, p.y, p.z)
                self.get_logger().info(f"goals around the start position [{p.x:.2f}, {p.y:.2f}, {p.z:.2f}]")
        self.odom = msg
        if self.gz is not None and not self.ground_truth:
            p = msg.pose.pose.position
            self.est_samples.append((stamp_ns(msg), (p.x, p.y, p.z)))

    def gz_ground_truth_cb(self, msg):
        # pair with the estimate of the closest stamp, at most 5 ms apart (the ground truth comes at ~83 Hz)
        if not self.est_samples:
            return
        t = stamp_ns(msg)
        stamps = [s[0] for s in self.est_samples]
        i = bisect.bisect_left(stamps, t)
        j = min((k for k in (i - 1, i) if 0 <= k < len(stamps)), key=lambda k: abs(stamps[k] - t))
        if abs(stamps[j] - t) > 5e6:
            return
        p, e = msg.pose.pose.position, self.est_samples[j][1]
        self.gz_offsets.append((p.x - e[0], p.y - e[1], p.z - e[2]))

    def diagnostics_cb(self, msg):
        self.active_controller = msg.active_controller

    def check_distance(self):
        # safety guard: the goals stay inside the box, so being far from it means the UAV is overshooting
        if self.relative:
            center = self.center
        else:
            center = tuple(0.5 * (b[0] + b[1]) for b in (self.x_bounds, self.y_bounds, self.z_bounds))
        p = self.odom.pose.pose.position
        dist = math.dist((p.x, p.y, p.z), center)
        if dist > self.max_distance:
            self.get_logger().warn(
                f"⚠️  UAV is {dist:.2f} m from the centre of the goal box [{center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}] "
                f"(limit {self.max_distance:.1f} m), pos [{p.x:.2f}, {p.y:.2f}, {p.z:.2f}]", throttle_duration_sec=0.5)
            if self.safety_controller and self.active_controller == "RLGoto":
                self.trigger_safety()

    def trigger_safety(self):
        # stop the goals for good and switch away from RLGoto, retried until the control manager accepts it
        if not self.safety_triggered:
            self.safety_triggered = True
            self.get_logger().error(f"⚠️  RLGoto too far from the goal box: switching to {self.safety_controller}, no more goals")
        if self.safety_switch == "pending":
            return
        if not self.switch_client.service_is_ready():
            self.get_logger().error("⚠️  control_manager/switch_controller not available", throttle_duration_sec=1.0)
            return
        self.safety_switch = "pending"
        self.switch_client.call_async(StringSrv.Request(value=self.safety_controller)).add_done_callback(self.safety_switch_cb)

    def safety_switch_cb(self, future):
        res = future.result()
        if res is not None and res.success:
            self.safety_switch = "done"
            self.get_logger().warn(f"⚠️  switched to {self.safety_controller}: {res.message}")
        else:
            self.safety_switch = None  # retried on the next check while RLGoto is still active
            self.get_logger().error(f"⚠️  switching to {self.safety_controller} failed: {res.message if res else 'no response'}")

    def loop(self):
        if self.odom is None:
            self.get_logger().info(f"waiting for {self.odom_topic}", throttle_duration_sec=5.0)
            return
        self.check_distance()
        if self.safety_triggered:
            self.get_logger().warn("⚠️  goals stopped by the safety guard, restart random_goto.py to continue",
                                   throttle_duration_sec=5.0)
            return
        if self.pending:
            return
        if self.direct and self.goal is not None:
            self.publish_goal()  # republish, RLGoto may start listening later
        if not self.direct and not self.client.service_is_ready():
            self.get_logger().info("waiting for control_manager/reference", throttle_duration_sec=5.0)
            return

        if self.goal is None:
            # back off after a rejected goal (e.g. UAV still on the ground)
            if self.retry_time is None or self.get_clock().now() > self.retry_time:
                self.send_new_goal()
            return

        p = self.odom.pose.pose.position
        dist = math.dist((p.x, p.y, p.z), self.goal)
        elapsed = (self.get_clock().now() - self.goal_time).nanoseconds * 1e-9
        self.get_logger().info(
            f"pos [{p.x:.2f}, {p.y:.2f}, {p.z:.2f}] dist to goal {dist:.2f} m", throttle_duration_sec=2.0)

        if dist < self.radius:
            self.get_logger().info(f"[random_goto] ✅ Reached goal in {elapsed:.1f} s")
            self.send_new_goal()
        elif elapsed > self.timeout:
            self.get_logger().warn(f"[random_goto] ❌ Goal not reached after {self.timeout:.0f} s (dist {dist:.2f} m), resampling")
            self.send_new_goal()

    def send_new_goal(self):
        goal = tuple(random.uniform(*b) for b in (self.x_bounds, self.y_bounds, self.z_bounds))
        if self.relative:
            goal = tuple(c + g for c, g in zip(self.center, goal))
        p = self.odom.pose.pose.position
        # heading of the current odometry would need a quaternion->yaw conversion; 0 keeps it simple
        heading = random.uniform(-math.pi, math.pi) if self.random_heading else 0.0

        req = ReferenceStampedSrv.Request()
        req.header.frame_id = self.frame_id or self.odom.header.frame_id
        req.header.stamp = self.get_clock().now().to_msg()
        req.reference.position.x, req.reference.position.y, req.reference.position.z = goal
        req.reference.heading = heading

        self.get_logger().info(
            f"new goal [{goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f}] hdg {heading:.2f} in '{req.header.frame_id}' "
            f"({math.dist((p.x, p.y, p.z), goal):.2f} m away)")

        if self.direct:
            self.goal_msg = ReferenceStamped(header=req.header, reference=req.reference)
            self.goal = goal
            self.goal_frame = req.header.frame_id
            self.goal_time = self.get_clock().now()
            self.publish_goal()
            self.gz_placed = False
            self.publish_marker()
            self.move_gz_marker()
            return

        self.pending = True
        self.client.call_async(req).add_done_callback(lambda f: self.response_cb(f, goal))

    def publish_goal(self):
        self.goal_msg.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(self.goal_msg)

    def response_cb(self, future, goal):
        self.pending = False
        res = future.result()
        if res is not None and res.success:
            self.goal = goal
            self.goal_frame = self.frame_id or self.odom.header.frame_id
            self.goal_time = self.get_clock().now()
            self.gz_placed = False
            self.publish_marker()
            self.move_gz_marker()
        else:
            # e.g. outside safety area or UAV not flying yet; loop() retries with a fresh sample
            self.get_logger().warn(f"goal rejected: {res.message if res else 'no response'}")
            self.goal = None
            self.retry_time = self.get_clock().now() + rclpy.duration.Duration(seconds=2.0)

    def publish_marker(self):
        if self.goal is None:
            return
        m = Marker()
        m.header.frame_id = self.goal_frame
        # stamp 0 = RViz uses the latest available transform
        m.ns = "random_goto"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.goal
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 2.0 * self.radius
        m.color.g = 1.0
        m.color.a = 0.4
        self.marker_pub.publish(m)

    def goal_in_gz_frame(self):
        # the goal in the Gazebo world, None until the estimate - ground truth offset is known
        if self.ground_truth:
            return self.goal
        if len(self.gz_offsets) < 10:
            self.get_logger().info("goal sphere waiting for ground truth to place it in gazebo", throttle_duration_sec=10.0)
            return None
        return tuple(g + statistics.median(o[i] for o in self.gz_offsets) for i, g in enumerate(self.goal))

    def refresh_markers(self):
        self.publish_marker()
        if self.goal is not None and not self.gz_placed:
            self.move_gz_marker()

    def move_gz_marker(self):
        if self.gz is None:
            return
        goal = self.goal_in_gz_frame()
        if goal is None:
            return
        timeout_ms = 500
        if not self.gz_spawned:
            req = EntityFactory()
            req.sdf = GZ_SPHERE_SDF.format(name=self.gz_model, radius=self.radius)
            req.pose.position.x, req.pose.position.y, req.pose.position.z = goal
            ok, rep = self.gz.request(f"/world/{self.gz_world}/create", req, EntityFactory, Boolean, timeout_ms)
            if ok and rep.data:
                self.gz_spawned = True
                self.gz_placed = True
                return
            # already exists from a previous run, or gazebo unreachable: fall through to set_pose
            self.gz_spawned = True
        req = Pose()
        req.name = self.gz_model
        req.position.x, req.position.y, req.position.z = goal
        req.orientation.w = 1.0
        ok, rep = self.gz.request(f"/world/{self.gz_world}/set_pose", req, Pose, Boolean, timeout_ms)
        self.gz_placed = ok and rep.data
        if not self.gz_placed:
            self.get_logger().warn(
                f"could not place goal sphere in gazebo world '{self.gz_world}'", throttle_duration_sec=10.0)

    def remove_gz_marker(self):
        if self.gz is None or not self.gz_spawned:
            return
        req = Entity()
        req.name = self.gz_model
        req.type = Entity.MODEL
        self.gz.request(f"/world/{self.gz_world}/remove", req, Entity, Boolean, 500)


def stamp_ns(msg):
    return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec


def main():
    rclpy.init()
    node = RandomGoto()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.remove_gz_marker()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
