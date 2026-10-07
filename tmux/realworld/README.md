# Real-world flight session (x500, RLGoto / ActuatorsController)

`tmux.sh` is based on `mrs_uav_deployment/tmux/just_flying/tmux.sh`: every window is logged to
`~/bag_files/rl_goto/<n>_<date>/tmux/`, a rosbag is recorded once the UAV is in offboard.
Differences: only `/opt/ros/jazzy` + `~/adam_ws` are sourced, CycloneDDS kept on the drone
(`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, no zenoh), our platform/custom configs, controller switch
windows, and plain `;` instead of the literal `^M` of the MRS original (bash does not execute after it).

## New drone

1. `~/.ssh/config` on the laptop:
   ```
   Host uavN
       HostName 192.168.69.<100+N>
       User mrs
       IdentityFile ~/.ssh/id_ed25519
       ServerAliveInterval 30
   ```
   then `ssh-copy-id uavN`.
2. On the drone, `~/.bashrc` must have `UAV_NAME`, `UAV_TYPE` (with a `config/platform_$UAV_TYPE.yaml`), `UAV_MASS`, `WORLD_NAME`, `RUN_TYPE=realworld`
   (`tmux.sh` refuses to start otherwise). Put the UAV_NAME into `config/network_config.yaml`.
3. `utils/sync_to_drone.sh uavN` from the laptop (dry run, asks before copying), then build as it prints.

## Before the first flight

- Flight controller: our PX4 fork (`SET_ACTUATOR_CONTROL_TARGET` handler) flashed, params backed up.
- PX4: `EKF2_PREDICT_US 4000`, `IMU_INTEG_RATE >= 250`; `/fs/microsd/etc/extras.txt` streams
  `LOCAL_POSITION_NED` and `ODOMETRY` at `-r 500` (not 250: the limiter drops the jittery EKF output to ~205 Hz)
  on the MAVROS link, whose `mavlink start` needs `-r 180000`; uav58's is `config/px4_extras_uav58.txt`.
  With a GPS safety switch, `CBRK_IO_SAFETY 22027` or the motor test is refused.
- `config/platform_x500.yaml`: replace the ⚠️ simulation `model_params` with the flown drone's values;
  check `motor_params` against its motors.
- `rl_goto.yaml` `drone:` block: the flown drone's measured parameters.
- Props off: `/dev/pixhawk` exists, `hw_api/odometry` at 250 Hz (`.debug/odom_rate_probe.py`, not `ros2 topic hz`),
  passthrough reaches `READY_FOR_FLIGHT_STATE`, motors 0..3 of `HwApiActuatorCmd` spin in the
  `allocation_matrix` order.
- Rehearse the same safety limits in HITL (`config/custom_config.yaml` layers).

## Recording

`record.sh` (Record window, starts automatically once the HwApi is up) records all topics into `~/bag_files/rl_goto/<n>_<date>/` with
`run_info/`: session configs, `rl_goto.yaml`, `random_goto.yaml`, policy md5, the drone's environment, MRS package
versions and `deployed_version.txt` (git commit + uncommitted files, written by `utils/sync_to_drone.sh`).
HITL records the same way into `~/bag_files/hitl/<date>/` on the drone.
RLGoto publishes what the policy saw and did, stamped with the UAV state it used:
`control_manager/rl_goto/{observation,action,goal_used}` at the policy rate, `drone_params` latched.
Also set the PX4 ULog to log at high rate (`SDLOG_PROFILE`) for the motor outputs / ESC data.

## Flight

```
ssh uavN
~/adam_ws/tmux/realworld/tmux.sh
```

1. Safety pilot: arm + offboard on the RC, `AutoStart` takes off into MpcController (2.5 m).
2. `Actuators` window: <enter>, hover. `Mpc` window: <enter> to go back from ActuatorsController.
3. `RLGoto` window: <enter>, it holds the position where it was switched on. To go back: `Actuators` first, then `Mpc`;
   switching RLGoto straight to MpcController loses control.
4. `RandomGoto` window: <enter>, goals from `rl_goto_controller/config/random_goto.yaml` (seed 0, 2 x 2 m square around that position,
   the same sequence as in HITL);
   beyond 3 m it switches to ActuatorsController and stops the goals.
5. Beyond 3.5 m position error, large tilt or tilt error, MRS hands control to the RC (`rc_emergency_handoff`).
