#!/bin/bash

# PX4 refuses to arm until its own EKF has initialised. With the ground_truth
# estimator MRS reports ready long before PX4 does, so a single arming attempt
# gets denied. Switching to OFFBOARD after a failed arm then deadlocks: PX4 won't
# arm in OFFBOARD without a setpoint stream, and automatic_start only turns the
# setpoint stream on once the UAV is armed. So retry arming until PX4 accepts,
# and only then switch to offboard.

for attempt in $(seq 1 60); do

  echo "arming (attempt $attempt)"

  if ros2 service call /uav1/hw_api/arming std_srvs/srv/SetBool '{"data": true}' | grep -q "success=True"; then

    sleep 1.0

    echo "toggling offboard"

    ros2 service call /uav1/hw_api/offboard std_srvs/srv/Trigger '{}'

    exit 0
  fi

  sleep 2.0
done

echo "arming failed after 60 attempts, not switching to offboard"
exit 1
