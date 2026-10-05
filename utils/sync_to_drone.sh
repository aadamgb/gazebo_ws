#!/bin/bash
# Copies what the drone needs from this workspace into ~/adam_ws on the drone:
# the controller packages, the patched MRS packages, the tmux sessions (hitl, realworld, position_control)
# and the x500 platform config of the HITL session. Shows a dry run first and asks before copying.
# Never deletes anything on the drone.
#
# usage: utils/sync_to_drone.sh <ssh host, e.g. uav1>

set -e

if [ -z "$1" ]; then
  echo "usage: $0 <ssh host, e.g. uav1>"
  exit 1
fi
HOST=$1

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# the workspace root, one level above utils/
WS=$(dirname $(dirname $SCRIPT))
cd "$WS"

PACKAGES=(src/rl_goto_controller src/srt_controller src/mrs_uav_px4_api src/mrs_uav_state_estimators)
SESSIONS=(tmux/hitl tmux/realworld tmux/position_control)
PLATFORM=src/alien_gazebo_resources/config/mrs_uav_system/x500.yaml

sync() {
  # $1: extra rsync flags
  rsync $1 --mkpath --exclude .git --exclude __pycache__ "${PACKAGES[@]}" "$HOST:adam_ws/src/"
  rsync $1 --mkpath --exclude __pycache__ "${SESSIONS[@]}" "$HOST:adam_ws/tmux/"
  rsync $1 --mkpath "$PLATFORM" "$HOST:adam_ws/platform/"
}

echo "branch: $(git branch --show-current)"
UNCOMMITTED=$(git status --porcelain -- "${PACKAGES[@]}" "${SESSIONS[@]}" | grep -v __pycache__ || true)
if [ -n "$UNCOMMITTED" ]; then
  echo "uncommitted changes that will be copied too:"
  echo "$UNCOMMITTED"
fi

echo
echo "dry run, files that differ on $HOST (compared by content):"
CHANGES=$(sync "-ainc" | grep -v '^\.d' || true)
if [ -z "$CHANGES" ]; then
  echo "  none, $HOST:~/adam_ws is up to date"
  exit 0
fi
echo "$CHANGES"

echo
read -p "copy these to $HOST:~/adam_ws? [y/N] " answer
if [ "$answer" != "y" ]; then
  echo "nothing copied"
  exit 0
fi

sync "-aic" > /dev/null
echo "copied"

if echo "$CHANGES" | grep -qE '^[<c]f.* (rl_goto_controller|srt_controller|mrs_uav_px4_api|mrs_uav_state_estimators)/'; then
  echo
  echo "packages changed, rebuild on $HOST (only against /opt/ros/jazzy, not the other overlays in its ~/.bashrc):"
  echo "  ssh $HOST"
  echo "  env -i HOME=\$HOME PATH=/usr/bin:/bin bash --norc -c 'source /opt/ros/jazzy/setup.bash && cd ~/adam_ws && colcon build --symlink-install --packages-select rl_goto_controller srt_controller mrs_uav_px4_api mrs_uav_state_estimators --allow-overriding mrs_uav_px4_api mrs_uav_state_estimators --cmake-args -DCMAKE_BUILD_TYPE=Release'"
fi
