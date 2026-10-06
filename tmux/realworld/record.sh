#!/bin/bash
# Copy of mrs_uav_deployment just_flying/record.sh, used by the real flight (realworld/tmux.sh) and HITL
# (hitl/session_drone.yaml) so both are recorded the same way. Run it from the session folder (its ./config is
# snapshotted). Besides the bag (all topics, incl. control_manager/rl_goto/{observation,action,goal_used,drone_params}),
# it stores run_info/ next to it: what was deployed and how it was configured, to compare simulation and real flights.
#
# usage: record.sh [target folder], default ~/bag_files/latest (created by realworld/tmux.sh)

target_path="${1:-$HOME/bag_files/latest}"

# By default, we record everything.
# Except for this list of EXCLUDED topics:
exclude=(

# IN GENERAL, DON'T RECORD CAMERAS
#
# If you want to record cameras, create a copy of this script
# and place it at your tmux session.
#
# Please, seek an advice of a senior researcher of MRS about
# what can be recorded. Recording too much data can lead to
# ROS communication hiccups, which can lead to eland, failsafe
# or just a CRASH.

# this is how you exclude a topic with the use of wildcards
# '.*control_manager.*'

# this is how you exclude the image_raw namespace WITHOUT excluding its subnamespaces
# '.*camera/image_raw$'
)

if [ ! -e "$target_path" ]; then
  mkdir -p "$target_path"
fi

# | ------------------- snapshot of the run ------------------- |
run_info="$target_path/run_info"
mkdir -p "$run_info"
ws="$HOME/adam_ws"
cp -rL ./config "$run_info/session_config"
# the HITL session passes its platform config through PLATFORM_CONFIG instead of ./config
[ -n "$PLATFORM_CONFIG" ] && cp -L "$PLATFORM_CONFIG" "$run_info/" 2>/dev/null
cp -L ../position_control/config/random_goto.yaml "$run_info/" 2>/dev/null
cp -L "$ws/src/rl_goto_controller/config/rl_goto.yaml" "$run_info/" 2>/dev/null
cp -L "$ws/deployed_version.txt" "$run_info/" 2>/dev/null
md5sum "$ws"/src/rl_goto_controller/policies/*.rlp > "$run_info/policies.md5" 2>/dev/null
{
  echo "date: $(date --iso-8601=seconds)"
  echo "host: $(hostname)"
  echo "session: $(pwd)"
  env | grep -E '^(UAV_|RUN_TYPE|WORLD_NAME|RMW_|ROS_|CYCLONEDDS|PX4|PIXGARM|OLD_PX4|USE_SIM_TIME)' | sort
} > "$run_info/environment.txt"
dpkg-query -W -f '${Package} ${Version}\n' 'ros-jazzy-mrs-*' > "$run_info/mrs_packages.txt" 2>/dev/null
echo "run info stored in $run_info"

# file's header
filename=$(mktemp)

echo -n "cd $target_path; ros2 bag record -a" >> "$filename"

# if there is anything to exclude
if [ "${#exclude[*]}" -gt 0 ]; then

  echo -n " --exclude-regex " >> "$filename"

  # list all the strings and separate the with |
  for ((i=0; i < ${#exclude[*]}; i++));
  do

    if [ "$i" -eq "0" ]; then
      echo -n "\"(" >> "$filename"
    fi

    echo -n "${exclude[$i]}" >> "$filename"

    if [ "$i" -lt "$( expr ${#exclude[*]} - 1)" ]; then
      echo -n "|" >> "$filename"
    else
      echo -n ")\"" >> "$filename"
    fi

  done

fi

cat $filename

eval $(cat "$filename")
