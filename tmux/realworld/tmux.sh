#!/usr/bin/env bash
# Real-world flight session for the RLGoto / ActuatorsController experiments on an MRS x500.
# Based on mrs_uav_deployment/tmux/just_flying/tmux.sh (window logging, rosbag, same start-up logic);
# see README.md in this folder for the checklist before using it.
#
# Expects the workspace at ~/adam_ws (src/ + this tmux/ folder), built against /opt/ros/jazzy only.
if [ "$(id -u)" == "0" ]; then
  exec sudo -u mrs "$0" "$@"
fi

source $HOME/.bashrc

# refuse to start on a drone that is not set up for this session
for var in UAV_NAME UAV_TYPE UAV_MASS WORLD_NAME RUN_TYPE; do
  if [ -z "${!var}" ]; then
    echo "$var is not set in ~/.bashrc, not starting"
    exit 1
  fi
done
if [ "$RUN_TYPE" != "realworld" ]; then
  echo "RUN_TYPE is '$RUN_TYPE', this session is for RUN_TYPE=realworld, not starting"
  exit 1
fi
# the platform config of this UAV_TYPE (model_params/motor_params of the flown drone), e.g. config/platform_m430.yaml
PLATFORM_CONFIG=./config/platform_$UAV_TYPE.yaml
if [ ! -f "$(dirname $(readlink -f $0))/$PLATFORM_CONFIG" ]; then
  echo "UAV_TYPE is '$UAV_TYPE' but there is no $PLATFORM_CONFIG, not starting"
  exit 1
fi
if [ ! -f "$HOME/adam_ws/install/setup.bash" ]; then
  echo "~/adam_ws is not built, not starting"
  exit 1
fi
if [ ! -e /dev/pixhawk ]; then
  echo "WARNING: /dev/pixhawk does not exist (flight controller not powered?), the HwApi will wait for it"
fi

# location for storing the bag files
# * do not change unless you know what you are doing
MAIN_DIR="$HOME/bag_files"

# the project name
# * is used to define folder name in ~/$MAIN_DIR
PROJECT_NAME=rl_goto

# the name of the TMUX session
# * can be used for attaching as 'tmux a -t <session name>'
SESSION_NAME=mav

# following commands will be executed first in each window
# * do NOT put ; at the end
# * only jazzy + adam_ws (not the overlays of other people from ~/.bashrc)
# * CycloneDDS, not zenoh (zenoh delivered the 250 Hz odometry in 100-200 ms clumps), kept on this computer
#   so the 250 Hz topics do not go over the wifi; monitor the UAV through ssh + this tmux session
pre_input="unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH; source /opt/ros/jazzy/setup.bash; source $HOME/adam_ws/install/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"

# define commands
# 'name' 'command'
# * DO NOT PUT SPACES IN THE NAMES
# * "new line" after the command    => the command will be called after start
# * NO "new line" after the command => the command will wait for user's <enter>
#
# Flight procedure: the safety pilot takes off (arm + offboard on the RC, AutoStart takes off into MpcController),
# then press <enter> in the windows Actuators -> RLGoto -> RandomGoto. To go back, step down the same chain:
# RLGoto -> Actuators -> Mpc (switching RLGoto straight to MpcController loses control).
input=(
  # records automatically from when the HwApi is up (bag + run_info/ into ~/bag_files/latest), also when offboard
  # never comes: on 2026-10-07 nothing was recorded because <enter> was never pressed here
  'Record' 'waitForHwApi; ./record.sh
'
  'HwApi' 'ros2 launch mrs_uav_px4_api api.launch.py
'
  # the transform_manager shuts the core down without a fcu -> garmin tf (mrs_uav_deployment x500_example.launch);
  # uav58 has no rangefinder, so it only has to exist: pointing down below the fcu
  # 'StaticTf' 'ros2 run tf2_ros static_transform_publisher --z -0.05 --pitch 1.5708 --frame-id $UAV_NAME/fcu --child-frame-id $UAV_NAME/garmin --ros-args -r __ns:=/$UAV_NAME -r __node:=fcu_to_garmin
# '
  'Status' 'ros2 run mrs_uav_status status.sh
'
  'Core' 'ros2 launch mrs_uav_core core.launch.py platform_config:='"$PLATFORM_CONFIG"' world_config:=`ros2 pkg prefix mrs_uav_deployment --share`/config/worlds/world_$WORLD_NAME.yaml custom_config:=./config/custom_config.yaml network_config:=./config/network_config.yaml
'
  'AutoStart' 'ros2 launch mrs_uav_autostart automatic_start.launch.py
'
  'Mpc' 'ros2 service call /$UAV_NAME/control_manager/switch_controller mrs_msgs/srv/String "{value: MpcController}"'
  'Actuators' 'ros2 service call /$UAV_NAME/control_manager/switch_controller mrs_msgs/srv/String "{value: ActuatorsController}"'
  'RLGoto' 'ros2 service call /$UAV_NAME/control_manager/switch_controller mrs_msgs/srv/String "{value: RLGoto}"'
  # same settings and seed as in HITL (rl_goto_controller config/random_goto.yaml): 2 x 2 m box around where
  # RLGoto hovers, beyond 3 m it switches to ActuatorsController and stops the goals
  'RandomGoto' 'ros2 run rl_goto_controller random_goto.py --ros-args --params-file `ros2 pkg prefix --share rl_goto_controller`/config/random_goto.yaml -p uav_name:=$UAV_NAME'
  # 'EstimDiag' 'waitForCore; ros2 topic echo /'"$UAV_NAME"'/estimation_manager/diagnostics --flow-style'

  # 'kernel_log' 'tail -f /var/log/kern.log -n 100'
)

# the name of the window to focus after start
init_window="Status"

# automatically attach to the new session?
# {true, false}, default true
attach=true

###########################
### DO NOT MODIFY BELOW ###
###########################

export TMUX_BIN="tmux -L mrs -f /etc/ctu-mrs/tmux.conf"

# find the session
FOUND=$( $TMUX_BIN ls | grep $SESSION_NAME )

if [ $? == "0" ]; then
  echo "The session already exists"
  $TMUX_BIN -2 attach-session -t $SESSION_NAME
  exit
fi

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`

TMUX= $TMUX_BIN new-session -s "$SESSION_NAME" -d
echo "Starting new session."

# get the iterator
ITERATOR_FILE="$MAIN_DIR/$PROJECT_NAME"/iterator.txt
if [ -e "$ITERATOR_FILE" ]
then
  ITERATOR=`cat "$ITERATOR_FILE"`
  ITERATOR=$(($ITERATOR+1))
else
  echo "iterator.txt does not exist, creating it"
  mkdir -p "$MAIN_DIR/$PROJECT_NAME"
  touch "$ITERATOR_FILE"
  ITERATOR="1"
fi
echo "$ITERATOR" > "$ITERATOR_FILE"

# create file for logging terminals' output
LOG_DIR="$MAIN_DIR/$PROJECT_NAME/"
SUFFIX=$(date +"%Y_%m_%d_%H_%M_%S")
SUBLOG_DIR="$LOG_DIR/"$ITERATOR"_"$SUFFIX""
TMUX_DIR="$SUBLOG_DIR/tmux"
mkdir -p "$SUBLOG_DIR"
mkdir -p "$TMUX_DIR"

# link the "latest" folder to the recently created one
rm "$LOG_DIR/latest" > /dev/null 2>&1
rm "$MAIN_DIR/latest" > /dev/null 2>&1
ln -sf "$SUBLOG_DIR" "$LOG_DIR/latest"
ln -sf "$SUBLOG_DIR" "$MAIN_DIR/latest"

# create arrays of names and commands
for ((i=0; i < ${#input[*]}; i++));
do
  ((i%2==0)) && names[$i/2]="${input[$i]}"
  ((i%2==1)) && cmds[$i/2]="${input[$i]}"
done

# run tmux windows
for ((i=0; i < ${#names[*]}; i++));
do
  $TMUX_BIN new-window -t $SESSION_NAME:$(($i+1)) -n "${names[$i]}"
done

sleep 3

# start loggers
for ((i=0; i < ${#names[*]}; i++));
do
  $TMUX_BIN pipe-pane -t $SESSION_NAME:$(($i+1)) -o "ts | cat >> $TMUX_DIR/$(($i+1))_${names[$i]}.log"
done

# send commands
for ((i=0; i < ${#cmds[*]}; i++));
do
  # (the MRS original separates these with a literal "^M", which bash does not execute: plain ';' instead)
  $TMUX_BIN send-keys -t $SESSION_NAME:$(($i+1)) "cd $SCRIPTPATH; ${pre_input}; ${cmds[$i]}"
done

# identify the index of the init window
init_index=0
for ((i=0; i < ((${#names[*]})); i++));
do
  if [ ${names[$i]} == "$init_window" ]; then
    init_index=$(expr $i + 1)
  fi
done

$TMUX_BIN select-window -t $SESSION_NAME:$init_index

if $attach; then

  if [ -z ${TMUX} ];
  then
    $TMUX_BIN -2 attach-session -t $SESSION_NAME
  else
    tmux detach-client -E "tmux -L mrs a -t $SESSION_NAME"
  fi
else
  echo "The session was started"
  echo "You can later attach by calling:"
  echo "  tmux -L mrs a -t $SESSION_NAME"
fi
