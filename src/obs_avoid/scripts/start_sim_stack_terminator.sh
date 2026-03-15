#!/usr/bin/env bash

set -euo pipefail

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[error] missing command: $1" >&2
    exit 1
  fi
}

SESSION="${SESSION:-obs_avoid_tilted_stack}"
USE_TMUX="${USE_TMUX:-0}"
RECREATE_SESSION="${RECREATE_SESSION:-1}"
KILL_BEFORE_LAUNCH="${KILL_BEFORE_LAUNCH:-1}"
ATTACH_SESSION="${ATTACH_SESSION:-1}"
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS_DEFAULT="$(cd "${PKG_DIR}/../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
QGC_DIR="${QGC_DIR:-${HOME}/Downloads}"
QGC_APP="${QGC_APP:-${QGC_DIR}/QGroundControl-x86_64.AppImage}"
WORLD_NAME="${WORLD_NAME:-walls}"
PX4_TARGET="${PX4_TARGET:-gz_x500_lidar_2d_tilted}"
MODEL_NAME="${MODEL_NAME:-x500_lidar_2d_tilted_0}"
FCU_URL="${FCU_URL:-udp://:14540@127.0.0.1:14580}"
MAVROS_LAUNCH_FILE="${MAVROS_LAUNCH_FILE:-px4.launch}"
MAVROS_RESPAWN="${MAVROS_RESPAWN:-true}"
OBS_AVOID_DELAY_SEC="${OBS_AVOID_DELAY_SEC:-30}"
ENABLE_MISSION_OBS_AVOID="${ENABLE_MISSION_OBS_AVOID:-0}"
MISSION_OBS_AVOID_LAUNCH_FILE="${MISSION_OBS_AVOID_LAUNCH_FILE:-mission_obs_avoid_dwa.launch.py}"
DWA_ROS_ARGS="${DWA_ROS_ARGS:-}"

TOPIC_VERT="/world/${WORLD_NAME}/model/${MODEL_NAME}/model/lidar_vert/link/link/sensor/lidar_2d_v2/scan"
TOPIC_HORIZ="/world/${WORLD_NAME}/model/${MODEL_NAME}/model/lidar_horiz/link/link/sensor/lidar_2d_v2/scan"

require_cmd ros2

if [[ "${ENABLE_MISSION_OBS_AVOID}" == "1" ]]; then
  CMD_VEL_REMAP_ARG="-r /mavros/setpoint_velocity/cmd_vel:=/offboard_stack/cmd_vel"
  USER_CTRL_RUN_CMD="ros2 run obs_avoid user_ctrl --ros-args -p use_sim_time:=true ${CMD_VEL_REMAP_ARG}"
  SPIRAL_RUN_CMD="ros2 run obs_avoid spiral_mapping_mode --ros-args -p use_sim_time:=true ${CMD_VEL_REMAP_ARG}"
  MISSION_OBS_AVOID_CMD="source '${ROS_SETUP}' && if ! ros2 pkg prefix mission_obs_avoid >/dev/null 2>&1; then echo '[error] mission_obs_avoid package not found in overlay'; echo '[hint] COLCON_LOG_PATH=/tmp/colcon_log_mission colcon build --base-paths ${ROS_WS}/src/obs_avoid/mission_obs_avoid --packages-select mission_obs_avoid'; exec bash; fi && ros2 launch mission_obs_avoid ${MISSION_OBS_AVOID_LAUNCH_FILE} use_sim_time:=true"
else
  USER_CTRL_RUN_CMD="ros2 run obs_avoid user_ctrl --ros-args -p use_sim_time:=true"
  SPIRAL_RUN_CMD="ros2 run obs_avoid spiral_mapping_mode --ros-args -p use_sim_time:=true"
  MISSION_OBS_AVOID_CMD="echo '[info] mission_obs_avoid disabled (set ENABLE_MISSION_OBS_AVOID=1)'; exec bash"
fi

kill_if_running() {
  local pattern="$1"
  pgrep -af "${pattern}" >/dev/null 2>&1 || return 0
  pkill -f "${pattern}" >/dev/null 2>&1 || true
}

kill_name_if_running() {
  local name="$1"
  command -v killall >/dev/null 2>&1 || return 0
  killall -9 "${name}" >/dev/null 2>&1 || true
}

kill_existing_stack() {
  kill_name_if_running "gz"
  kill_name_if_running "gz-sim"
  kill_name_if_running "ign"
  kill_name_if_running "gazebo"
  kill_name_if_running "px4"
  kill_if_running "gz"
  kill_if_running "ign"
  kill_if_running "gazebo"
  kill_if_running "px4"

  kill_if_running "${QGC_APP}"
  kill_if_running "make px4_sitl ${PX4_TARGET}"
  kill_if_running "gz_x500_lidar_2d_tilted"
  kill_if_running "ros2 launch mavros ${MAVROS_LAUNCH_FILE}"
  kill_if_running "/clock@rosgraph_msgs/msg/Clock"
  kill_if_running "/world/${WORLD_NAME}/model/${MODEL_NAME}/model/lidar_vert"
  kill_if_running "/world/${WORLD_NAME}/model/${MODEL_NAME}/model/lidar_horiz"
  kill_if_running "ros2 run vertical_lidar_mapper scan_frame_override_node"
  kill_if_running "ros2 run vertical_lidar_mapper odom_to_tf_bridge_node"
  kill_if_running "ros2 run odom_flatten px4_odom_flatten_node"
  kill_if_running "ros2 run obs_avoid user_ctrl"
  kill_if_running "ros2 run obs_avoid spiral_mapping_mode"
  kill_if_running "ros2 run obs_avoid local_planner_mode_a"
  kill_if_running "ros2 launch mission_obs_avoid mission_obs_avoid.launch.py"
  kill_if_running "ros2 launch mission_obs_avoid mission_obs_avoid_dwa.launch.py"
  kill_if_running "ros2 run mission_obs_avoid mission_interceptor_node"
  kill_if_running "ros2 run mission_obs_avoid cmd_vel_arbiter_node"
  kill_if_running "mission_interceptor"
  kill_if_running "cmd_vel_arbiter"
  kill_if_running "tf2_ros static_transform_publisher.*lidar_vert_link"
  kill_if_running "tf2_ros static_transform_publisher.*lidar_horiz_link"
  kill_if_running "rviz2"
}

json_escape() {
  printf '%s' "$1" | sed -e 's/\\/\\\\/g' -e 's/"/\\"/g'
}

write_terminator_layout_json() {
  local out="$1"

  local cmd_px4
  local cmd_qgc
  local cmd_mavros
  local cmd_bridge_clock
  local cmd_bridge_vert
  local cmd_bridge_horiz
  local cmd_scan_override
  local cmd_tf_pair
  local cmd_odom_bridge
  local cmd_mavros_params
  local cmd_user_mode
  local cmd_mission_obs_avoid
  local cmd_dwa
  local cmd_rviz

  cmd_px4="cd '${PX4_DIR}' && PX4_GZ_WORLD='${WORLD_NAME}' make px4_sitl '${PX4_TARGET}'"
  cmd_qgc="cd '${QGC_DIR}' && if [[ -x '${QGC_APP}' ]]; then '${QGC_APP}'; else echo '[warn] QGroundControl AppImage not executable: ${QGC_APP}'; fi"
  cmd_mavros="source '${ROS_SETUP}' && echo '[wait] waiting for mavros package ...' && until ros2 pkg prefix mavros >/dev/null 2>&1; do sleep 1; done && while true; do ros2 launch mavros '${MAVROS_LAUNCH_FILE}' fcu_url:='${FCU_URL}' respawn_mavros:='${MAVROS_RESPAWN}' use_sim_time:=true; rc=\$?; if [[ \$rc -eq 130 ]]; then echo '[info] mavros stopped by user'; break; fi; echo \"[warn] mavros exited (rc=\$rc), retrying in 2s\"; sleep 2; done"
  cmd_bridge_clock="source '${ROS_SETUP}' && ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'"
  cmd_bridge_vert="source '${ROS_SETUP}' && ros2 run ros_gz_bridge parameter_bridge '${TOPIC_VERT}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' --ros-args -p use_sim_time:=true"
  cmd_bridge_horiz="source '${ROS_SETUP}' && ros2 run ros_gz_bridge parameter_bridge '${TOPIC_HORIZ}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' --ros-args -p use_sim_time:=true"
  cmd_scan_override="source '${ROS_SETUP}' && ros2 run vertical_lidar_mapper scan_frame_override_node --ros-args -p use_sim_time:=true"
  cmd_tf_pair="source '${ROS_SETUP}' && ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch -1.570796 --roll 0 --frame-id base_link --child-frame-id lidar_vert_link --ros-args -p use_sim_time:=true & ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id base_link --child-frame-id lidar_horiz_link --ros-args -p use_sim_time:=true"
  cmd_odom_bridge="source '${ROS_SETUP}' && echo '[wait] waiting for MAVROS nodes ...' && until ros2 node list 2>/dev/null | grep -q '^/mavros'; do sleep 1; done && if ! ros2 topic list 2>/dev/null | grep -q '^/mavros/local_position/odom$'; then echo '[warn] /mavros/local_position/odom not listed yet; px4_odom_flatten_node will wait for messages.'; fi && echo '[run] starting px4_odom_flatten_node first (odom->base_link)' && ros2 run odom_flatten px4_odom_flatten_node --ros-args -p use_sim_time:=true -p odom_topic:=/mavros/local_position/odom -p parent_frame:=odom -p child_frame:=base_link"
  cmd_mavros_params="source '${ROS_SETUP}' && echo '[wait] waiting for px4_odom_flatten_node ...' && until ros2 node list 2>/dev/null | grep -q '^/px4_odom_flatten_node$'; do sleep 1; done && echo '[run] applying MAVROS use_sim_time params ...' && for n in \$(ros2 node list | grep '^/mavros'); do ros2 param set \"\$n\" use_sim_time true 2>/dev/null || true; done && echo '[done] mavros params applied'; exec bash"
  cmd_user_mode="source '${ROS_SETUP}' && echo '[ready] mode pane is blank. choose one:' && echo '  ${USER_CTRL_RUN_CMD}' && echo '  ${SPIRAL_RUN_CMD}' && exec bash"
  cmd_mission_obs_avoid="${MISSION_OBS_AVOID_CMD}"
  cmd_dwa="source '${ROS_SETUP}' && echo '[wait] delaying ${OBS_AVOID_DELAY_SEC}s before obs_avoid startup ...' && sleep ${OBS_AVOID_DELAY_SEC} && ros2 run obs_avoid local_planner_mode_a --ros-args -p use_sim_time:=true ${DWA_ROS_ARGS}"
  cmd_rviz="source '${ROS_SETUP}' && rviz2 --ros-args -p use_sim_time:=true"

  cat > "${out}" <<EOF
{
  "profile": {
    "exit_action": "hold"
  },
  "layout": {
    "vertical": true,
    "Main": [
      {
        "children": [
          { "command": "$(json_escape "${cmd_px4}")" },
          { "command": "$(json_escape "${cmd_qgc}")" },
          { "command": "$(json_escape "${cmd_mavros}")" },
          { "command": "$(json_escape "${cmd_scan_override}")" },
          { "command": "$(json_escape "${cmd_user_mode}")" },
          { "command": "$(json_escape "${cmd_mission_obs_avoid}")" }
        ]
      },
      {
        "children": [
          { "command": "$(json_escape "${cmd_dwa}")" },
          { "command": "$(json_escape "${cmd_bridge_clock}")" },
          { "command": "$(json_escape "${cmd_bridge_vert}")" },
          { "command": "$(json_escape "${cmd_bridge_horiz}")" },
          { "command": "$(json_escape "${cmd_tf_pair}")" },
          { "command": "$(json_escape "${cmd_odom_bridge}")" },
          { "command": "$(json_escape "${cmd_mavros_params}")" },
          { "command": "$(json_escape "${cmd_rviz}")" }
        ]
      }
    ]
  }
}
EOF
}

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[error] ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -d "${PX4_DIR}" ]]; then
  echo "[error] PX4 directory not found: ${PX4_DIR}" >&2
  exit 1
fi

if [[ "${KILL_BEFORE_LAUNCH}" == "1" ]]; then
  kill_existing_stack
fi

if [[ "${USE_TMUX}" != "1" ]] || ! command -v tmux >/dev/null 2>&1; then
  if ! command -v terminator >/dev/null 2>&1; then
    echo "[error] missing command: terminator" >&2
    exit 1
  fi

  TERMINATOR_LAYOUT_JSON="${TERMINATOR_LAYOUT_JSON:-/tmp/obs_avoid_terminator_layout.json}"
  write_terminator_layout_json "${TERMINATOR_LAYOUT_JSON}"
  exec terminator -j "${TERMINATOR_LAYOUT_JSON}"
fi

if tmux has-session -t "${SESSION}" 2>/dev/null && [[ "${RECREATE_SESSION}" == "1" ]]; then
  tmux kill-session -t "${SESSION}"
fi

if ! tmux has-session -t "${SESSION}" 2>/dev/null; then
  tmux new-session -d -s "${SESSION}" -n sim

  local_sim_px4="${SESSION}:sim.0"
  tmux send-keys -t "${local_sim_px4}" \
    "cd '${PX4_DIR}' && PX4_GZ_WORLD='${WORLD_NAME}' make px4_sitl '${PX4_TARGET}'" C-m

  local_sim_qgc="$(tmux split-window -h -d -P -F "#{pane_id}" -t "${local_sim_px4}")"
  tmux send-keys -t "${local_sim_qgc}" \
    "cd '${QGC_DIR}' && if [[ -x '${QGC_APP}' ]]; then '${QGC_APP}'; else echo '[warn] QGroundControl AppImage not executable: ${QGC_APP}'; exec bash; fi" C-m

  local_sim_mavros="$(tmux split-window -v -d -P -F "#{pane_id}" -t "${local_sim_px4}")"
  tmux send-keys -t "${local_sim_mavros}" \
    "source '${ROS_SETUP}' && echo '[wait] waiting for mavros package ...' && until ros2 pkg prefix mavros >/dev/null 2>&1; do sleep 1; done && while true; do ros2 launch mavros '${MAVROS_LAUNCH_FILE}' fcu_url:='${FCU_URL}' respawn_mavros:='${MAVROS_RESPAWN}' use_sim_time:=true; rc=\$?; if [[ \"\$rc\" -eq 130 ]]; then echo '[info] mavros stopped by user'; break; fi; echo \"[warn] mavros exited (rc=\$rc), retrying in 2s\"; sleep 2; done; exec bash" C-m
  tmux select-layout -t "${SESSION}:sim" tiled

  tmux new-window -t "${SESSION}" -n bridges
  local_br_clock="${SESSION}:bridges.0"
  tmux send-keys -t "${local_br_clock}" \
    "source '${ROS_SETUP}' && ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'" C-m

  local_br_vert="$(tmux split-window -h -d -P -F "#{pane_id}" -t "${local_br_clock}")"
  tmux send-keys -t "${local_br_vert}" \
    "source '${ROS_SETUP}' && ros2 run ros_gz_bridge parameter_bridge '${TOPIC_VERT}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' --ros-args -p use_sim_time:=true" C-m

  local_br_horiz="$(tmux split-window -v -d -P -F "#{pane_id}" -t "${local_br_clock}")"
  tmux send-keys -t "${local_br_horiz}" \
    "source '${ROS_SETUP}' && ros2 run ros_gz_bridge parameter_bridge '${TOPIC_HORIZ}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' --ros-args -p use_sim_time:=true" C-m
  tmux select-layout -t "${SESSION}:bridges" tiled

  tmux new-window -t "${SESSION}" -n nodes
  local_nodes_scan="${SESSION}:nodes.0"
  tmux send-keys -t "${local_nodes_scan}" \
    "source '${ROS_SETUP}' && ros2 run vertical_lidar_mapper scan_frame_override_node --ros-args -p use_sim_time:=true" C-m

  local_nodes_tf_vert="$(tmux split-window -h -d -P -F "#{pane_id}" -t "${local_nodes_scan}")"
  tmux send-keys -t "${local_nodes_tf_vert}" \
    "source '${ROS_SETUP}' && ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch -1.570796 --roll 0 --frame-id base_link --child-frame-id lidar_vert_link --ros-args -p use_sim_time:=true" C-m

  local_nodes_tf_horiz="$(tmux split-window -v -d -P -F "#{pane_id}" -t "${local_nodes_scan}")"
  tmux send-keys -t "${local_nodes_tf_horiz}" \
    "source '${ROS_SETUP}' && ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id base_link --child-frame-id lidar_horiz_link --ros-args -p use_sim_time:=true" C-m

  local_nodes_odom="$(tmux split-window -v -d -P -F "#{pane_id}" -t "${local_nodes_tf_vert}")"
  tmux send-keys -t "${local_nodes_odom}" \
    "source '${ROS_SETUP}' && echo '[wait] waiting for MAVROS nodes ...' && until ros2 node list 2>/dev/null | grep -q '^/mavros'; do sleep 1; done && if ! ros2 topic list 2>/dev/null | grep -q '^/mavros/local_position/odom$'; then echo '[warn] /mavros/local_position/odom not listed yet; px4_odom_flatten_node will wait for messages.'; fi && echo '[run] starting px4_odom_flatten_node first (odom->base_link)' && ros2 run odom_flatten px4_odom_flatten_node --ros-args -p use_sim_time:=true -p odom_topic:=/mavros/local_position/odom -p parent_frame:=odom -p child_frame:=base_link" C-m

  local_nodes_mavros_params="$(tmux split-window -v -d -P -F "#{pane_id}" -t "${local_nodes_tf_horiz}")"
  tmux send-keys -t "${local_nodes_mavros_params}" \
    "source '${ROS_SETUP}' && echo '[wait] waiting for px4_odom_flatten_node ...' && until ros2 node list 2>/dev/null | grep -q '^/px4_odom_flatten_node$'; do sleep 1; done && echo '[run] applying MAVROS use_sim_time params ...' && for n in \$(ros2 node list | grep '^/mavros'); do ros2 param set \"\$n\" use_sim_time true 2>/dev/null || true; done && echo '[done] mavros params applied'; exec bash" C-m
  tmux select-layout -t "${SESSION}:nodes" tiled

  tmux new-window -t "${SESSION}" -n obs_avoid
  local_obs_mode="${SESSION}:obs_avoid.0"
  tmux send-keys -t "${local_obs_mode}" \
    "source '${ROS_SETUP}' && echo '[ready] mode pane is blank. choose one:' && echo '  ${USER_CTRL_RUN_CMD}' && echo '  ${SPIRAL_RUN_CMD}' && exec bash" C-m

  local_obs_dwa="$(tmux split-window -h -d -P -F "#{pane_id}" -t "${local_obs_mode}")"
  tmux send-keys -t "${local_obs_dwa}" \
    "source '${ROS_SETUP}' && echo '[wait] delaying ${OBS_AVOID_DELAY_SEC}s before obs_avoid startup ...' && sleep ${OBS_AVOID_DELAY_SEC} && ros2 run obs_avoid local_planner_mode_a --ros-args -p use_sim_time:=true ${DWA_ROS_ARGS}" C-m

  local_obs_mission="$(tmux split-window -v -d -P -F "#{pane_id}" -t "${local_obs_dwa}")"
  tmux send-keys -t "${local_obs_mission}" "${MISSION_OBS_AVOID_CMD}" C-m
  tmux select-layout -t "${SESSION}:obs_avoid" tiled

  tmux new-window -t "${SESSION}" -n rviz
  tmux send-keys -t "${SESSION}:rviz.0" \
    "source '${ROS_SETUP}' && rviz2 --ros-args -p use_sim_time:=true" C-m

  tmux select-window -t "${SESSION}:sim"
fi

if [[ "${ATTACH_SESSION}" != "1" ]]; then
  echo "[info] tmux session ready: ${SESSION}"
  exit 0
fi

if command -v terminator >/dev/null 2>&1; then
  exec terminator -x bash -lc "tmux attach -t '${SESSION}'"
fi

echo "[warn] terminator not found, attaching to tmux in current terminal."
exec tmux attach -t "${SESSION}"
