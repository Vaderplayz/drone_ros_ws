#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${1:-$HOME/drone_ros_ws}"
SRC_DIR="$WS_DIR/src"
VINS_DIR="$SRC_DIR/VINS-MONO-ROS2"
REPO_URL="https://github.com/dongbo19/VINS-MONO-ROS2.git"
BRANCH="ros2_jazzy"

if [[ ! -d "$SRC_DIR" ]]; then
  echo "[ERR] workspace src not found: $SRC_DIR"
  exit 1
fi

echo "[1/7] Install dependencies"
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  libeigen3-dev libceres-dev \
  ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-message-filters ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-tf2-eigen ros-jazzy-rviz2

echo "[2/7] Clone VINS-MONO-ROS2 (jazzy branch)"
if [[ ! -d "$VINS_DIR/.git" ]]; then
  git clone --branch "$BRANCH" "$REPO_URL" "$VINS_DIR"
else
  git -C "$VINS_DIR" fetch --all
  git -C "$VINS_DIR" checkout "$BRANCH"
  git -C "$VINS_DIR" pull --ff-only
fi

echo "[3/7] Disable OpenVINS packages in this workspace"
mkdir -p "$SRC_DIR/open_vins"
echo "disabled: switched to VINS-Mono workflow" > "$SRC_DIR/open_vins/COLCON_IGNORE"

echo "[4/7] Install PX4 downcam config template for VINS-Mono"
VINS_CFG_DIR="$VINS_DIR/config_pkg/config/px4_downcam"
mkdir -p "$VINS_CFG_DIR"
cp "$SRC_DIR/px4_vio_bridge/config/vinsmono_px4_downcam_template.yaml" \
   "$VINS_CFG_DIR/px4_downcam_config.yaml"

echo "[5/7] Create launch variants that point to px4_downcam config (best-effort)"
patch_launch() {
  local src_file="$1"
  local dst_file="$2"
  if [[ -f "$src_file" ]]; then
    cp "$src_file" "$dst_file"
    sed -i 's#config/euroc/euroc_config.yaml#config/px4_downcam/px4_downcam_config.yaml#g' "$dst_file"
    sed -i 's#config/euroc/euroc_config#config/px4_downcam/px4_downcam_config#g' "$dst_file"
    sed -i 's#euroc_config.yaml#px4_downcam_config.yaml#g' "$dst_file"
  fi
}

patch_launch "$VINS_DIR/feature_tracker/launch/feature_tracker.launch.py" \
             "$VINS_DIR/feature_tracker/launch/px4_downcam.launch.py"
patch_launch "$VINS_DIR/feature_tracker/launch/vins_feature_tracker.launch.py" \
             "$VINS_DIR/feature_tracker/launch/px4_downcam.launch.py"
patch_launch "$VINS_DIR/feature_tracker/launch/euroc.launch.py" \
             "$VINS_DIR/feature_tracker/launch/px4_downcam.launch.py"
patch_launch "$VINS_DIR/vins_estimator/launch/vins_estimator.launch.py" \
             "$VINS_DIR/vins_estimator/launch/px4_downcam.launch.py"
patch_launch "$VINS_DIR/vins_estimator/launch/euroc.launch.py" \
             "$VINS_DIR/vins_estimator/launch/px4_downcam.launch.py"

if [[ ! -f "$VINS_DIR/feature_tracker/launch/px4_downcam.launch.py" ]] || [[ ! -f "$VINS_DIR/vins_estimator/launch/px4_downcam.launch.py" ]]; then
  echo "[WARN] Could not auto-generate one or both px4_downcam launch files."
  echo "       Manually duplicate a working launch file in feature_tracker/vins_estimator and set config to:"
  echo "       config/px4_downcam/px4_downcam_config.yaml"
fi

echo "[6/7] Build VINS-Mono + bridge"
# Avoid AMENT_TRACE_SETUP_FILES unbound variable when caller has `set -u`.
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$WS_DIR"
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF

echo "[7/7] Done"
echo "Next: source $WS_DIR/install/setup.bash"
echo "Then run: ros2 launch px4_vio_bridge vinsmono_full_stack.launch.py"
