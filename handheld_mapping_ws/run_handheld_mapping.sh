#!/usr/bin/env bash

WORKSPACE="/home/argen/my_robot_ws/handheld_mapping_ws"
LAUNCH_PATTERN="ros2 launch handheld_mapping handheld_nav.launch.py"

cleanup_stale_nodes() {
  local patterns=(
    "slam_gmapping" "rf2o_laser_odometry_node" "ydlidar_ros2_driver_node"
    "planner_server" "controller_server" "behavior_server" "bt_navigator"
    "waypoint_follower" "velocity_smoother" "lifecycle_manager"
    "virtual_goal_publisher" "stm32_bridge" "mqtt_bridge"
    "indoor_map_bridge" "indoor_mission_manager" "map_saver"
    "cmd_vel_monitor"
  )
  local pids=""
  # The launch parent owns the single-instance lock. Cleaning only child
  # nodes leaves that parent alive, so the next desktop click cannot start.
  while read -r pid; do
    [ -n "$pid" ] && pids+=" $pid"
  done < <(pgrep -f "$LAUNCH_PATTERN" || true)
  for pattern in "${patterns[@]}"; do
    while read -r pid; do
      [ -n "$pid" ] && pids+=" $pid"
    done < <(pgrep -f "install/.*/${pattern}([[:space:]]|$)" || true)
  done
  if [ -n "$pids" ]; then
    echo "发现旧的工程进程，正在清理:$pids"
    kill -TERM $pids 2>/dev/null || true
    sleep 2
    kill -KILL $pids 2>/dev/null || true
  fi
}

echo "========================================"
echo "  手持建图与导航系统"
echo "========================================"

cleanup_stale_nodes

cd "$WORKSPACE" || {
  echo "无法进入工作区：$WORKSPACE"
  read -r -p "按回车键关闭窗口..."
  exit 1
}

# Desktop launchers do not always inherit the interactive shell locale/path.
# Use a deterministic UTF-8 locale and make Python output visible immediately
# in the terminal window for startup diagnostics.
export LANG="${LANG:-C.UTF-8}"
export LC_ALL="${LC_ALL:-C.UTF-8}"
export PYTHONUNBUFFERED=1

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "未找到 ROS 2 Humble：/opt/ros/humble/setup.bash"
  read -r -p "按回车键关闭窗口..."
  exit 1
fi

source /opt/ros/humble/setup.bash

if [ ! -f install/setup.bash ]; then
  echo "尚未编译工程，正在执行首次编译..."
  if ! colcon build --symlink-install; then
    echo
    echo "工程编译失败，请检查上方错误信息。"
    read -r -p "按回车键关闭窗口..."
    exit 1
  fi
fi

source install/setup.bash

echo
echo "正在启动雷达、GMapping、Nav2、STM32、MQTT 和 RViz..."
echo "需要停止时，请在本窗口按 Ctrl+C。"
echo

ros2 launch handheld_mapping handheld_nav.launch.py
status=$?

echo
echo "程序已退出，状态码：$status"
read -r -p "按回车键关闭窗口..."
exit "$status"
