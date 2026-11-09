#!/usr/bin/env bash

uma_platform() {
  case "$1:$2" in
    ubuntu:22.04) UMA_ROS_DISTRO=humble; UMA_PYTHON_VERSION=3.10 ;;
    ubuntu:24.04) UMA_ROS_DISTRO=jazzy; UMA_PYTHON_VERSION=3.12 ;;
    *) echo "支持 Ubuntu 22.04/Humble 和 Ubuntu 24.04/Jazzy，当前为 $1 $2" >&2; return 1 ;;
  esac
  if [[ -n "${3:-}" && "$3" != "$UMA_ROS_DISTRO" ]]; then
    echo "Ubuntu $2 需要 ROS $UMA_ROS_DISTRO，当前环境为 $3。请在新终端加载匹配的环境。" >&2
    return 1
  fi
}

uma_detect_platform() {
  source /etc/os-release
  uma_platform "$ID" "$VERSION_ID" "${ROS_DISTRO:-}"
}

uma_check_python() {
  if [[ "$1" != "$UMA_PYTHON_VERSION" ]]; then
    echo "ROS $UMA_ROS_DISTRO 需要 Python $UMA_PYTHON_VERSION，当前为 $1。请指定匹配的 CONDA_ENV 后执行完整构建。" >&2
    return 1
  fi
}

uma_check_workspace() {
  local recorded
  if [[ -f "$1/install/.uma-platform" ]]; then
    read -r recorded < "$1/install/.uma-platform"
    if [[ "$recorded" != "$UMA_ROS_DISTRO:$UMA_PYTHON_VERSION" ]]; then
      echo "工作区由 $recorded 构建，请使用 build.sh 的重新构建模式生成 $UMA_ROS_DISTRO:$UMA_PYTHON_VERSION 产物。" >&2
      return 1
    fi
  elif [[ "$UMA_ROS_DISTRO" == jazzy && -d "$1/install" ]]; then
    echo "工作区缺少平台记录，请使用 build.sh 的重新构建模式生成 Jazzy 产物。" >&2
    return 1
  fi
}

uma_source_environment() {
  export PYTHONNOUSERSITE=1
  uma_detect_platform || return 1
  uma_check_python "$(python -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')" || return 1
  uma_check_workspace "$WORKSPACE_DIR" || return 1
  source "/opt/ros/${UMA_ROS_DISTRO}/setup.bash" || return 1
  source "$WORKSPACE_DIR/install/setup.bash" || return 1
  python -c 'import rclpy; from sensor_msgs.msg import JointState; JointState.__class__.__import_type_support__()'
}
