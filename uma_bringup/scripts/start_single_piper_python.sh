#!/usr/bin/env bash
# ROS setup scripts reference some variables before defining them, so enable
# nounset only after Conda, ROS, and the workspace have been sourced.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-}"
if [[ -z "${WORKSPACE_DIR}" ]]; then
  _dir="${SCRIPT_DIR}"
  while [[ "${_dir}" != "/" ]]; do
    if [[ -f "${_dir}/install/setup.bash" ]]; then
      WORKSPACE_DIR="${_dir}"
      break
    fi
    _dir="$(dirname "${_dir}")"
  done
  WORKSPACE_DIR="${WORKSPACE_DIR:-${SCRIPT_DIR}}"
fi
CONDA_BASE="${CONDA_BASE:-${HOME}/miniconda3}"
CONDA_ENV="${CONDA_ENV:-uma}"
SIDE="${SIDE:-right}"
case "${SIDE}" in
  left|right) EXTRA_LAUNCH_ARGS=(side:=${SIDE} backend:=python) ;;
  *) echo "SIDE 只支持 left/right,got: ${SIDE}" >&2; exit 1 ;;
esac
LAUNCH_FILE="${LAUNCH_FILE:-demo_piper.launch.py}"

if [[ ! -f "${CONDA_BASE}/etc/profile.d/conda.sh" ]]; then
  echo "找不到 Conda: ${CONDA_BASE}" >&2
  exit 1
fi
if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "找不到工作区环境: ${WORKSPACE_DIR}/install/setup.bash,请先构建工作区。" >&2
  exit 1
fi

source "${CONDA_BASE}/etc/profile.d/conda.sh"
conda activate "${CONDA_ENV}"
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

set -u

CONDA_SITE="${CONDA_SITE:-${CONDA_PREFIX:+${CONDA_PREFIX}/lib/python3.10/site-packages}}"

if [[ -n "${CONDA_SITE}" && "${PYTHONPATH:-}" == *"${CONDA_SITE}"* ]]; then
  PYTHONPATH="${PYTHONPATH//${CONDA_SITE}/}"
  PYTHONPATH="${PYTHONPATH//::/:}"
  PYTHONPATH="${PYTHONPATH#:}"
  PYTHONPATH="${PYTHONPATH%:}"
  if [[ -n "${PYTHONPATH}" ]]; then
    export PYTHONPATH
  else
    unset PYTHONPATH
  fi
fi
export PYTHONNOUSERSITE=1

# Python 桥接节点需要 uma_sdk 源码可导入(未 pip install 时的兜底)
export PYTHONPATH="${WORKSPACE_DIR}/src/UMA_SDK/python${PYTHONPATH:+:${PYTHONPATH}}"

cd "${WORKSPACE_DIR}"

# 启动前清理上次遗留的栈进程(Ctrl+C 可能残留孤儿进程,重复节点会导致遥操抖动)
for pat in "agx_arm_ctrl_single" "teleop_single_arm_node" "uma_ros_node" "uma_ros_py_node"; do
  pkill -f "${pat}" 2>/dev/null || true
done

set +e
ros2 launch uma_bringup "${LAUNCH_FILE}" "${EXTRA_LAUNCH_ARGS[@]}" "$@" &
LAUNCH_PID=$!
cleanup() {
  kill "${LAUNCH_PID}" 2>/dev/null
  for pat in "teleop_single_arm_node" "uma_ros_node" "uma_ros_py_node"; do
    pkill -f "${pat}" 2>/dev/null || true
  done
}
trap 'cleanup; exit 130' INT TERM
wait "${LAUNCH_PID}"
LAUNCH_RC=$?
trap - INT TERM
cleanup
exit "${LAUNCH_RC}"
