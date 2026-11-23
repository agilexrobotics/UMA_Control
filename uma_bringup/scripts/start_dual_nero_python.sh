#!/usr/bin/env bash
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
LAUNCH_FILE="${LAUNCH_FILE:-demo_nero_bringup.launch.py}"
EXTRA_LAUNCH_ARGS=(backend:=python)

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
source "${SCRIPT_DIR}/../../platform.sh"
uma_source_environment

set -u

CONDA_SITE="${CONDA_SITE:-${CONDA_PREFIX}/lib/python${UMA_PYTHON_VERSION}/site-packages}"

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

# Do not terminate drivers belonging to another launch session.  In particular,
# a leftover driver must be dealt with before opening the same arm CAN again.
if RUNNING_DRIVERS="$(pgrep -af '[a]gx_arm_ctrl/lib/agx_arm_ctrl/agx_arm_ctrl_single([[:space:]]|$)')"; then
  echo "检测到已运行的 AGX 机械臂驱动，拒绝重复启动：" >&2
  echo "${RUNNING_DRIVERS}" >&2
  echo "请先按设备规范安全停机并清理旧启动会话，然后重试。" >&2
  exit 1
else
  PSEARCH_RC=$?
  if [[ "${PSEARCH_RC}" -ne 1 ]]; then
    echo "无法检查已有 AGX 驱动，取消启动（pgrep 返回 ${PSEARCH_RC}）。" >&2
    exit "${PSEARCH_RC}"
  fi
fi

# Keep launch in the foreground and replace this shell.  Ctrl+C then reaches
# ROS launch as SIGINT, allowing it to stop, wait for, and reap its own children.
# Sending SIGTERM to launch or exiting a background wrapper can orphan them.
exec ros2 launch uma_bringup "${LAUNCH_FILE}" "${EXTRA_LAUNCH_ARGS[@]}" "$@"
