#!/usr/bin/env bash
# 构建 UMA_Control 和 UMA_SDK。

set -Eeo pipefail

if (( $# )); then
  printf '请直接运行，在菜单中选择构建方式：\n  bash %q\n' "$0" >&2
  exit 2
fi

# 路径与默认配置
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONTROL_DIR="${SCRIPT_DIR}"
source "${SCRIPT_DIR}/platform.sh"
PYTHON_BIN="/usr/bin/python3"
EXPECTED_PKGS=(uma_ros uma_msgs uma_bringup uma_description \
               agx_arm_msgs agx_arm_ctrl arm_safety_monitor uma_sdk)

WORKSPACE_DIR="${WORKSPACE_DIR:-}"
ROS_DISTRO="${ROS_DISTRO:-}"
CONDA_BASE="${CONDA_BASE:-}"
CONDA_ENV="${CONDA_ENV:-uma}"

OPERATION=""            # full | rebuild
OPERATION_LABEL=""
BACKEND="both"          # cpp | python | both
DO_CLEAN=0              # 重新构建: 删除 build/install/log
SKIP_PYTHON=0
STAGES=()
export PYTHONNOUSERSITE=1

# 依赖版本
APT_PACKAGES=(
  python3-colcon-common-extensions python3-colcon-recursive-crawl
  python3-rosdep python3-catkin-pkg python3-pip python3-empy python3-lark
  python3-numpy python3-scipy python3-tk python3-matplotlib
  can-utils ethtool iproute2
  libboost-all-dev libeigen3-dev liborocos-kdl-dev
  libnlopt-dev libnlopt-cxx-dev
)
CONDA_PACKAGES=(pinocchio=3.2.0 casadi=3.6.7 scipy typing_extensions exceptiongroup)
PIP_PACKAGES=(lark numpy==2.0.2 empy==3.3.4 pyyaml opencv-python ur-rtde netifaces meshcat)
PYAGXARM_URL="git+https://github.com/agilexrobotics/pyAgxArm.git"
CONDA_CHANNEL=(-c conda-forge)

# 终端输出与日志
if [[ -t 1 ]]; then
  C_RST=$'\033[0m'; C_BLD=$'\033[1m'; C_DIM=$'\033[2m'
  C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_ERR=$'\033[31m'; C_CYN=$'\033[36m'
else
  C_RST=""; C_BLD=""; C_DIM=""; C_OK=""; C_WARN=""; C_ERR=""; C_CYN=""
fi

LOG_FILE=""
say()  { printf '%s\n' "$*"; if [[ -n "${LOG_FILE}" ]]; then printf '%s\n' "$*" >>"${LOG_FILE}"; fi; return 0; }
warn() { printf '%s\n' "$*" >&2; if [[ -n "${LOG_FILE}" ]]; then printf '%s\n' "$*" >>"${LOG_FILE}"; fi; return 0; }
die()  { warn "${C_ERR}错误：${C_RST}$*"; exit 1; }

STEP_NO=0 TOTAL_STEPS=0 CURRENT_STAGE=""
step() {
  STEP_NO=$((STEP_NO + 1))
  CURRENT_STAGE="$*"
  local elapsed=$(( $(date +%s) - START_TS ))
  say ""
  say "${C_BLD}${C_CYN}[${STEP_NO}/${TOTAL_STEPS}] $* ${C_DIM}（已用 ${elapsed} 秒）${C_RST}"
}

run() {
  say "  ${C_DIM}\$ $*${C_RST}"
  "$@" 2>&1 | tee -a "${LOG_FILE}"
}

on_error() {
  local rc=$?
  warn ""
  warn "${C_ERR}${CURRENT_STAGE:-环境检查}失败，退出码 ${rc}。${C_RST}"
  [[ -n "${LOG_FILE}" ]] && warn "日志：${LOG_FILE}"
  warn "请处理上方错误后重新运行构建脚本。"
  exit "${rc}"
}

# 按环境变量、已激活环境、PATH、常见目录的顺序查找 Conda。
detect_conda_base() {
  local -a cands=()
  local c exe
  [[ -n "${CONDA_BASE}" ]] && cands+=("${CONDA_BASE}")
  if [[ -n "${CONDA_PREFIX:-}" ]]; then
    if [[ "${CONDA_PREFIX}" == */envs/* ]]; then cands+=("${CONDA_PREFIX%%/envs/*}")
    else                                        cands+=("${CONDA_PREFIX}"); fi
  fi
  exe="$(command -v conda 2>/dev/null || true)"
  if [[ -n "${exe}" ]]; then cands+=("$(dirname -- "$(dirname -- "${exe}")")"); fi
  cands+=("${HOME}/miniconda3" "${HOME}/anaconda3" "${HOME}/miniforge3" \
          "${HOME}/mambaforge" "/opt/conda")

  for c in "${cands[@]}"; do
    if [[ -n "${c}" && -f "${c}/etc/profile.d/conda.sh" ]]; then printf '%s' "${c}"; return 0; fi
  done
  return 1
}

locate_workspace() {
  local cand
  if [[ -n "${WORKSPACE_DIR}" ]]; then
    [[ -d "${WORKSPACE_DIR}" ]] \
      || die "工作区目录不存在：${WORKSPACE_DIR}"
    WORKSPACE_DIR="$(cd -- "${WORKSPACE_DIR}" && pwd)"
  else
    cand="$(cd -- "${CONTROL_DIR}/../.." && pwd)"
    if [[ -d "${cand}/src/UMA_SDK" ]]; then
      WORKSPACE_DIR="${cand}"
    elif [[ -d "$(dirname -- "${CONTROL_DIR}")/UMA_SDK" ]]; then
      WORKSPACE_DIR="$(dirname -- "${CONTROL_DIR}")"
    else
      WORKSPACE_DIR="${cand}"
    fi
  fi

  if   [[ -d "${WORKSPACE_DIR}/src/UMA_SDK" ]]; then SDK_DIR="${WORKSPACE_DIR}/src/UMA_SDK"
  elif [[ -d "${WORKSPACE_DIR}/UMA_SDK" ]];     then SDK_DIR="${WORKSPACE_DIR}/UMA_SDK"
  else                                                SDK_DIR=""
  fi
}

# 构建菜单
CHOICE=""
choose_one() {
  local title="$1" defval="$2"; shift 2
  local -a vals=() descs=()
  while (( $# >= 2 )); do vals+=("$1"); descs+=("$2"); shift 2; done

  local n=${#vals[@]} defidx=1 i
  for (( i = 0; i < n; i++ )); do
    [[ "${vals[i]}" == "${defval}" ]] && defidx=$((i + 1))
  done

  if [[ ! -t 0 ]]; then
    say "${title}：${descs[defidx - 1]}（非交互默认选项）"
    CHOICE="${defval}"
    return 0
  fi

  say ""
  say "${C_BLD}${title}${C_RST}"
  for (( i = 0; i < n; i++ )); do say "  $((i + 1))) ${descs[i]}"; done

  local reply=""
  while :; do
    printf '  输入序号 [1-%d]，回车选择 %d：' "${n}" "${defidx}"
    if ! read -r reply; then
      say ""
      say "  使用默认选项：${descs[defidx - 1]}"
      reply="${defidx}"
    fi
    [[ -z "${reply}" ]] && reply="${defidx}"
    if [[ "${reply}" =~ ^[0-9]+$ ]] && (( reply >= 1 && reply <= n )); then
      CHOICE="${vals[reply - 1]}"
      return 0
    fi
    say "  请输入 1 到 ${n} 之间的数字。"
  done
}

choose_operation() {
  choose_one "选择构建方式" "full" \
    full    "完整构建：安装依赖、构建并校验" \
    rebuild "重新构建：删除 build/install/log 后构建并校验"
  OPERATION="${CHOICE}"
  case "${OPERATION}" in
    full)    OPERATION_LABEL="完整构建"; DO_CLEAN=0 ;;
    rebuild) OPERATION_LABEL="重新构建"; DO_CLEAN=1 ;;
  esac
}

# 完整构建按桥接类型安装依赖。
choose_backend() {
  if [[ "${OPERATION}" != "full" ]]; then
    BACKEND="both"
    SKIP_PYTHON=1
    return 0
  fi
  choose_one "选择桥接类型" "both" \
    cpp    "C++ 桥接" \
    python "Python 桥接（安装 python-can 和 Python SDK）" \
    both   "C++ 和 Python 桥接"
  BACKEND="${CHOICE}"
  if [[ "${BACKEND}" == "cpp" ]]; then SKIP_PYTHON=1; else SKIP_PYTHON=0; fi
}

# Conda 环境与依赖检查
conda_env_exists() {
  [[ -n "${CONDA_BASE}" && -f "${CONDA_BASE}/etc/profile.d/conda.sh" ]] || return 1
  # shellcheck disable=SC1090,SC1091
  source "${CONDA_BASE}/etc/profile.d/conda.sh" >/dev/null 2>&1 || return 1
  conda env list 2>/dev/null | awk '!/^#/ && NF {print $1}' | grep -qx -- "$1"
}

activate_conda() {
  if [[ -n "${CONDA_PREFIX:-}" && "${CONDA_PREFIX}" == */envs/"${CONDA_ENV}" ]]; then
    return 0
  fi
  [[ -n "${CONDA_BASE}" && -f "${CONDA_BASE}/etc/profile.d/conda.sh" ]] \
    || die "找不到 Conda：${CONDA_BASE:-请设置 CONDA_BASE}"
  # shellcheck disable=SC1090,SC1091
  source "${CONDA_BASE}/etc/profile.d/conda.sh"
  conda activate "${CONDA_ENV}" >/dev/null 2>&1 \
    || die "无法激活 Conda 环境 ${CONDA_ENV}。请先选择完整构建，安装环境和依赖。"
}

env_missing_pip() {
  python - "$@" <<'PY' 2>/dev/null || echo "python-unavailable"
import sys, importlib.metadata as md
bad = []
for spec in sys.argv[1:]:
    name, _, want = spec.partition("==")
    try:
        got = md.version(name)
    except Exception:
        bad.append(name)
        continue
    if want and got != want:
        bad.append(f"{name}({got}!={want})")
print(" ".join(bad))
PY
}

env_missing_conda() {
  python - <<'PY' 2>/dev/null || echo "python-unavailable"
import importlib
checks = [("pinocchio", "3.2.0"), ("casadi", "3.6.7"),
          ("scipy", None), ("typing_extensions", None), ("exceptiongroup", None)]
bad = []
for mod, want in checks:
    try:
        m = importlib.import_module(mod)
    except Exception:
        bad.append(mod)
        continue
    if want and getattr(m, "__version__", None) != want:
        bad.append(f"{mod}({getattr(m, '__version__', '?')}!={want})")
print(" ".join(bad))
PY
}

env_python_version_ok() {
  uma_check_python "$(python -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
}

conda_import_ok() {
  ( activate_conda >/dev/null 2>&1 && python -c "import $1" >/dev/null 2>&1 )
}

ensure_sudo() {
  if ! sudo -n true 2>/dev/null; then
    say "  安装系统依赖需要 sudo 权限，请输入密码。"
    sudo -v || die "无法获取 sudo 权限，请检查当前用户的权限。"
  fi
}

# 构建前检查
MISSING=()
NOTES=()

preflight() {
  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    uma_platform "${ID}" "${VERSION_ID}" "${ROS_DISTRO}" || return 1
  fi

  if [[ -z "${ROS_DISTRO}" || ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    MISSING+=("找不到 /opt/ros/${ROS_DISTRO}/setup.bash。请为 ${PRETTY_NAME} 安装 ROS ${ROS_DISTRO}：

        sudo apt install -y software-properties-common curl
        sudo add-apt-repository universe -y && sudo apt update
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo \$UBUNTU_CODENAME) main\" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
        sudo apt update && sudo apt install -y ros-${ROS_DISTRO}-desktop ros-dev-tools")
  fi

  # IK 运行需要匹配 ROS Python 版本的 Conda 环境。
  if [[ -z "${CONDA_BASE}" || ! -f "${CONDA_BASE}/etc/profile.d/conda.sh" ]]; then
    MISSING+=("找不到 Conda。请设置 CONDA_BASE，或安装 Miniconda：

        mkdir -p \"\${HOME}/Downloads\"
        curl -fsSL https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -o \"\${HOME}/Downloads/miniconda.sh\"
        bash \"\${HOME}/Downloads/miniconda.sh\" -b -p \"\${HOME}/miniconda3\"
        \"\${HOME}/miniconda3/bin/conda\" init bash")
  fi

  if [[ -z "${SDK_DIR}" ]]; then
    MISSING+=("在 ${WORKSPACE_DIR} 找不到 UMA_SDK。请将两个仓库并列放在工作区的 src/ 下：

        mkdir -p ${WORKSPACE_DIR}/src && cd ${WORKSPACE_DIR}/src
        git clone https://github.com/agilexrobotics/UMA_Control.git
        git clone https://github.com/agilexrobotics/UMA_SDK.git")
  fi

  if [[ ! -x "${PYTHON_BIN}" ]]; then
    MISSING+=("消息生成需要 ${PYTHON_BIN}，请执行：sudo apt install -y python3")
  elif ! uma_check_python "$("${PYTHON_BIN}" -c 'import sys; print("%d.%d" % sys.version_info[:2])')"; then
    MISSING+=("${PYTHON_BIN} 必须使用 Python ${UMA_PYTHON_VERSION}，以生成与 ${ROS_DISTRO} 匹配的消息扩展。")
  fi

  local free_gb
  free_gb=$(df -BG --output=avail "${WORKSPACE_DIR}" 2>/dev/null | tail -1 | tr -dc '0-9')
  if [[ -n "${free_gb}" ]] && (( free_gb < 5 )); then
    MISSING+=("工作区可用空间为 ${free_gb} GB，请清理到至少 5 GB 后再构建。")
  fi

  if command -v curl >/dev/null 2>&1; then
    if ! curl -fsS --max-time 8 -o /dev/null https://github.com 2>/dev/null; then
      NOTES+=("无法连接 github.com，安装 pyAgxArm 和更新 rosdep 可能失败。")
    fi
  fi

  # 检查用户目录中是否有同名 Python 包覆盖 Conda 包。
  if [[ -f "${CONDA_BASE}/etc/profile.d/conda.sh" ]] && conda_env_exists "${CONDA_ENV}"; then
    local iso plain
    iso="$( (activate_conda >/dev/null 2>&1 && python -c 'import pinocchio; print(pinocchio.__file__)') 2>/dev/null || echo '?')"
    plain="$( (activate_conda >/dev/null 2>&1 && PYTHONNOUSERSITE= python -c 'import pinocchio; print(pinocchio.__file__)') 2>/dev/null || echo '?')"
    if [[ "${iso}" != "${plain}" ]]; then
      NOTES+=("~/.local 中的 Python 包覆盖了 Conda 包。手动启动前执行 export PYTHONNOUSERSITE=1；start_*.sh 已设置该变量。")
    fi
  fi
}

# 安装、构建与校验
stage_apt() {
  step "安装系统依赖"
  local missing=() p
  for p in "${APT_PACKAGES[@]}"; do
    dpkg -s "${p}" >/dev/null 2>&1 || missing+=("${p}")
  done
  if (( ${#missing[@]} == 0 )); then
    say "  ${C_OK}✓${C_RST} ${#APT_PACKAGES[@]} 个系统依赖已安装"
    return 0
  fi
  say "  待安装 ${#missing[@]} 项：${missing[*]}"
  ensure_sudo
  run sudo apt-get update
  run sudo apt-get install -y "${missing[@]}"
}

stage_conda() {
  step "配置 Conda 环境 ${CONDA_ENV}（Python ${UMA_PYTHON_VERSION}）"

  if conda_env_exists "${CONDA_ENV}"; then
    say "  ${C_OK}✓${C_RST} 环境已存在"
  else
    # shellcheck disable=SC1090,SC1091
    source "${CONDA_BASE}/etc/profile.d/conda.sh"
    run conda create -n "${CONDA_ENV}" "${CONDA_CHANNEL[@]}" "python=${UMA_PYTHON_VERSION}" -y
  fi

  activate_conda

  if ! env_python_version_ok; then
    die "${CONDA_ENV} 需要 Python ${UMA_PYTHON_VERSION} (实际 $(python -V 2>&1))。请指定新的环境名称后完整构建，例如 CONDA_ENV=uma_${ROS_DISTRO} bash ${SCRIPT_DIR}/build.sh"
  fi

  local bad
  bad="$(env_missing_conda)"
  if [[ -n "${bad}" ]]; then
    say "  更新 Conda 依赖：${bad}"
    run conda install -y "${CONDA_CHANNEL[@]}" "python=${UMA_PYTHON_VERSION}" "${CONDA_PACKAGES[@]}"
  else
    say "  ${C_OK}✓${C_RST} Conda 依赖检查通过"
  fi

  bad="$(env_missing_pip "${PIP_PACKAGES[@]}")"
  if [[ -n "${bad}" ]]; then
    say "  更新 pip 依赖：${bad}"
    run python -m pip install "${PIP_PACKAGES[@]}"
  else
    say "  ${C_OK}✓${C_RST} pip 依赖检查通过"
  fi

  if python -c "import pyAgxArm" >/dev/null 2>&1; then
    say "  ${C_OK}✓${C_RST} pyAgxArm 导入成功"
  else
    say "  从 GitHub 安装 pyAgxArm"
    run python -m pip install "${PYAGXARM_URL}"
  fi
}

stage_rosdep() {
  step "检查 ROS 依赖"
  # shellcheck disable=SC1090,SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"

  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    say "  初始化 rosdep"
    ensure_sudo
    run sudo rosdep init
  else
    say "  ${C_OK}✓${C_RST} rosdep 已初始化"
  fi

  if [[ ! -f "${HOME}/.ros/rosdep/sources.cache/index" ]]; then
    run rosdep update
  else
    say "  使用已有 rosdep 缓存"
  fi

  if rosdep check --from-paths "${CONTROL_DIR}" "${SDK_DIR}" --ignore-src -r; then
    say "  ${C_OK}✓${C_RST} ROS 依赖检查通过"
  else
    ensure_sudo
    run rosdep install --from-paths "${CONTROL_DIR}" "${SDK_DIR}" --ignore-src -r -y
  fi
}

stage_build() {
  step "构建工作区"
  activate_conda
  env_python_version_ok || die "Conda Python 与 ROS 不匹配"
  # shellcheck disable=SC1090,SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"

  # colcon 在工作区根目录生成 build、install 和 log。
  cd "${WORKSPACE_DIR}"
  run colcon build \
    --base-paths "${CONTROL_DIR}" "${SDK_DIR}" \
    --symlink-install \
    --cmake-args "-DPython3_EXECUTABLE=${PYTHON_BIN}" -DBUILD_TESTING=OFF
  printf '%s:%s\n' "$UMA_ROS_DISTRO" "$UMA_PYTHON_VERSION" > "${WORKSPACE_DIR}/install/.uma-platform"
}

stage_python() {
  step "安装 Python 桥接依赖"
  activate_conda

  local bad
  bad="$(env_missing_pip python-can)"
  if [[ -n "${bad}" ]]; then
    run python -m pip install python-can
  else
    say "  ${C_OK}✓${C_RST} python-can 已安装"
  fi

  local where=""
  where="$(python -c 'import os, uma_sdk; print(os.path.dirname(os.path.abspath(uma_sdk.__file__)))' 2>/dev/null || true)"
  if [[ "${where}" == "${SDK_DIR}/python" ]]; then
    say "  ${C_OK}✓${C_RST} Python SDK 已关联到 ${SDK_DIR}/python"
  else
    run python -m pip install -e "${SDK_DIR}/python"
  fi
}

stage_verify() {
  step "校验构建结果"
  local failed=0

  if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    say "  ${C_OK}✓${C_RST} install/setup.bash 已生成"
  else
    say "  ${C_ERR}✗${C_RST} 缺少 install/setup.bash"
    failed=1
  fi

  local p
  for p in "${EXPECTED_PKGS[@]}"; do
    if [[ ! -d "${WORKSPACE_DIR}/install/${p}" ]]; then
      say "  ${C_ERR}✗${C_RST} 缺少 install/${p}"
      failed=1
    fi
  done
  (( failed )) || say "  ${C_OK}✓${C_RST} ${#EXPECTED_PKGS[@]} 个必需 ROS 包已安装"

  if ( activate_conda && source "/opt/ros/${ROS_DISTRO}/setup.bash" && source "${WORKSPACE_DIR}/install/setup.bash" && python -c 'import rclpy, pinocchio, pinocchio.casadi, casadi, scipy, pyAgxArm; from uma_msgs.msg import MtSensorStatus; MtSensorStatus.__class__.__import_type_support__()' ); then
    say "  ${C_OK}✓${C_RST} ROS 消息、IK 和驱动依赖导入成功"
  else
    say "  ${C_ERR}✗${C_RST} ROS 消息、IK 或驱动依赖导入失败，请查看上方错误"
    failed=1
  fi

  if (( SKIP_PYTHON )); then
    say "  Python 桥接依赖：本次构建未检查"
  elif conda_import_ok "can, uma_sdk"; then
    say "  ${C_OK}✓${C_RST} Python 桥接依赖导入成功"
  else
    say "  ${C_ERR}✗${C_RST} Python 桥接需要 python-can 和 uma_sdk，请选择完整构建安装"
    failed=1
  fi

  # 在独立 shell 中加载环境，检查 colcon 识别到的包。
  if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    local n
    n="$( cd "${WORKSPACE_DIR}" \
          && source "/opt/ros/${ROS_DISTRO}/setup.bash" >/dev/null 2>&1 \
          && source "${WORKSPACE_DIR}/install/setup.bash" >/dev/null 2>&1 \
          && colcon list --base-paths "${CONTROL_DIR}" "${SDK_DIR}" 2>/dev/null | wc -l )" || n=0
    if [[ "${n:-0}" -ge ${#EXPECTED_PKGS[@]} ]]; then
      say "  ${C_OK}✓${C_RST} colcon 识别到 ${n} 个包"
    else
      say "  ${C_ERR}✗${C_RST} colcon 识别到 ${n:-0} 个包，至少需要 ${#EXPECTED_PKGS[@]} 个"
      failed=1
    fi
  fi

  return "${failed}"
}

next_steps() {
  local suffix=cpp model command
  [[ "$BACKEND" == python ]] && suffix=python
  say ""
  say "1. 查询 CAN 接口，将 bus-info 填入 can_config.sh："
  printf -v command 'bash %q' "${CONTROL_DIR}/uma_bringup/scripts/find_all_can_port.sh"
  say "   $command"
  say "2. 配置 CAN："
  printf -v command 'bash %q' "${CONTROL_DIR}/uma_bringup/scripts/can_config.sh"
  say "   $command"
  say "3. 按型号选择一个启动命令："
  for model in single_piper single_piper_x dual_nero; do
    printf -v command 'CONDA_BASE=%q CONDA_ENV=%q WORKSPACE_DIR=%q bash %q' \
      "$CONDA_BASE" "$CONDA_ENV" "$WORKSPACE_DIR" \
      "${CONTROL_DIR}/uma_bringup/scripts/start_${model}_${suffix}.sh"
    say "   $command"
  done
}

# 主流程
main() {
  START_TS=$(date +%s)

  locate_workspace
  CONDA_BASE="$(detect_conda_base || true)"
  uma_detect_platform || die "系统与 ROS 发行版不匹配"
  ROS_DISTRO="$UMA_ROS_DISTRO"

  say "${C_BLD}UMA 构建${C_RST}"
  say "  工作区：${WORKSPACE_DIR}"
  say "  SDK：${SDK_DIR:-未找到}"
  say "  ROS：${ROS_DISTRO}，Python：${UMA_PYTHON_VERSION}"
  say "  Conda：${CONDA_ENV}（${CONDA_BASE:-未找到安装目录}）"

  choose_operation
  choose_backend
  if (( ! DO_CLEAN )); then
    uma_check_workspace "$WORKSPACE_DIR" || die "请重新构建工作区"
  fi

  preflight
  if (( ${#MISSING[@]} )); then
    warn "${C_ERR}请先处理以下问题，再运行构建脚本：${C_RST}"
    warn ""
    local m
    for m in "${MISSING[@]}"; do warn "  ${C_ERR}✗${C_RST} ${m}"; warn ""; done
    exit 1
  fi
  local k
  for k in "${NOTES[@]}"; do warn "${C_WARN}!${C_RST} ${k}"; done

  if (( DO_CLEAN )); then
    say ""
    say "${C_WARN}删除 ${WORKSPACE_DIR}/{build,install,log}${C_RST}"
    rm -rf "${WORKSPACE_DIR}/build" "${WORKSPACE_DIR}/install" "${WORKSPACE_DIR}/log"
  fi

  mkdir -p "${WORKSPACE_DIR}/log"
  LOG_FILE="${WORKSPACE_DIR}/log/build_$(date +%Y%m%d_%H%M%S).log"
  : >"${LOG_FILE}"
  trap on_error ERR

  say ""
  say "${C_BLD}${OPERATION_LABEL}${C_RST}"
  if [[ "${OPERATION}" == "full" ]]; then
    say "  桥接类型：${BACKEND}；IK 使用 Conda 环境 ${CONDA_ENV}"
  fi
  say "  日志：${LOG_FILE}"

  if [[ "${OPERATION}" == "full" ]]; then
    STAGES=(stage_apt stage_conda stage_rosdep stage_build)
    (( SKIP_PYTHON )) || STAGES+=(stage_python)
    STAGES+=(stage_verify)
  else
    STAGES=(stage_build stage_verify)
  fi
  TOTAL_STEPS=${#STAGES[@]}

  local st verify_rc=0
  for st in "${STAGES[@]}"; do
    if [[ "${st}" == "stage_verify" ]]; then
      stage_verify || verify_rc=$?
    else
      "${st}"
    fi
  done

  trap - ERR
  if (( verify_rc != 0 )); then
    warn ""
    warn "${C_ERR}校验失败，请处理上方标记为 ✗ 的项目。${C_RST}"
    warn "日志：${LOG_FILE}"
    exit 1
  fi
  say ""
  say "${C_OK}${C_BLD}构建与校验通过。${C_RST}"
  say "日志：${LOG_FILE}"
  next_steps
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main
fi
