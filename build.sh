#!/usr/bin/env bash
# =============================================================================
# UMA_Control / UMA_SDK 一键构建脚本  ·  one-command build script
# =============================================================================

set -eo pipefail

if (( $# )); then
  printf '本脚本不需要命令行参数, 做什么、部署哪个 backend 都在终端里选择。\nThis script takes no command-line arguments; choose everything in the terminal.\n\n直接运行 / Just run:\n  bash %s\n' "$0" >&2
  exit 2
fi

# ------------------------------------------------------------------ 默认值 ---
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONTROL_DIR="${SCRIPT_DIR}"
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

# ------------------------------------------- README 中列出的依赖(逐条对应) ---
APT_PACKAGES=(
  python3-colcon-common-extensions python3-colcon-recursive-crawl
  python3-rosdep python3-catkin-pkg python3-pip
  python3-numpy python3-scipy python3-tk python3-matplotlib
  can-utils ethtool iproute2
  libboost-all-dev libeigen3-dev liborocos-kdl-dev
  libnlopt-dev libnlopt-cxx-dev
)
CONDA_PACKAGES=(pinocchio=3.2.0 casadi=3.6.7 scipy typing_extensions exceptiongroup)
PIP_PACKAGES=(lark numpy==2.0.2 empy==3.3.4 pyyaml opencv-python ur-rtde netifaces meshcat)
PYAGXARM_URL="git+https://github.com/agilexrobotics/pyAgxArm.git"
CONDA_CHANNEL=(-c conda-forge)

# ------------------------------------------------------------- 输出与日志 ---
if [[ -t 1 ]]; then
  C_RST=$'\033[0m'; C_BLD=$'\033[1m'; C_DIM=$'\033[2m'
  C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_ERR=$'\033[31m'; C_CYN=$'\033[36m'
else
  C_RST=""; C_BLD=""; C_DIM=""; C_OK=""; C_WARN=""; C_ERR=""; C_CYN=""
fi

LOG_FILE=""
say()  { printf '%s\n' "$*"; if [[ -n "${LOG_FILE}" ]]; then printf '%s\n' "$*" >>"${LOG_FILE}"; fi; return 0; }
warn() { printf '%s\n' "$*" >&2; if [[ -n "${LOG_FILE}" ]]; then printf '%s\n' "$*" >>"${LOG_FILE}"; fi; return 0; }
die()  { warn "${C_ERR}错误 / Error:${C_RST} $*"; exit 1; }

STEP_NO=0 TOTAL_STEPS=0 CURRENT_STAGE=""
step() {
  STEP_NO=$((STEP_NO + 1))
  CURRENT_STAGE="$*"
  local elapsed=$(( $(date +%s) - START_TS ))
  say ""
  say "${C_BLD}${C_CYN}━━━ [${STEP_NO}/${TOTAL_STEPS}] $* ${C_DIM}(已用 / elapsed ${elapsed}s)${C_RST}"
}

run() {
  say "  ${C_DIM}\$ $*${C_RST}"
  "$@" 2>&1 | tee -a "${LOG_FILE}"
}

on_error() {
  local rc=$?
  warn ""
  warn "${C_ERR}✗ 阶段失败 / Stage failed: ${CURRENT_STAGE:-前置检查 / preflight} (退出码 / exit ${rc})${C_RST}"
  [[ -n "${LOG_FILE}" ]] && warn "  完整日志 / Full log: ${LOG_FILE}"
  warn "  修好后重新运行本脚本即可, 已完成的步骤会自动跳过。"
  warn "  Fix the problem and re-run; completed steps are skipped automatically."
  exit "${rc}"
}

# ------------------------------------------------------------- 环境探测 ---
# 依次尝试: 环境变量 -> 已激活的 Conda -> PATH 里的 conda -> 常见安装位置
# Tries, in order: env var -> active Conda -> conda on PATH -> common locations
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

detect_ros_distros() {
  local d
  for d in /opt/ros/*/; do
    [[ -f "${d}setup.bash" ]] && basename -- "${d}"
  done
}

locate_workspace() {
  local cand
  if [[ -n "${WORKSPACE_DIR}" ]]; then
    [[ -d "${WORKSPACE_DIR}" ]] \
      || die "WORKSPACE_DIR 指向的目录不存在 / points to a directory that does not exist: ${WORKSPACE_DIR}"
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

# --------------------------------------------------------------- 交互菜单 ---
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
    say "${C_DIM}  ${title}${C_RST}"
    say "${C_DIM}  -> 非交互终端, 使用默认值 / non-interactive terminal, using default: ${defval}${C_RST}"
    CHOICE="${defval}"
    return 0
  fi

  say ""
  say "${C_BLD}${title}${C_RST}"
  for (( i = 0; i < n; i++ )); do say "  $((i + 1))) ${descs[i]}"; done

  local reply=""
  while :; do
    printf '  选择 / Choice [1-%d] (默认 / default %d): ' "${n}" "${defidx}"
    if ! read -r reply; then
      say ""
      say "  ${C_DIM}未读到输入, 使用默认值 / no input, using default: ${defval}${C_RST}"
      reply="${defidx}"
    fi
    [[ -z "${reply}" ]] && reply="${defidx}"
    if [[ "${reply}" =~ ^[0-9]+$ ]] && (( reply >= 1 && reply <= n )); then
      CHOICE="${vals[reply - 1]}"
      return 0
    fi
    say "  请输入 1 到 ${n} 之间的数字 / enter a number between 1 and ${n}"
  done
}

choose_operation() {
  choose_one "请选择要执行的操作 / Select an operation" "full" \
    full    "完整构建    安装依赖 + 构建工作区 + 校验              (推荐)
     Full build  install dependencies, build the workspace, verify   (recommended)" \
    rebuild "重新构建    删除 build/install/log 后重新构建 + 校验
     Rebuild     delete build/install/log, then rebuild and verify"
  OPERATION="${CHOICE}"
  case "${OPERATION}" in
    full)    OPERATION_LABEL="完整构建 / full build"; DO_CLEAN=0 ;;
    rebuild) OPERATION_LABEL="重新构建 / rebuild";    DO_CLEAN=1 ;;
  esac
}

# 只有完整构建会安装依赖, 才需要问 backend
# Only a full build installs dependencies, so only then is the backend relevant
choose_backend() {
  if [[ "${OPERATION}" != "full" ]]; then
    BACKEND="both"
    SKIP_PYTHON=1     # 重新构建不装 Python bridge 依赖, 校验也不应因此判失败
    return 0
  fi
  choose_one "请选择要部署的 bridge backend / Select the bridge backend" "both" \
    cpp    "cpp     只用 C++ bridge, 跳过 python-can 与 Python SDK
     cpp     C++ bridge only; skips python-can and the Python SDK" \
    python "python  要用 Python bridge
     python  use the Python bridge" \
    both   "both    两套都装
     both    install both"
  BACKEND="${CHOICE}"
  if [[ "${BACKEND}" == "cpp" ]]; then SKIP_PYTHON=1; else SKIP_PYTHON=0; fi
}

# ----------------------------------------------------------- 小工具函数 ---
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
    || die "找不到 Conda / Conda not found (${CONDA_BASE:-未探测到 / not detected})"
  # shellcheck disable=SC1090,SC1091
  source "${CONDA_BASE}/etc/profile.d/conda.sh"
  conda activate "${CONDA_ENV}" >/dev/null 2>&1 \
    || die "无法激活 Conda 环境 / cannot activate Conda environment '${CONDA_ENV}'。请先选择「完整构建」安装依赖 / run the \"Full build\" mode first to install dependencies."
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
  [[ "$(python -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null)" == "3.10" ]]
}

conda_import_ok() {
  ( activate_conda >/dev/null 2>&1 && python -c "import $1" >/dev/null 2>&1 )
}

ensure_sudo() {
  if ! sudo -n true 2>/dev/null; then
    say "  ${C_DIM}需要 root 权限, 请输入密码 / root privileges required, enter your password:${C_RST}"
    sudo -v || die "无法获取 sudo 权限 / cannot obtain sudo (apt 安装与 rosdep init 需要 root / apt install and rosdep init need root)。"
  fi
}

# --------------------------------------------------------------- 前置检查 ---
MISSING=()          # 阻塞项 / blocking items, each says how to fix it
NOTES=()            # 非阻塞提示 / non-blocking notes

preflight() {
  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
      MISSING+=("当前系统是 ${PRETTY_NAME:-未知}, ROS 2 ${ROS_DISTRO} 官方只支持 Ubuntu 22.04 (jammy)。
       This system is ${PRETTY_NAME:-unknown}; ROS 2 ${ROS_DISTRO} officially supports Ubuntu 22.04 (jammy) only.")
    fi
  fi

  if [[ -z "${ROS_DISTRO}" || ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    MISSING+=("找不到 ROS 2 / ROS 2 not found (/opt/ros/<distro>/setup.bash)。安装 / Install (Ubuntu 22.04, 约 / about 2GB):

        sudo apt install -y software-properties-common curl
        sudo add-apt-repository universe -y && sudo apt update
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo \$UBUNTU_CODENAME) main\" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
        sudo apt update && sudo apt install -y ros-humble-desktop ros-dev-tools")
  fi

  # colcon 构建期间需要 Conda 环境提供 lark / empy, 所以它是硬前置
  # The colcon build needs lark / empy from the Conda env, so Conda is required
  if [[ -z "${CONDA_BASE}" || ! -f "${CONDA_BASE}/etc/profile.d/conda.sh" ]]; then
    MISSING+=("找不到 Conda / Conda not found。安装 / Install Miniconda (约 / about 100MB):

        curl -fsSL https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -o /tmp/miniconda.sh
        bash /tmp/miniconda.sh -b -p \"\${HOME}/miniconda3\"
        \"\${HOME}/miniconda3/bin/conda\" init bash")
  fi

  if [[ -z "${SDK_DIR}" ]]; then
    MISSING+=("在 / under ${WORKSPACE_DIR} 找不到 UMA_SDK / UMA_SDK not found —— 两个仓库要并列放在工作区的 src/ 下 / both repositories must sit side by side under src/:

        mkdir -p ${WORKSPACE_DIR}/src && cd ${WORKSPACE_DIR}/src
        git clone https://github.com/agilexrobotics/UMA_Control.git
        git clone https://github.com/agilexrobotics/UMA_SDK.git")
  fi

  if [[ ! -x "${PYTHON_BIN}" ]]; then
    MISSING+=("找不到 / not found: ${PYTHON_BIN} (消息生成需要它 / needed for message generation): sudo apt install -y python3")
  fi

  local free_gb
  free_gb=$(df -BG --output=avail "${WORKSPACE_DIR}" 2>/dev/null | tail -1 | tr -dc '0-9')
  if [[ -n "${free_gb}" ]] && (( free_gb < 5 )); then
    MISSING+=("工作区可用空间仅 / only ${free_gb}GB free in the workspace, 建议预留 5GB 以上 / 5GB or more recommended (meshes, asio headers, install space)。")
  fi

  if command -v curl >/dev/null 2>&1; then
    if ! curl -fsS --max-time 8 -o /dev/null https://github.com 2>/dev/null; then
      NOTES+=("访问 github.com 失败 / cannot reach github.com: pip 安装 pyAgxArm 与 rosdep update 可能会失败 / installing pyAgxArm and rosdep update may fail。")
    fi
  fi

  # ~/.local 的 user site-packages 会盖住 Conda 环境里的同名包, 检测到就提醒一句
  # Warn when ~/.local shadows same-named packages inside the Conda env
  if [[ -f "${CONDA_BASE}/etc/profile.d/conda.sh" ]] && conda_env_exists "${CONDA_ENV}"; then
    local iso plain
    iso="$( (activate_conda >/dev/null 2>&1 && python -c 'import pinocchio; print(pinocchio.__file__)') 2>/dev/null || echo '?')"
    plain="$( (activate_conda >/dev/null 2>&1 && PYTHONNOUSERSITE= python -c 'import pinocchio; print(pinocchio.__file__)') 2>/dev/null || echo '?')"
    if [[ "${iso}" != "${plain}" ]]; then
      NOTES+=("~/.local 的 pip 包会遮住 Conda 环境里的同名包 (如 pinocchio) / pip packages in ~/.local shadow same-named ones in the Conda env (e.g. pinocchio):
         手动跑 python 前先 export PYTHONNOUSERSITE=1, start_*.sh 已自动设置 / export PYTHONNOUSERSITE=1 before running python manually; start_*.sh already does this。")
    fi
  fi
}

# --------------------------------------------------------------- 各阶段 ---
stage_apt() {
  step "系统依赖 / System dependencies (apt)"
  local missing=() p
  for p in "${APT_PACKAGES[@]}"; do
    dpkg -s "${p}" >/dev/null 2>&1 || missing+=("${p}")
  done
  if (( ${#missing[@]} == 0 )); then
    say "  ${C_OK}✓${C_RST} ${#APT_PACKAGES[@]} 个系统依赖均已安装, 跳过 / all ${#APT_PACKAGES[@]} system packages installed, skipping"
    return 0
  fi
  warn "  待安装 / To install (${#missing[@]}/${#APT_PACKAGES[@]}): ${missing[*]}"
  ensure_sudo
  run sudo apt-get update
  run sudo apt-get install -y "${missing[@]}"
}

stage_conda() {
  step "Conda 环境 / Conda environment (${CONDA_ENV})"

  if conda_env_exists "${CONDA_ENV}"; then
    say "  ${C_OK}✓${C_RST} 环境已存在 / environment already exists"
  else
    # shellcheck disable=SC1090,SC1091
    source "${CONDA_BASE}/etc/profile.d/conda.sh"
    run conda create -n "${CONDA_ENV}" "${CONDA_CHANNEL[@]}" python=3.10 -y
  fi

  activate_conda

  if ! env_python_version_ok; then
    die "环境 / environment '${CONDA_ENV}' 的 Python 不是 3.10 / is not Python 3.10 (实际 / actually $(python -V 2>&1))。
      README 要求 python=3.10, 请重建 / README requires python=3.10, please rebuild:
        conda env remove -n ${CONDA_ENV}
      然后重新运行本脚本 / then re-run this script"
  fi

  local bad
  bad="$(env_missing_conda)"
  if [[ -n "${bad}" ]]; then
    warn "  待安装/修正 / To install or fix (conda): ${bad}"
    run conda install -y "${CONDA_CHANNEL[@]}" "${CONDA_PACKAGES[@]}"
  else
    say "  ${C_OK}✓${C_RST} conda 依赖已满足 / conda dependencies satisfied (pinocchio 3.2.0 / casadi 3.6.7 / scipy ...)"
  fi

  bad="$(env_missing_pip "${PIP_PACKAGES[@]}")"
  if [[ -n "${bad}" ]]; then
    warn "  待安装/修正 / To install or fix (pip): ${bad}"
    run python -m pip install "${PIP_PACKAGES[@]}"
  else
    say "  ${C_OK}✓${C_RST} pip 依赖已满足 / pip dependencies satisfied (numpy==2.0.2 / empy==3.3.4 / lark ...)"
  fi

  if python -c "import pyAgxArm" >/dev/null 2>&1; then
    say "  ${C_OK}✓${C_RST} pyAgxArm 已安装 / pyAgxArm installed"
  else
    say "  安装 pyAgxArm (来自 GitHub, 需要外网) / installing pyAgxArm from GitHub (needs internet)"
    run python -m pip install "${PYAGXARM_URL}"
  fi
}

stage_rosdep() {
  step "rosdep 依赖 / rosdep dependencies"
  # shellcheck disable=SC1090,SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"

  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    say "  首次使用, 初始化 rosdep / first run, initializing rosdep"
    ensure_sudo
    run sudo rosdep init
  else
    say "  ${C_OK}✓${C_RST} rosdep 已初始化 / rosdep already initialized"
  fi

  if [[ ! -f "${HOME}/.ros/rosdep/sources.cache/index" ]]; then
    run rosdep update
  else
    say "  ${C_OK}✓${C_RST} rosdep 缓存已存在, 跳过 update / rosdep cache present, skipping update"
  fi

  if rosdep check --from-paths "${CONTROL_DIR}" "${SDK_DIR}" --ignore-src -r; then
    say "  ${C_OK}✓${C_RST} 系统依赖已满足, 跳过 / all system dependencies satisfied, skipping"
  else
    ensure_sudo
    run rosdep install --from-paths "${CONTROL_DIR}" "${SDK_DIR}" --ignore-src -r -y
  fi
}

stage_build() {
  step "colcon 构建 / colcon build"
  activate_conda
  # shellcheck disable=SC1090,SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"

  # colcon 的 build/ install/ log/ 落在当前目录, 必须是工作区根
  # colcon writes build/ install/ log/ into the current directory: the workspace root
  cd "${WORKSPACE_DIR}"
  run colcon build \
    --base-paths "${CONTROL_DIR}" "${SDK_DIR}" \
    --symlink-install \
    --cmake-args "-DPython3_EXECUTABLE=${PYTHON_BIN}" -DBUILD_TESTING=OFF
}

stage_python() {
  step "Python bridge 依赖 / Python bridge dependencies"
  activate_conda

  local bad
  bad="$(env_missing_pip python-can)"
  if [[ -n "${bad}" ]]; then
    run python -m pip install python-can
  else
    say "  ${C_OK}✓${C_RST} python-can 已安装 / python-can installed"
  fi

  local where=""
  where="$(python -c 'import os, uma_sdk; print(os.path.dirname(os.path.abspath(uma_sdk.__file__)))' 2>/dev/null || true)"
  if [[ "${where}" == "${SDK_DIR}/python" ]]; then
    say "  ${C_OK}✓${C_RST} uma_sdk 已以 editable 方式安装到环境 / uma_sdk already installed editable into ${CONDA_ENV}"
  else
    run python -m pip install -e "${SDK_DIR}/python"
  fi
}

stage_verify() {
  step "校验 / Verification"
  local failed=0

  if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    say "  ${C_OK}✓${C_RST} install/setup.bash 已生成 / generated"
  else
    say "  ${C_ERR}✗${C_RST} install/setup.bash 缺失 / missing"
    failed=1
  fi

  local p
  for p in "${EXPECTED_PKGS[@]}"; do
    if [[ ! -d "${WORKSPACE_DIR}/install/${p}" ]]; then
      say "  ${C_ERR}✗${C_RST} install/${p} 缺失 / missing"
      failed=1
    fi
  done
  (( failed )) || say "  ${C_OK}✓${C_RST} ${#EXPECTED_PKGS[@]} 个 ROS 包均已安装到 install/ / all ${#EXPECTED_PKGS[@]} ROS packages installed into install/"

  if conda_import_ok "pinocchio, casadi, scipy, pyAgxArm"; then
    say "  ${C_OK}✓${C_RST} IK 与驱动依赖 / IK and driver dependencies (pinocchio/casadi/scipy/pyAgxArm)"
  else
    say "  ${C_ERR}✗${C_RST} IK 与驱动依赖缺失, 两个 backend 都无法启动 / IK and driver dependencies missing; neither backend can start"
    failed=1
  fi

  if (( SKIP_PYTHON )); then
    say "  ${C_DIM}– 本次未安装 Python bridge 依赖, 跳过该项检查 / Python bridge deps not installed in this run, skipping this check${C_RST}"
  elif conda_import_ok "can, uma_sdk"; then
    say "  ${C_OK}✓${C_RST} Python bridge 依赖 / Python bridge dependencies (python-can/uma_sdk)"
  else
    say "  ${C_ERR}✗${C_RST} Python bridge 依赖缺失 (只影响 Python backend) / Python bridge deps missing (affects the Python backend only)"
    failed=1
  fi

  # 在子 shell 里 source 工作区, 确认 colcon 真能认出这些包
  # Source the workspace in a subshell to confirm colcon really sees the packages
  if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    local n
    n="$( cd "${WORKSPACE_DIR}" \
          && source "/opt/ros/${ROS_DISTRO}/setup.bash" >/dev/null 2>&1 \
          && source "${WORKSPACE_DIR}/install/setup.bash" >/dev/null 2>&1 \
          && colcon list --base-paths "${CONTROL_DIR}" "${SDK_DIR}" 2>/dev/null | wc -l )" || n=0
    if [[ "${n:-0}" -ge ${#EXPECTED_PKGS[@]} ]]; then
      say "  ${C_OK}✓${C_RST} colcon 识别到 ${n} 个包 / colcon found ${n} packages"
    else
      say "  ${C_ERR}✗${C_RST} colcon 只识别到 ${n:-0} 个包 / colcon found only ${n:-0} packages (预期至少 / expected at least ${#EXPECTED_PKGS[@]})"
      failed=1
    fi
  fi

  return "${failed}"
}

next_steps() {
  local rel_ctl="${CONTROL_DIR#"${WORKSPACE_DIR}/"}"
  say ""
  say "${C_BLD}下一步 / Next steps:${C_RST}"
  say "  1) 配置 CAN / Configure CAN —— 先按 find_all_can_port.sh 的输出改 can_config.sh 里的 bus-info"
  say "     update bus-info in can_config.sh from the find_all_can_port.sh output:"
  say "       cd ${WORKSPACE_DIR}"
  say "       bash ${rel_ctl}/uma_bringup/scripts/find_all_can_port.sh"
  say "       bash ${rel_ctl}/uma_bringup/scripts/can_config.sh"
  say "  2) 启动遥操 / Start teleoperation (脚本会自动激活 / activates the ${CONDA_ENV} environment):"
  say "       bash ${rel_ctl}/uma_bringup/scripts/start_single_piper_cpp.sh   # Piper 单臂 / single arm"
  say "       bash ${rel_ctl}/uma_bringup/scripts/start_dual_nero_cpp.sh     # Nero 双臂 / dual arm"
  say "       Python backend 用同名的 / use the same-named *_python.sh for the Python backend"
  say "  手动使用工作区 / Use the workspace manually:"
  say "       cd ${WORKSPACE_DIR} && source install/setup.bash"
  if [[ -n "${LOG_FILE}" ]]; then say "  构建日志 / Build log: ${LOG_FILE}"; fi
  return 0
}

# ------------------------------------------------------------------ 主流程 ---
main() {
  START_TS=$(date +%s)

  # ---- 环境探测 / Environment detection -----------------------------------
  locate_workspace
  CONDA_BASE="$(detect_conda_base || true)"

  if [[ -z "${ROS_DISTRO}" ]]; then
    local -a distros=()
    mapfile -t distros < <(detect_ros_distros)
    case ${#distros[@]} in
      0) : ;;
      1) ROS_DISTRO="${distros[0]}" ;;
      *)
        local d
        for d in "${distros[@]}"; do [[ "${d}" == "humble" ]] && ROS_DISTRO="humble"; done
        if [[ -z "${ROS_DISTRO}" ]]; then
          local -a args=()
          for d in "${distros[@]}"; do args+=("${d}" "ROS 2 ${d}"); done
          choose_one "检测到多个 ROS 2 发行版, 请选择 / Multiple ROS 2 distributions found, select one" \
                     "${distros[0]}" "${args[@]}"
          ROS_DISTRO="${CHOICE}"
        fi
        ;;
    esac
  fi

  say "${C_BLD}uma 构建脚本 / build script${C_RST}  ${C_DIM}(全部通过交互选择 / everything is chosen interactively)${C_RST}"
  say "  工作区 / workspace ${WORKSPACE_DIR}"
  say "  SDK                ${SDK_DIR:-未找到 / not found}"
  say "  ROS                ${ROS_DISTRO:-未找到 / not found}"
  say "  Conda              ${CONDA_ENV} @ ${CONDA_BASE:-未找到 / not found}"

  # ---- 选择要做什么 / Choose what to do -----------------------------------
  choose_operation
  choose_backend

  # ---- 前置检查 / Preflight ----------------------------------------------
  preflight
  if (( ${#MISSING[@]} )); then
    warn "${C_ERR}前置条件不满足, 未做任何改动。 / Prerequisites not met; nothing was changed.${C_RST}"
    warn ""
    local m
    for m in "${MISSING[@]}"; do warn "  ${C_ERR}✗${C_RST} ${m}"; warn ""; done
    warn "补齐后重新运行本脚本即可。 / Fix the items above and re-run this script."
    exit 1
  fi
  local k
  for k in "${NOTES[@]}"; do warn "${C_WARN}!${C_RST} ${k}"; done

  # ---- 清理 / Cleanup -----------------------------------------------------
  if (( DO_CLEAN )); then
    say ""
    say "${C_WARN}重新构建 / Rebuild: 删除 / removing ${WORKSPACE_DIR}/{build,install,log}${C_RST}"
    rm -rf "${WORKSPACE_DIR}/build" "${WORKSPACE_DIR}/install" "${WORKSPACE_DIR}/log"
  fi

  mkdir -p "${WORKSPACE_DIR}/log"
  LOG_FILE="${WORKSPACE_DIR}/log/build_$(date +%Y%m%d_%H%M%S).log"
  : >"${LOG_FILE}"
  trap on_error ERR

  say ""
  say "${C_BLD}开始 / Starting: ${OPERATION_LABEL}${C_RST}"
  if [[ "${OPERATION}" == "full" ]]; then
    say "  bridge ${BACKEND}"
    if [[ "${BACKEND}" == "cpp" ]]; then
      say "  ${C_DIM}backend=cpp: 跳过 python-can 与 Python SDK; Conda 环境仍会安装, 因为 C++ bridge 也走 Python IK${C_RST}"
      say "  ${C_DIM}             skips python-can and the Python SDK; the Conda env is still installed because the C++ bridge also uses Python IK${C_RST}"
    fi
  fi
  say "  日志 / Log ${LOG_FILE}"

  # ---- 组装阶段 / Assemble stages ----------------------------------------
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
  next_steps
  if (( verify_rc != 0 )); then
    warn ""
    warn "${C_ERR}构建结束, 但校验未全部通过 (见上方 ✗ 项)。 / Build finished, but verification did not fully pass (see the ✗ items above).${C_RST}"
    exit 1
  fi
  say ""
  say "${C_OK}${C_BLD}完成。 / Done.${C_RST}"
}

main
