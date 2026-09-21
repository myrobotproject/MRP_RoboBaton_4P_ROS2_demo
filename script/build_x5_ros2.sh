#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PKG_NAME="robobaton_4p_ros2_demo"

X5_CROSS_ROOT="${X5_CROSS_ROOT:-}"
CROSS_ENV_SCRIPT="${CROSS_ENV_SCRIPT:-}"
TOOLCHAIN_FILE="${TOOLCHAIN_FILE:-}"
BUILD_ROOT="${BUILD_ROOT:-${PKG_DIR}/1.ros2_build}"
BUILD_BASE="${BUILD_BASE:-}"
INSTALL_BASE="${INSTALL_BASE:-}"
LOG_BASE="${LOG_BASE:-}"
ROBOBATON_LIB_DIR="${ROBOBATON_LIB_DIR:-${PKG_DIR}/lib}"
CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
PARALLEL_WORKERS="${PARALLEL_WORKERS:-}"
CLEAN=0
EXTRA_COLCON_ARGS=()

usage() {
  cat <<'USAGE'
Usage:
  source /opt/ros/humble/setup.bash
  script/build_x5_ros2.sh [options] [-- extra colcon args...]

Options:
  --clean                         Remove selected build/install/log roots before building.
  --cross-root <path>             X5 cross bundle root. Required unless X5_CROSS_ROOT is set.
  --cross-env <path>              Cross environment script.
  --toolchain-file <path>         X5 CMake toolchain file.
  --robobaton-lib-dir <path>      Existing in-package ABI-v2 producer directory, default ./lib.
  --primary-lib-dir <path>        Backward-compatible alias for --robobaton-lib-dir.
  --build-root <path>             Parent for build/install/log, default ./1.ros2_build.
  --build-base <path>             Colcon build base.
  --install-base <path>           Colcon merged install base.
  --log-base <path>               Colcon log base.
  --cmake-build-type <type>       CMake build type, default Release.
  --parallel-workers <n>          Forward to colcon.
  -h, --help                      Show this help.

No module-root variables are required. Upstream producer builds must synchronize
libicm42688.so.2.1.0/.2/unversioned and libsc132.so.2.0.1/.2/unversioned into ./lib.
USAGE
}

resolve_project_path() {
  local path="$1"
  if [[ "${path}" = /* ]]; then printf '%s\n' "${path}"; else printf '%s\n' "${PKG_DIR}/${path}"; fi
}
resolve_existing_dir() {
  local path="$1"
  [[ -d "${path}" ]] || { echo "Missing directory: ${path}" >&2; exit 2; }
  cd "${path}" && pwd -P
}
resolve_existing_file() {
  local path="$1"
  [[ -f "${path}" ]] || { echo "Missing file: ${path}" >&2; exit 2; }
  cd "$(dirname "${path}")" && printf '%s/%s\n' "$(pwd -P)" "$(basename "${path}")"
}
resolve_output_dir() {
  local path="$1"
  mkdir -p "${path}"
  cd "${path}" && pwd -P
}
refuse_unsafe_clean_dir() {
  local path="$1"
  if [[ "${path}" == "/" || "${path}" == "${PKG_DIR}" || "${path}" == "${SCRIPT_DIR}" ]]; then
    echo "Refusing unsafe clean path: ${path}" >&2
    exit 2
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --clean) CLEAN=1; shift ;;
    --cross-root) X5_CROSS_ROOT="$2"; shift 2 ;;
    --cross-env) CROSS_ENV_SCRIPT="$2"; shift 2 ;;
    --toolchain-file) TOOLCHAIN_FILE="$2"; shift 2 ;;
    --robobaton-lib-dir|--primary-lib-dir) ROBOBATON_LIB_DIR="$2"; shift 2 ;;
    --build-root) BUILD_ROOT="$2"; shift 2 ;;
    --install-base) INSTALL_BASE="$2"; shift 2 ;;
    --log-base) LOG_BASE="$2"; shift 2 ;;
    --build-base) BUILD_BASE="$2"; shift 2 ;;
    --cmake-build-type) CMAKE_BUILD_TYPE="$2"; shift 2 ;;
    --parallel-workers) PARALLEL_WORKERS="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    --) shift; EXTRA_COLCON_ARGS+=("$@"); break ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "${X5_CROSS_ROOT}" ]]; then
  echo "Missing X5 cross bundle root; pass --cross-root <path> or set X5_CROSS_ROOT." >&2
  exit 2
fi

X5_CROSS_ROOT="$(resolve_project_path "${X5_CROSS_ROOT}")"
CROSS_ENV_SCRIPT="$(resolve_project_path "${CROSS_ENV_SCRIPT:-${X5_CROSS_ROOT}/scripts/setup_x5_cross_env.sh}")"
TOOLCHAIN_FILE="$(resolve_project_path "${TOOLCHAIN_FILE:-${X5_CROSS_ROOT}/toolchain/aarch64_x5_host_toolchain.cmake}")"
BUILD_ROOT="$(resolve_project_path "${BUILD_ROOT}")"
ROBOBATON_LIB_DIR="$(resolve_project_path "${ROBOBATON_LIB_DIR}")"
BUILD_BASE="$(resolve_project_path "${BUILD_BASE:-${BUILD_ROOT}/build}")"
INSTALL_BASE="$(resolve_project_path "${INSTALL_BASE:-${BUILD_ROOT}/install}")"
LOG_BASE="$(resolve_project_path "${LOG_BASE:-${BUILD_ROOT}/log}")"

X5_CROSS_ROOT="$(resolve_existing_dir "${X5_CROSS_ROOT}")"
CROSS_ENV_SCRIPT="$(resolve_existing_file "${CROSS_ENV_SCRIPT}")"
TOOLCHAIN_FILE="$(resolve_existing_file "${TOOLCHAIN_FILE}")"
ROBOBATON_LIB_DIR="$(resolve_existing_dir "${ROBOBATON_LIB_DIR}")"

for required_file in \
  "${ROBOBATON_LIB_DIR}/libicm42688.so.2.1.0" \
  "${ROBOBATON_LIB_DIR}/libicm42688.so.2" \
  "${ROBOBATON_LIB_DIR}/libicm42688.so" \
  "${ROBOBATON_LIB_DIR}/libsc132.so.2.0.1" \
  "${ROBOBATON_LIB_DIR}/libsc132.so.2" \
  "${ROBOBATON_LIB_DIR}/libsc132.so"; do
  [[ -e "${required_file}" ]] || { echo "Missing required producer artifact: ${required_file}" >&2; exit 2; }
done
command -v colcon >/dev/null 2>&1 || { echo "Missing command: colcon" >&2; exit 2; }
python3 -c 'import ament_package' >/dev/null 2>&1 || {
  echo "Host ROS environment missing; run: source /opt/ros/humble/setup.bash" >&2
  exit 2
}

if [[ "${CLEAN}" -eq 1 ]]; then
  refuse_unsafe_clean_dir "${BUILD_BASE}"
  refuse_unsafe_clean_dir "${INSTALL_BASE}"
  refuse_unsafe_clean_dir "${LOG_BASE}"
  rm -rf "${BUILD_BASE}" "${INSTALL_BASE}" "${LOG_BASE}"
fi
BUILD_BASE="$(resolve_output_dir "${BUILD_BASE}")"
INSTALL_BASE="$(resolve_output_dir "${INSTALL_BASE}")"
LOG_BASE="$(resolve_output_dir "${LOG_BASE}")"

# ROS and cross setup scripts intentionally read unset variables.
set +u
# shellcheck disable=SC1090
source "${CROSS_ENV_SCRIPT}"
set -u

colcon_args=(
  --log-base "${LOG_BASE}"
  build
  --base-paths "${PKG_DIR}"
  --packages-select "${PKG_NAME}"
  --build-base "${BUILD_BASE}"
  --install-base "${INSTALL_BASE}"
  --merge-install
  --cmake-force-configure)
if [[ -n "${PARALLEL_WORKERS}" ]]; then colcon_args+=(--parallel-workers "${PARALLEL_WORKERS}"); fi
if [[ "${#EXTRA_COLCON_ARGS[@]}" -gt 0 ]]; then colcon_args+=("${EXTRA_COLCON_ARGS[@]}"); fi
colcon_args+=(
  --cmake-args
  -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}"
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
  -DROBOBATON_LIB_DIR="${ROBOBATON_LIB_DIR}"
  -DX5_BUILD_AMENT_PREFIX_PATH="${EXTRA_PREFIX}:${TARGET_ROS_PREFIX}"
  -DCMAKE_PREFIX_PATH="${EXTRA_PREFIX};${TARGET_ROS_PREFIX};${SYSROOT_ROOT}/usr;${SYSROOT_ROOT}/usr_x5"
  -DBUILD_TESTING=OFF)

printf '[build_x5_ros2] package=%s\n' "${PKG_DIR}"
printf '[build_x5_ros2] install=%s\n' "${INSTALL_BASE}"
printf '[build_x5_ros2] toolchain=%s\n' "${TOOLCHAIN_FILE}"
printf '[build_x5_ros2] target_ros=%s\n' "${TARGET_ROS_PREFIX:-unset}"
printf '[build_x5_ros2] robobaton_lib=%s\n' "${ROBOBATON_LIB_DIR}"
env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH colcon "${colcon_args[@]}"

node_path="${INSTALL_BASE}/lib/${PKG_NAME}/robobaton_sensors_node"
[[ -x "${node_path}" ]] || { echo "Build finished but expected node is missing: ${node_path}" >&2; exit 3; }
file "${node_path}"
python3 "${PKG_DIR}/script/verify_install.py" --write-manifest "${INSTALL_BASE}"
chmod 644 "${INSTALL_BASE}/lib/${PKG_NAME}/abi_manifest.sha256"
printf '[build_x5_ros2] PASS install=%s\n' "${INSTALL_BASE}"
printf '[build_x5_ros2] deploy/source on target: source %s/robobaton_ros2_env.bash\n' "${INSTALL_BASE}"
printf '[build_x5_ros2] one-shot command wrapper: %s/robobaton_ros2_env.bash ros2 topic list --no-daemon --include-hidden-topics\n' "${INSTALL_BASE}"
