#!/usr/bin/env bash

robobaton_ros2_env_is_sourced() {
  [[ "${BASH_SOURCE[0]}" != "$0" ]]
}

robobaton_ros2_env_script_dir() {
  local source_path="${BASH_SOURCE[0]}"
  builtin cd "$(dirname "${source_path}")" >/dev/null && pwd -P
}

robobaton_ros2_env_load() {
  local prefix="${ROBOBATON_ROS2_INSTALL_PREFIX:-}"
  local underlay="${ROBOBATON_ROS_UNDERLAY-/opt/ros/humble/setup.bash}"
  local profile

  if [[ -z "${prefix}" ]]; then
    prefix="$(robobaton_ros2_env_script_dir)" || return 2
  fi
  if [[ ! -f "${prefix}/setup.bash" ]]; then
    echo "missing ROS2 install setup: ${prefix}/setup.bash" >&2
    return 2
  fi

  if [[ -n "${underlay}" ]]; then
    if [[ ! -f "${underlay}" ]]; then
      echo "missing ROS2 underlay setup: ${underlay}" >&2
      echo "override with ROBOBATON_ROS_UNDERLAY=<path> or set it empty to skip" >&2
      return 2
    fi
    # ROS setup scripts take effect only when sourced.
    # shellcheck disable=SC1090
    source "${underlay}" || return
  fi

  # Keep the overlay setup relocatable and install package-level Fast DDS and library-path hooks.
  # shellcheck disable=SC1090
  source "${prefix}/setup.bash" || return
  export ROBOBATON_ROS2_INSTALL_PREFIX="${prefix}"

  profile="${prefix}/share/robobaton_4p_ros2_demo/config/fastdds/robobaton_shm.xml"
  if [[ ! -f "${profile}" ]]; then
    echo "missing FastDDS profile: ${profile}" >&2
    return 2
  fi
  if [[ -z "${FASTDDS_DEFAULT_PROFILES_FILE:-}" ]]; then
    export FASTDDS_DEFAULT_PROFILES_FILE="${profile}"
  fi
  if [[ ! -f "${FASTDDS_DEFAULT_PROFILES_FILE}" ]]; then
    echo "invalid FASTDDS_DEFAULT_PROFILES_FILE: ${FASTDDS_DEFAULT_PROFILES_FILE}" >&2
    return 2
  fi
  if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"
  fi
  export RCUTILS_LOGGING_BUFFERED_STREAM="${RCUTILS_LOGGING_BUFFERED_STREAM:-0}"
}

robobaton_ros2_shm_status() {
  df -h /dev/shm
}

robobaton_ros2_list_topics() {
  ros2 topic list --no-daemon --include-hidden-topics
  ros2 node list --no-daemon
}

robobaton_ros2_restart_daemon() {
  ros2 daemon stop
  ros2 daemon start
}

robobaton_ros2_clean_stale_shm() {
  if pgrep -x robobaton_sensors_node >/dev/null 2>&1 || \
     pgrep -f '[r]os2 launch' >/dev/null 2>&1 || \
     pgrep -f '[r]os2 run' >/dev/null 2>&1; then
    echo "still found ROS2 runtime processes; stop them before cleaning /dev/shm" >&2
    return 2
  fi
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
}

robobaton_ros2_env_check() {
  printf 'ROBOBATON_ROS2_INSTALL_PREFIX=%s\n' "${ROBOBATON_ROS2_INSTALL_PREFIX:-}"
  printf 'FASTDDS_DEFAULT_PROFILES_FILE=%s\n' "${FASTDDS_DEFAULT_PROFILES_FILE:-}"
  printf 'FASTRTPS_DEFAULT_PROFILES_FILE=%s\n' "${FASTRTPS_DEFAULT_PROFILES_FILE:-}"
  printf 'RCUTILS_LOGGING_BUFFERED_STREAM=%s\n' "${RCUTILS_LOGGING_BUFFERED_STREAM:-}"
  robobaton_ros2_shm_status
}

robobaton_ros2_env_help() {
  cat <<'USAGE'
Usage:
  source /root/ros2_demo/install/robobaton_ros2_env.bash
  /root/ros2_demo/install/robobaton_ros2_env.bash [command...]
  /root/ros2_demo/install/robobaton_ros2_env.bash --check
  /root/ros2_demo/install/robobaton_ros2_env.bash --list-topics
  /root/ros2_demo/install/robobaton_ros2_env.bash --restart-daemon
  /root/ros2_demo/install/robobaton_ros2_env.bash --clean-shm

Environment overrides:
  ROBOBATON_ROS_UNDERLAY=/opt/ros/humble/setup.bash
  ROBOBATON_ROS2_INSTALL_PREFIX=/root/ros2_demo/install
  RCUTILS_LOGGING_BUFFERED_STREAM=0
USAGE
}

robobaton_ros2_env_run_command() {
  if robobaton_ros2_env_is_sourced; then
    "$@"
  else
    exec "$@"
  fi
}

robobaton_ros2_env_main() {
  case "${1:-}" in
    -h|--help)
      robobaton_ros2_env_help
      return 0
      ;;
  esac

  robobaton_ros2_env_load || return $?

  case "${1:-}" in
    "")
      robobaton_ros2_env_check
      if [[ -t 0 && -t 1 ]]; then
        exec "${SHELL:-/bin/bash}" -i
      fi
      ;;
    --check)
      robobaton_ros2_env_check
      ;;
    --list-topics)
      robobaton_ros2_list_topics
      ;;
    --restart-daemon)
      robobaton_ros2_restart_daemon
      ;;
    --clean-shm)
      robobaton_ros2_clean_stale_shm
      ;;
    --)
      shift
      if [[ $# -eq 0 ]]; then
        echo "missing command after --" >&2
        return 2
      fi
      robobaton_ros2_env_run_command "$@"
      ;;
    *)
      robobaton_ros2_env_run_command "$@"
      ;;
  esac
}

if robobaton_ros2_env_is_sourced; then
  robobaton_ros2_env_load
  return $?
fi

robobaton_ros2_env_main "$@"
