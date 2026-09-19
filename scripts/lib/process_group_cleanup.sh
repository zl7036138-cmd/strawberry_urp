#!/usr/bin/env bash
# Bounded process-group cleanup helpers for ROS/Gazebo development runners.
#
# This file is sourced by runners and intentionally has no top-level side
# effects. Group liveness is read from `ps` instead of `kill -0 -PGID`; the
# latter can report false negatives after a launch-group leader exits while
# Gazebo descendants remain alive under WSL.

process_group_alive() {
  local target_pgid="$1"
  [[ "${target_pgid}" =~ ^[1-9][0-9]*$ ]] || return 2
  ps -eo pgid=,stat= | awk -v target="${target_pgid}" '
    $1 == target && $2 !~ /^Z/ { found = 1 }
    END { exit(found ? 0 : 1) }
  '
}

process_group_pids() {
  local target_pgid="$1"
  [[ "${target_pgid}" =~ ^[1-9][0-9]*$ ]] || return 2
  ps -eo pid=,pgid=,stat= | awk -v target="${target_pgid}" '
    $2 == target && $3 !~ /^Z/ { print $1 }
  '
}

process_group_stably_absent() {
  local target_pgid="$1"
  local consecutive_checks="${2:-8}"
  [[ "${consecutive_checks}" =~ ^[1-9][0-9]*$ ]] || return 2
  for ((index = 0; index < consecutive_checks; index += 1)); do
    process_group_alive "${target_pgid}" && return 1
    sleep 0.25
  done
  return 0
}

terminate_process_group() {
  local target_pgid="$1"
  local group_leader_pid="${2:-}"
  local term_checks="${3:-80}"
  local kill_checks="${4:-20}"

  [[ "${target_pgid}" =~ ^[1-9][0-9]*$ ]] || return 2
  [[ "${term_checks}" =~ ^[1-9][0-9]*$ ]] || return 2
  [[ "${kill_checks}" =~ ^[1-9][0-9]*$ ]] || return 2
  if ! process_group_alive "${target_pgid}" && \
     process_group_stably_absent "${target_pgid}" 4; then
    return 0
  fi

  kill -TERM -- "-${target_pgid}" 2>/dev/null || true
  for ((index = 0; index < term_checks; index += 1)); do
    if ! process_group_alive "${target_pgid}" && \
       process_group_stably_absent "${target_pgid}" 4; then
      if [[ -n "${group_leader_pid}" ]]; then
        wait "${group_leader_pid}" 2>/dev/null || true
      fi
      return 0
    fi
    sleep 0.25
  done

  kill -KILL -- "-${target_pgid}" 2>/dev/null || true
  for ((index = 0; index < kill_checks; index += 1)); do
    if ! process_group_alive "${target_pgid}" && \
       process_group_stably_absent "${target_pgid}" 4; then
      if [[ -n "${group_leader_pid}" ]]; then
        wait "${group_leader_pid}" 2>/dev/null || true
      fi
      return 0
    fi
    sleep 0.10
  done
  if [[ -n "${group_leader_pid}" ]]; then
    wait "${group_leader_pid}" 2>/dev/null || true
  fi
  # A launch supervisor can briefly disappear from `ps` while descendants are
  # being reaped. Recheck after a short settle period and kill any surviving
  # non-zombie members individually before declaring cleanup complete.
  sleep 0.50
  local remaining_pids
  remaining_pids="$(process_group_pids "${target_pgid}")"
  if [[ -n "${remaining_pids}" ]]; then
    # shellcheck disable=SC2086
    kill -KILL ${remaining_pids} 2>/dev/null || true
    sleep 0.50
  fi
  process_group_stably_absent "${target_pgid}" 8
}
