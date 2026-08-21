#!/usr/bin/env bash

set -uo pipefail

exit_code=0
ros2 topic pub --once \
  /ego_racecar/control std_msgs/msg/String "{data: start}" &
ego_publisher_pid=$!
ros2 topic pub --once \
  /opp_racecar/control std_msgs/msg/String "{data: start}" &
opponent_publisher_pid=$!

wait "$ego_publisher_pid" || exit_code=$?
wait "$opponent_publisher_pid" || exit_code=$?

exit "$exit_code"
