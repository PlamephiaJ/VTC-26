#!/usr/bin/env bash

set -euo pipefail

vehicle="${1:-all}"
case "$vehicle" in
  all)
    control_topic=/rrt/control
    matching_subscriptions=2
    ;;
  1|ego|ego_racecar)
    control_topic=/ego_racecar/control
    matching_subscriptions=1
    ;;
  2|opp|opp_racecar)
    control_topic=/opp_racecar/control
    matching_subscriptions=1
    ;;
  *)
    echo "Usage: $0 [all|1|2|ego|opp]" >&2
    exit 2
    ;;
esac

ros2 topic pub --once \
  --wait-matching-subscriptions "$matching_subscriptions" \
  "$control_topic" std_msgs/msg/String "{data: start}"
