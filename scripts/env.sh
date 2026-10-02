#!/usr/bin/env bash
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PROJECT_DIR
export XDG_CACHE_HOME="$PROJECT_DIR/.cache"
export XDG_CONFIG_HOME="$PROJECT_DIR/.cache/config"
export XDG_DATA_HOME="$PROJECT_DIR/.cache/data"
export YOLO_CONFIG_DIR="$PROJECT_DIR/.cache/ultralytics"
export ROS_HOME="$PROJECT_DIR/.cache/ros"
export ROS_LOG_DIR="$PROJECT_DIR/log/ros"
export GZ_HOMEDIR="$PROJECT_DIR/.cache/gazebo"
export APPTAINER_CACHEDIR="$PROJECT_DIR/.cache/apptainer"
export APPTAINER_TMPDIR="$PROJECT_DIR/.cache/tmp"
export TMPDIR="$PROJECT_DIR/.cache/tmp"
mkdir -p "$TMPDIR" "$YOLO_CONFIG_DIR" "$ROS_LOG_DIR"

