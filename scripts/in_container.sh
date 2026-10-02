#!/usr/bin/env bash
set -eo pipefail
cd /workspace
source /opt/ros/jazzy/setup.bash

python_path="/workspace/runtime/python:/workspace/scripts"
if [[ "${SIM_USE_GPU:-0}" == 1 ]]; then
  python_path="/workspace/runtime/python-gpu:$python_path"
fi
export PYTHONPATH="$python_path${PYTHONPATH:+:$PYTHONPATH}"
export YOLO_CONFIG_DIR=/workspace/.cache/ultralytics
export TORCH_HOME=/workspace/.cache/torch
export ROS_HOME=/workspace/.cache/ros
export ROS_LOG_DIR=/workspace/log/ros
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OMP_NUM_THREADS="${SIM_CPU_THREADS:-2}"
export OPENBLAS_NUM_THREADS="${SIM_BLAS_THREADS:-2}"
export GZ_SIM_RESOURCE_PATH="/workspace/models:${GZ_SIM_RESOURCE_PATH:-}"

if [[ "${SIM_USE_GPU:-0}" == 1 ]]; then
  unset LIBGL_ALWAYS_SOFTWARE EGL_PLATFORM
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
else
  export LIBGL_ALWAYS_SOFTWARE=1
  export EGL_PLATFORM=surfaceless
  export GALLIUM_DRIVER=llvmpipe
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json
fi

if [[ "${1:-}" == python ]]; then
  shift
  set -- python3 "$@"
fi
exec "$@"
