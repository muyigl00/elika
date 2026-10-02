#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"

command -v apptainer >/dev/null 2>&1 || {
  echo "Apptainer fehlt. Siehe README.md (Abschnitt 'Voraussetzungen')." >&2
  exit 1
}

container_home="$PROJECT_DIR/.cache/home"
mkdir -p "$container_home" "$PROJECT_DIR/.cache/apptainer-work"

for required_path in \
  "$PROJECT_DIR/runtime/ros-jazzy.sif" \
  "$PROJECT_DIR/runtime/python" \
  "$PROJECT_DIR/models/detector/shahed_full_best.pt"; do
  [[ -e "$required_path" ]] || {
    echo "Portable Laufzeitdatei fehlt: $required_path" >&2
    exit 1
  }
done

gpu_args=()
if [[ "${SIM_USE_GPU:-0}" == 1 ]]; then
  [[ -d "$PROJECT_DIR/runtime/python-gpu/torch" ]] || {
    echo "GPU-Pakete fehlen. Einmalig ausfuehren: bash scripts/setup_gpu.sh" >&2
    exit 1
  }
  command -v nvidia-smi >/dev/null 2>&1 || {
    echo "nvidia-smi fehlt; der NVIDIA-Treiber ist nicht sichtbar." >&2
    exit 1
  }
  gpu_backend=--nv
  if grep -qi microsoft /proc/version 2>/dev/null; then
    gpu_backend=--nvccli
  fi
  gpu_args=("$gpu_backend" --env SIM_USE_GPU=1 --env "CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-0}")
fi

exec apptainer exec "${gpu_args[@]}" --cleanenv --contain --no-mount hostfs,cwd \
  --home "$container_home:/home/benchmark" \
  --workdir "$PROJECT_DIR/.cache/apptainer-work" \
  --bind "$PROJECT_DIR:/workspace" \
  --pwd /workspace \
  "$PROJECT_DIR/runtime/ros-jazzy.sif" \
  bash --noprofile --norc /workspace/scripts/in_container.sh "$@"
