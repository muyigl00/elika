#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."

config=configs/benchmark.yaml
if [[ $# -gt 0 && "$1" != -* ]]; then
  config="$1"
  shift
fi
[[ "$config" != /* && -f "$config" ]] || {
  echo "Konfiguration muss eine vorhandene relative Datei sein: $config" >&2
  exit 1
}

mkdir -p log recordings
run_id="$(date -u +%Y%m%dT%H%M%SZ)_$$"
output="recordings/run_$run_id"

if [[ "${SHAHED_USE_GPU:-0}" == 1 ]]; then
  export SIM_USE_GPU=1
  device="${SHAHED_DEVICE:-0}"
else
  export SIM_USE_GPU=0
  device="${SHAHED_DEVICE:-cpu}"
fi

echo "Modus: $([[ "$SIM_USE_GPU" == 1 ]] && echo NVIDIA-GPU || echo CPU)"
echo "Ausgabe: $output"
exec bash scripts/container.sh python scripts/benchmark.py \
  --config "/workspace/$config" \
  --device "$device" \
  --output "/workspace/$output" \
  "$@"
