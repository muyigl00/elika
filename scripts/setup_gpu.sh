#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."

command -v nvidia-smi >/dev/null 2>&1 || {
  echo "nvidia-smi fehlt. Zuerst einen passenden NVIDIA-Treiber installieren." >&2
  exit 1
}

mkdir -p runtime/python-gpu
echo "Installiere die offiziellen PyTorch-CUDA-12.8-Pakete (mehrere GB Download) ..."
SIM_USE_GPU=0 bash scripts/container.sh python -m pip install \
  --upgrade \
  --target /workspace/runtime/python-gpu \
  torch torchvision \
  --index-url https://download.pytorch.org/whl/cu128

SIM_USE_GPU=1 bash scripts/container.sh python -c \
  'import torch; assert torch.cuda.is_available(), "CUDA ist fuer PyTorch nicht verfuegbar"; print(torch.__version__, torch.cuda.get_device_name(0))'
echo "GPU-Modus ist eingerichtet. Start: SHAHED_USE_GPU=1 bash run.sh --smoke"
