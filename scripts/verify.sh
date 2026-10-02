#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."

command -v apptainer >/dev/null 2>&1 || {
  echo "FEHLER: Apptainer fehlt." >&2
  exit 1
}
[[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]] || {
  echo "FEHLER: Dieses Paket benoetigt Linux x86_64 (direkt oder via WSL2/VM)." >&2
  exit 1
}

bash -n scripts/*.sh run.sh
bash scripts/container.sh python -m pytest -q tests
bash scripts/container.sh bash scripts/build_gazebo_service_client.sh
bash scripts/container.sh python scripts/benchmark.py \
  --dry-run --config configs/benchmark.yaml >/dev/null
bash scripts/container.sh python -c \
  'import cv2, numpy, torch, ultralytics, yaml; from ultralytics import YOLO; m=YOLO("models/detector/shahed_full_best.pt"); assert "shahed" in m.names.values(); assert 4 in m.model.stride.tolist(); print("Python-Laufzeit OK:", torch.__version__, ultralytics.__version__)'
(cd models && sha256sum -c CHECKSUMS.sha256)
(cd runtime && sha256sum -c CHECKSUMS.sha256)
echo "Portable Pruefung erfolgreich."
