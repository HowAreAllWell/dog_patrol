#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"

VENV_DIR="${VENV_DIR:-${HOME}/venvs/vision-demo-tools}"
MODEL_DIR="${MODEL_DIR:-${WS_DIR}/assets/models/upstream}"
MODEL_PATH="${MODEL_PATH:-${MODEL_DIR}/yolo26n.pt}"
ENGINE_PATH="${ENGINE_PATH:-${WS_DIR}/assets/models/engines/orin_jp621_trt_local/yolo26n_fp16_640.engine}"
IMGSZ="${IMGSZ:-640}"
DEVICE="${DEVICE:-0}"
ULTRALYTICS_VERSION="${ULTRALYTICS_VERSION:-8.4.33}"
JETSON_TORCH_INDEX="${JETSON_TORCH_INDEX:-https://pypi.jetson-ai-lab.io/jp6/cu126}"
TORCH_VERSION="${TORCH_VERSION:-2.8.0}"
TORCHVISION_VERSION="${TORCHVISION_VERSION:-0.23.0}"
RECREATE_VENV=false
SKIP_VENV=false

usage() {
  cat << USAGE
Usage: $(basename "$0") [options]

Export yolo26n FP16 TensorRT engine on Jetson Orin / JetPack 6.2.1.

Options:
  --venv <path>          Python venv path (default: $VENV_DIR)
  --model-path <path>    yolo26n.pt path (default: $MODEL_PATH)
  --engine-path <path>   output engine path (default: $ENGINE_PATH)
  --imgsz <n>            export image size (default: $IMGSZ)
  --device <id>          CUDA device id (default: $DEVICE)
  --recreate-venv        recreate the venv before installing packages
  --skip-venv            use the existing venv without installing packages
  -h, --help             show this help

Environment overrides:
  WS_DIR, VENV_DIR, MODEL_PATH, ENGINE_PATH, IMGSZ, DEVICE,
  ULTRALYTICS_VERSION, JETSON_TORCH_INDEX, TORCH_VERSION, TORCHVISION_VERSION
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --venv) VENV_DIR="$2"; shift 2 ;;
    --model-path) MODEL_PATH="$2"; shift 2 ;;
    --engine-path) ENGINE_PATH="$2"; shift 2 ;;
    --imgsz) IMGSZ="$2"; shift 2 ;;
    --device) DEVICE="$2"; shift 2 ;;
    --recreate-venv) RECREATE_VENV=true; shift ;;
    --skip-venv) SKIP_VENV=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1"; usage; exit 2 ;;
  esac
done

PYTHON_BIN="${VENV_DIR}/bin/python"
PIP_BIN="${VENV_DIR}/bin/pip"

echo "[engine] workspace: $WS_DIR"
echo "[engine] venv:      $VENV_DIR"
echo "[engine] model:     $MODEL_PATH"
echo "[engine] output:    $ENGINE_PATH"

if [[ "$SKIP_VENV" != true ]]; then
  if [[ "$RECREATE_VENV" == true && -d "$VENV_DIR" ]]; then
    echo "[engine] removing existing venv: $VENV_DIR"
    rm -rf "$VENV_DIR"
  fi

  if [[ ! -x "$PYTHON_BIN" ]]; then
    echo "[engine] creating venv"
    python3 -m venv "$VENV_DIR"
  fi

  echo "[engine] installing export dependencies"
  env -u PYTHONPATH "$PIP_BIN" install --upgrade pip
  env -u PYTHONPATH "$PIP_BIN" install \
    "ultralytics==${ULTRALYTICS_VERSION}" \
    "onnx==1.21.0" \
    "numpy==1.26.4" \
    "opencv-python==4.10.0.84" \
    "scipy==1.15.3" \
    "pillow==12.1.1" \
    "PyYAML==6.0.3" \
    "requests==2.33.1" \
    "polars==1.39.3" \
    "psutil==7.2.2" \
    "ultralytics-thop==2.0.18"

  echo "[engine] installing JetPack CUDA torch wheels"
  env -u PYTHONPATH "$PIP_BIN" install \
    --index-url "$JETSON_TORCH_INDEX" \
    --force-reinstall \
    --no-cache-dir \
    "torch==${TORCH_VERSION}" \
    "torchvision==${TORCHVISION_VERSION}"
fi

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "[engine] missing venv python: $PYTHON_BIN"
  exit 1
fi

mkdir -p "$(dirname "$MODEL_PATH")" "$(dirname "$ENGINE_PATH")"

if [[ ! -f "$MODEL_PATH" ]]; then
  echo "[engine] downloading yolo26n.pt through Ultralytics"
  (
    cd "$(dirname "$MODEL_PATH")"
    env -u PYTHONPATH "$PYTHON_BIN" - << 'PY'
from ultralytics import YOLO
model = YOLO('yolo26n.pt')
print(model.ckpt_path)
PY
  )
fi

if [[ ! -f "$MODEL_PATH" ]]; then
  echo "[engine] model download failed: $MODEL_PATH"
  exit 1
fi

echo "[engine] exporting TensorRT engine on local Orin"
PYTHONPATH=/usr/lib/python3.10/dist-packages "$PYTHON_BIN" - << PY
from pathlib import Path
import shutil
from ultralytics import YOLO

model_path = Path("${MODEL_PATH}")
engine_path = Path("${ENGINE_PATH}")
engine_path.parent.mkdir(parents=True, exist_ok=True)

model = YOLO(str(model_path))
out = Path(model.export(format="engine", imgsz=int("${IMGSZ}"), half=True, device=int("${DEVICE}"), simplify=False))

if out.resolve() != engine_path.resolve():
    shutil.move(str(out), str(engine_path))

onnx_path = model_path.with_suffix(".onnx")
if onnx_path.exists():
    onnx_path.unlink()

print(f"ENGINE_OUTPUT={engine_path}")
PY

if [[ ! -s "$ENGINE_PATH" ]]; then
  echo "[engine] export did not create a non-empty engine: $ENGINE_PATH"
  exit 1
fi

echo "[engine] sha256:"
sha256sum "$ENGINE_PATH"

echo "[engine] smoke test through Ultralytics TensorRT loader"
PYTHONPATH=/usr/lib/python3.10/dist-packages "$PYTHON_BIN" - << PY
import numpy as np
from ultralytics import YOLO

img = np.zeros((int("${IMGSZ}"), int("${IMGSZ}"), 3), dtype=np.uint8)
model = YOLO("${ENGINE_PATH}")
res = model.predict(source=img, imgsz=int("${IMGSZ}"), device=int("${DEVICE}"), verbose=False)
print("predict_ok", len(res) >= 1)
print("boxes", len(res[0].boxes) if res else -1)
PY

echo "[engine] done"
