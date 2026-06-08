#!/usr/bin/env bash
# 下载 Sherpa-ONNX 官方中文声纹 embedding 模型（CAM++ 16k）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUT_DIR="${PKG_ROOT}/models/speaker"
MODEL_NAME="3dspeaker_speech_campplus_sv_zh-cn_16k-common.onnx"
URL="https://github.com/k2-fsa/sherpa-onnx/releases/download/speaker-recongition-models/${MODEL_NAME}"

mkdir -p "${OUT_DIR}"
DEST="${OUT_DIR}/${MODEL_NAME}"

if [[ -f "${DEST}" ]]; then
  echo "已存在: ${DEST} ($(du -h "${DEST}" | cut -f1))"
  exit 0
fi

echo "下载声纹模型 -> ${DEST}"
if command -v curl >/dev/null 2>&1; then
  curl -fL --retry 3 -o "${DEST}" "${URL}"
elif command -v wget >/dev/null 2>&1; then
  wget -O "${DEST}" "${URL}"
else
  echo "需要 curl 或 wget" >&2
  exit 1
fi

echo "完成: ${DEST}"
echo "请在 config/doubao.yaml 中设置 speaker_id.enabled: true"
