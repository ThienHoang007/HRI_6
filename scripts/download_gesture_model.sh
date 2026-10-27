#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODEL_DIR="${SCRIPT_DIR}/../models"
MODEL_PATH="${MODEL_DIR}/gesture_recognizer.task"
MODEL_URL="https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task"

mkdir -p "${MODEL_DIR}"
if command -v curl >/dev/null 2>&1; then
  curl -L --fail --retry 3 --output "${MODEL_PATH}" "${MODEL_URL}"
elif command -v wget >/dev/null 2>&1; then
  wget -O "${MODEL_PATH}" "${MODEL_URL}"
else
  echo 'Install curl or wget first.' >&2
  exit 1
fi

test -s "${MODEL_PATH}"
echo "Downloaded ${MODEL_PATH}"
