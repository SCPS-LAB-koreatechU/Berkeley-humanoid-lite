#!/usr/bin/env bash
# Berkeley Humanoid Lite — MuJoCo sim2sim launcher for WSL2 (RTX4090).
# Opens the MuJoCo passive viewer via WSLg with a pretrained ONNX policy.
#
# Usage:
#   bash run_sim2sim.sh                       # stand in place (biped 50Hz policy)
#   bash run_sim2sim.sh --vx 0.4              # walk forward
#   bash run_sim2sim.sh --config configs/policy_biped_25hz_a.yaml --vyaw 0.3
#
# A real USB gamepad is NOT required; pass --vx/--vy/--vyaw to drive.
set -e
cd "$(dirname "$0")"
source .venv_sim2sim/bin/activate
export PYTHONPATH="source/berkeley_humanoid_lite:${PYTHONPATH:-}"
# WSLg provides an X/Wayland display; let MuJoCo pick the windowed GL backend.
unset MUJOCO_GL

CONFIG="configs/policy_biped_50hz.yaml"
# pull out --config if the user supplied one; pass the rest (e.g. --vx) through
ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config) CONFIG="$2"; shift 2;;
    *) ARGS+=("$1"); shift;;
  esac
done

echo "[run_sim2sim] config=$CONFIG  extra=${ARGS[*]:-<none>}"
exec python scripts/sim2sim/play_mujoco_wsl.py --config "$CONFIG" "${ARGS[@]}"
