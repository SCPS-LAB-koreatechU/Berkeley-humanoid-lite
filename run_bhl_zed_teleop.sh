#!/usr/bin/env bash
# ZED teleop -> Berkeley Humanoid Lite (MuJoCo). Lean to walk.
#   Windows: ZED publisher streaming on :5556 (zed_publisher_windows.py)
#   WSL:     this script -> BHL walks in the MuJoCo viewer (WSLg)
#
# Usage:
#   bash run_bhl_zed_teleop.sh                 # host = WSL->Windows gateway, auto
#   bash run_bhl_zed_teleop.sh --debug         # print derived vx/vy/vyaw to tune
#   bash run_bhl_zed_teleop.sh --host 172.26.0.1 --sign-x -1   # flip an axis
set -e
cd "$(dirname "$0")"
source .venv_sim2sim/bin/activate
export PYTHONPATH="source/berkeley_humanoid_lite:${PYTHONPATH:-}"
unset MUJOCO_GL   # WSLg windowed viewer

# default ZED host = current WSL->Windows gateway (changes on reboot)
GW=$(grep -m1 nameserver /etc/resolv.conf | awk '{print $2}')
HOST_DEFAULT="${GW:-172.26.0.1}"

# inject default --host if the user didn't pass one
case " $* " in
  *" --host "*) HOST_ARGS=() ;;
  *) HOST_ARGS=(--host "$HOST_DEFAULT") ;;
esac

echo "[run_bhl_zed_teleop] ZED host default=$HOST_DEFAULT"
exec python scripts/sim2sim/play_mujoco_zed.py "${HOST_ARGS[@]}" "$@"
