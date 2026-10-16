#!/usr/bin/env bash
# Run ON THE PC: copy this workspace to the RDK X5 and build it there.
#   ./deploy_to_x5.sh sunrise@192.168.1.10            # copy + build
#   ./deploy_to_x5.sh sunrise@192.168.1.10 --setup    # first time: also ./setup_x5.sh
# The board keeps its own ~/.config/bhl_onboard/offsets.yaml; config edits made
# on the board are overwritten by the PC copy (edit src/bhl_onboard/config here).
set -e
target=${1:?usage: $0 user@host [--setup]}
cd "$(dirname "$0")"
rsync -az --delete --exclude build --exclude install --exclude log --exclude .git \
      --exclude __pycache__ ./ "$target:bhl_onboard_ws/"
if [ "$2" = "--setup" ]; then
  ssh -t "$target" 'cd ~/bhl_onboard_ws && ./setup_x5.sh'
else
  ssh "$target" 'cd ~/bhl_onboard_ws && source ./env.sh && colcon build --symlink-install 2>&1 | tail -3'
fi
echo "deployed to $target:~/bhl_onboard_ws"
