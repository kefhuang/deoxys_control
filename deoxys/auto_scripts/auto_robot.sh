#!/bin/bash
# Launch a single robot (arm + gripper) in a tmux session with two panes.
# Usage: ./auto_scripts/auto_robot.sh config/franka-two.yml

set -euo pipefail

if [ $# -lt 1 ]; then
    echo "Usage: $0 <config>"
    echo "Example: $0 config/franka-two.yml"
    exit 1
fi

CONFIG="$1"
SCRIPT_DIR="$(dirname "$0")"

# Derive session name from config filename (e.g. franka-two.yml -> franka-two)
SESSION_NAME="$(basename "$CONFIG" .yml)"

# Extract robot IP from config
ROBOT_IP=$(grep -A1 "^ROBOT:" "$CONFIG" | grep "IP:" | awk '{print $2}')

# Kill existing session if any
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

# Create session running auto_arm.sh in the first pane
tmux new-session -d -s "$SESSION_NAME" "$SCRIPT_DIR/auto_arm.sh $CONFIG"

# Split horizontally and run auto_gripper.sh in the right pane
tmux split-window -h -t "$SESSION_NAME" "$SCRIPT_DIR/auto_gripper.sh $CONFIG"

# Show pane border titles with robot IP
tmux set-option -t "$SESSION_NAME" pane-border-status top
tmux select-pane -t "$SESSION_NAME:0.0" -T "Arm [$ROBOT_IP]"
tmux select-pane -t "$SESSION_NAME:0.1" -T "Gripper [$ROBOT_IP]"

# Attach
tmux attach-session -t "$SESSION_NAME"
