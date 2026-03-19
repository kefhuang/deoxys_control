#!/bin/bash
# Launch two robots (4 processes) in a tmux session with 2x2 layout.
# Usage: ./auto_scripts/auto_bimanual.sh [config1] [config2]
# Defaults: config/franka-two.yml config/franka-three.yml

set -euo pipefail

CONFIG1="${1:-config/franka-two.yml}"
CONFIG2="${2:-config/franka-three.yml}"
SCRIPT_DIR="$(dirname "$0")"
SESSION_NAME="bimanual"

# Extract robot IPs from configs
ROBOT_IP1=$(grep -A1 "^ROBOT:" "$CONFIG1" | grep "IP:" | awk '{print $2}')
ROBOT_IP2=$(grep -A1 "^ROBOT:" "$CONFIG2" | grep "IP:" | awk '{print $2}')

# Kill existing session if any
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

# Create 2x2 layout:
#   top-left  (0): arm 1     |  top-right (1): arm 2
#   bot-left  (2): gripper 1 |  bot-right (3): gripper 2

# Pane 0: top-left — arm for robot 1
tmux new-session -d -s "$SESSION_NAME" "$SCRIPT_DIR/auto_arm.sh $CONFIG1"

# Pane 1: top-right — arm for robot 2
tmux split-window -h -t "$SESSION_NAME:0.0" "$SCRIPT_DIR/auto_arm.sh $CONFIG2"

# Pane 2: bottom-left — gripper for robot 1 (split top-left vertically)
tmux split-window -v -t "$SESSION_NAME:0.0" "$SCRIPT_DIR/auto_gripper.sh $CONFIG1"

# Pane 3: bottom-right — gripper for robot 2 (split top-right vertically)
tmux split-window -v -t "$SESSION_NAME:0.1" "$SCRIPT_DIR/auto_gripper.sh $CONFIG2"

# Force even 2x2 tiled layout
tmux select-layout -t "$SESSION_NAME" tiled

# Show pane border titles with robot IPs
tmux set-option -t "$SESSION_NAME" pane-border-status top
tmux select-pane -t "$SESSION_NAME:0.0" -T "Arm [$ROBOT_IP1]"
tmux select-pane -t "$SESSION_NAME:0.1" -T "Arm [$ROBOT_IP2]"
tmux select-pane -t "$SESSION_NAME:0.2" -T "Gripper [$ROBOT_IP1]"
tmux select-pane -t "$SESSION_NAME:0.3" -T "Gripper [$ROBOT_IP2]"

# Attach
tmux attach-session -t "$SESSION_NAME"
