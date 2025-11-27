#!/bin/bash

# Left pane (0): instructions
# Right-top pane (1): minicom
# Right-bottom pane (2): player input

SERIAL_PORT="/dev/ttyACM0"

# Create tmux session
SESSION_NAME="pitch-game"

# Kill existing session if it exists
tmux kill-session -t "$SESSION_NAME" 2>/dev/null

# Create new session
tmux new-session -d -s "$SESSION_NAME" -n "Pitch Matching Game"

# Split horizontally: left 30%, right 70%
tmux split-window -h -p 20    # new pane gets 20%

# Split vertically: bottom 20%
tmux select-pane -t 0
tmux split-window -v -p 20    # new pane gets 20%

# Select bottom pane
tmux select-pane -t 2

# Attach
tmux attach-session -t "$SESSION_NAME"
