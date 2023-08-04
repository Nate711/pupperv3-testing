#!/bin/bash

tmux new-session -d -s pupperv3

tmux split-window -h
tmux split-window -h

tmux send-keys -t pupperv3:0 "source_ros; source_install" C-m
tmux send-keys -t pupperv3:1 "source_ros; source_install" C-m
tmux send-keys -t pupperv3:2 "source_ros; source_install" C-m

tmux select-layout -t pupperv3:0 even-horizontal

tmux attach-session -t pupperv3