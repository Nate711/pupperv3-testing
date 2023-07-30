#!/bin/bash

tmux new-session -d -s pupperv3

tmux split-window -h
tmux split-window -h

tmux send-keys -t pupperv3:0.0 "source_ros; source_install; cd pupperv3-testing" C-m
tmux send-keys -t pupperv3:0.1 "source_ros; source_install; cd StanfordQuarduped" C-m
tmux send-keys -t pupperv3:0.2 "source_ros; source_install; ros2 run joy_linux joy_linux_node" C-m

tmux select-layout -t pupperv3:0 even-horizontal

tmux attach-session -t pupperv3