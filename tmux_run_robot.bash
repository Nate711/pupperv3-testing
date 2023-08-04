#!/bin/bash

tmux new-session -d -s pupperv3

tmux split-window -h
tmux split-window -h

tmux send-keys -t pupperv3:0.0 "source_ros; source_install" C-m
tmux send-keys -t pupperv3:0.0 "ros2 run cpp_pubsub motor_controller_generic_node_test"

tmux send-keys -t pupperv3:0.1 "source_ros; source_install" C-m
tmux send-keys -t pupperv3:0.1 "cd ~/StanfordQuadruped" C-m
tmux send-keys -t pupperv3:0.1 "python3 examples/pupper_v3_ds4_control.py"

tmux send-keys -t pupperv3:0.2 "source_ros; source_install" C-m
tmux send-keys -t pupperv3:0.2 "ros2 run joy_linux joy_linux_node"

tmux select-layout -t pupperv3:0 even-horizontal

tmux attach-session -t pupperv3