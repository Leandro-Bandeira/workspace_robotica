#!/bin/bash

SESSION="jetauto"

# Se a sessão já existir, apenas conecta
tmux has-session -t $SESSION 2>/dev/null
if [ $? -eq 0 ]; then
  tmux attach -t $SESSION
  exit 0
fi

# Cria nova sessão tmux em background
tmux new-session -d -s $SESSION -n description

# Aba 0 - Robot Description
tmux send-keys -t $SESSION:0 \
  "source install/setup.bash && ros2 launch robotics_class robot_description.launch.py" C-m
sleep 1 # Aguarda 1 segundo

# Aba 1 - Simulation World
tmux new-window -t $SESSION -n simulation
tmux send-keys -t $SESSION:1 \
  "source install/setup.bash && ros2 launch robotics_class simulation_world.launch.py" C-m
sleep 5 # RECOMENDAÇÃO: O Gazebo é pesado, 1s geralmente é pouco. Coloquei 5s aqui.

# Aba 2 - EKF
tmux new-window -t $SESSION -n ekf
tmux send-keys -t $SESSION:2 \
  "source install/setup.bash && ros2 launch robotics_class ekf.launch.py" C-m
sleep 1

# Aba 3 - RViz
tmux new-window -t $SESSION -n rviz
tmux send-keys -t $SESSION:3 \
  "source install/setup.bash && ros2 launch robotics_class rviz.launch.py" C-m
sleep 1

# Aba 4 - Teleop
tmux new-window -t $SESSION -n teleop
tmux send-keys -t $SESSION:4 \
  "source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=jetauto/cmd_vel" C-m
sleep 1

# Aba 5 - localization
tmux new-window -t $SESSION -n localization
tmux send-keys -t $SESSION:5 \
  "source install/setup.bash && ros2 launch robotics_class localization.launch.py" C-m
sleep 1

# Volta para a primeira aba
tmux select-window -t $SESSION:0

# Anexa à sessão
tmux attach -t $SESSION