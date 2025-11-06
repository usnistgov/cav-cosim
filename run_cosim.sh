#!/usr/bin/env bash
# One qterminal window, 5 stacked panes, each running your command.

SESSION=sim4

# Start clean
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

# -------------------- Panes & Commands --------------------

# Pane 1 (top): CARLA
#tmux new-session -d -s "$SESSION" \
 #"bash -lic 'source ~/.bashrc; iotav_matlab; vulkan; cd ~/iotav/CARLA_0.9.15; ./CarlaUE4.sh -renderoffscreen'"
#sleep 10


# Pane 2
#tmux split-window -fv -t "$SESSION":0 \
tmux new-session -d -s "$SESSION" \
  "bash -lic 'source ~/.bashrc; cd ~/iotav/cosim/ns3-v2i && python3 intermediate_server_python.py'"
sleep 5

# Pane 3
tmux split-window -fv -t "$SESSION":0 \
  "bash -lic \"source ~/.bashrc; cd ~/iotav/ns-3-dev && ./ns3 run 'gateway-v2i-wifi-2 --verbose'\""
sleep 5

# Pane 4
tmux split-window -fv -t "$SESSION":0 \
  "bash -lic 'source ~/.bashrc; iotav_matlab; ros2 run carla_traffic_monitor traffic_light_monitor'"
  
sleep 5

# Pane 5 (bottom)
tmux split-window -fv -t "$SESSION":0 \
  "bash -lic 'source ~/.bashrc; iotav_matlab; ros2 launch carla_ad_demo carla_matlab_demo_traffic_light.launch.py'"
  
# Make the layout exactly like your screenshot: five even rows tmux 
select-layout -t "$SESSION":0 even-vertical 
  
# Attach so you see the panes 
tmux attach -t "$SESSION"

