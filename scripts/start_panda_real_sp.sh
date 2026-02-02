#!/bin/bash

# ===========================================
# start_panda_real_sp.sh
# Startet: Panda Realbetrieb (franka_control.launch),
#          Panda CLI Control, Coordinate Translator
# Alles auf einem Rechner
#
# Param1: Robot-IP (Default: 172.16.0.2)
# ===========================================

# Robot IP (Franka Controller)
if [ -z "$1" ]; then
    ROBOT_IP='172.16.0.2'
else
    ROBOT_IP=$1
fi

echo "Starting SweetPicker Panda REAL setup (robot_ip: $ROBOT_IP)"

gnome-terminal \
    --tab --title="Panda Real Bringup" \
        --command="bash -c 'echo -e \"\e[1;34m===== FRANKA CONTROL + MOVEIT (REAL) =====\e[0m\"; \
                          source ~/catkin_ws/devel/setup.bash; \
                          roslaunch sp4_panda_ctrl franka_control.launch robot_ip:=$ROBOT_IP; \
                          exec bash'" \
    --tab --title="Panda CLI Control" \
        --command="bash -c 'echo -e \"\e[1;34m===== PANDA CLI CONTROL =====\e[0m\"; \
                          sleep 6; \
                          source ~/catkin_ws/devel/setup.bash; \
                          rosrun sp4_panda_ctrl panda_cli_control; \
                          exec bash'" \
    --tab --title="Coordinate Translator" \
        --command="bash -c 'echo -e \"\e[1;34m===== COORDINATE TRANSLATOR =====\e[0m\"; \
                          sleep 8; \
                          source ~/catkin_ws/devel/setup.bash; \
                          rosrun sp4_panda_ctrl coordinate_translator; \
                          exec bash'"

echo -e "\e[1;32mSweetPickerxPanda REAL launched (Bringup + CLI + Translator).\e[0m"

