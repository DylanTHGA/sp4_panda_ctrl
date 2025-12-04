#!/bin/bash

# ===========================================
# startPanda_gazebo_SP4koordinator.sh
# Startet: ROS Core, TF Publisher, Panda-Sim mit CLI Controller, SP4Koordinator,Coordinate Translator
# Alles auf EINEM Rechner
# Param1: Host-IP (Default: 127.0.0.1)
# ===========================================

# IP-Adresse für SP4Koordinator (z.B. Bildanalysator/Chatbot-Orchestrator Host)
if [ -z "$1" ]; then
    IP_ADRESS_HOST='127.0.0.1'
else
    IP_ADRESS_HOST=$1
fi

echo "SP4Koordinator --IP: $IP_ADRESS_HOST"

gnome-terminal \
    --tab --title="ROS Core" \
        --command="bash -c 'echo -e \"\e[1;34m===== ROS Core =====\e[0m\"; source ~/catkin_ws/devel/setup.bash; roscore; exec bash'" \
    --tab --title="Panda CLI Interface" \
        --command="bash -c 'echo -e \"\e[1;34m===== PANDA CLI CONTROLER =====\e[0m\"; sleep 2; source ~/catkin_ws/devel/setup.bash; roslaunch sp3_panda_ctrl sp3_sim.launch; exec bash'" \
    --tab --title="SP4 Koordinator" \
        --command="bash -c 'echo -e \"\e[1;34m===== SP4 Koordinator --IP $IP_ADRESS_HOST =====\e[0m\"; sleep 3; source ~/catkin_ws/devel/setup.bash; rosrun SP4Koordinator SP4Koordinator --IP $IP_ADRESS_HOST; exec bash'" \
    --tab --title="Coordinate Translator" \
        --command="bash -c 'echo -e \"\e[1;34m===== COORDINATE TRANSLATOR =====\e[0m\"; sleep 4; source ~/catkin_ws/devel/setup.bash; rosrun sp3_panda_ctrl coordinate_translator; exec bash'"

echo -e "\e[1;32mSP4 System with Panda launched using object detector.\e[0m"
