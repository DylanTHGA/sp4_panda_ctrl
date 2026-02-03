#!/bin/bash

# ===========================================
# start_panda_gazebo_sp.sh
# Startet: Panda Gazebo + MoveIt (panda_demo_gazebo.launch),
#          Panda CLI Control, Coordinate Translator
# Alles auf einem Rechner
# ===========================================

echo "Starting SweetPicker Panda Gazebo setup (demo + nodes)"

gnome-terminal \
    --tab --title="Panda Gazebo + MoveIt" \
        --command="bash -c 'echo -e \"\e[1;34m===== PANDA DEMO GAZEBO (MOVEIT) =====\e[0m\"; \
                          sleep 2; \
                          source ~/catkin_ws/devel/setup.bash; \
                          roslaunch sp4_panda_ctrl panda_demo_gazebo.launch; \
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

echo -e "\e[1;32mSweetPickerxPanda Gazebo launched (Bringup + CLI + Translator).\e[0m"

