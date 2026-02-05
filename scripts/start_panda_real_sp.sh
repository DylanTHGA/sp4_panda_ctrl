#!/bin/bash

# ===========================================
# start_panda_real_sp.sh
# Startet: Panda CLI Control, Coordinate Translator
# Alles auf einem Rechner

# ===========================================


gnome-terminal \
    --tab --title="Panda CLI Control" \
        --command="bash -c 'echo -e \"\e[1;34m===== PANDA CLI CONTROL =====\e[0m\"; \
                          source ~/catkin_ws/devel/setup.bash; \
                          rosrun sp4_panda_ctrl panda_cli_control; \
                          exec bash'" \
    --tab --title="Coordinate Translator" \
        --command="bash -c 'echo -e \"\e[1;34m===== COORDINATE TRANSLATOR =====\e[0m\"; \
                          sleep 10; \
                          source ~/catkin_ws/devel/setup.bash; \
                          rosrun sp4_panda_ctrl coordinate_translator; \
                          exec bash'"

echo -e "\e[1;32mSweetPickerxPanda REAL launched (Bringup + CLI + Translator).\e[0m"

