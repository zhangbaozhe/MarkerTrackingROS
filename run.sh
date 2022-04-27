#!/bin/bash
xterm -e "roscore" & 
xterm -e "sleep 5 && export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ubuntu/BaozheProject1/proj1/proj1/src/spark1_description/model && source ~/.bashrc && cd /home/ubuntu/BaozheProject1/proj1/proj1 && source devel/setup.bash && roslaunch spark1_description spark_gazebo.launch" &
xterm -e "sleep 10 && export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ubuntu/BaozheProject1/proj1/proj1/src/spark1_description/model && source ~/.bashrc && cd /home/ubuntu/BaozheProject1/proj1/proj1 && source devel/setup.bash && roslaunch ar_track_alvar ar_track_depth.launch" &
xterm -e "sleep 10 && export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ubuntu/BaozheProject1/proj1/proj1/src/spark1_description/model && source ~/.bashrc && cd /home/ubuntu/BaozheProject1/proj1/proj1 && source devel/setup.bash && roslaunch spark_navigation move_base_mapless_demo.launch" &
xterm -e "sleep 15 && export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ubuntu/BaozheProject1/proj1/proj1/src/spark1_description/model && source ~/.bashrc && cd /home/ubuntu/BaozheProject1/proj1/proj1 && source devel/setup.bash && rosrun spark_navigation marker_tracking.py" &

 