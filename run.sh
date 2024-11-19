#!/bin/bash
echo "[START] Ego_Planner_Swarm_V1 "

# 通过本脚本文件路径来获取 x152b 项目文件根目录
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"

max_vel=1.0
max_acc=1.0
drone_id=$(python3 ${PROJECT_DIR}/scripts/find_config.py drone_id)
# <!-- 1: use 2D Nav Goal to select goal  -->
# <!-- 2: use global waypoints below  -->
# <!-- 4: use REMOTE_TARGET  -->
# <!-- 5: use REMOTE_START  -->
flight_type=2
echo "Drone $drone_id  autonomous run in max_vel $max_vel max_acc:=$max_acc"

cx=$(python3 ${PROJECT_DIR}/scripts/find_config.py cx)
cy=$(python3 ${PROJECT_DIR}/scripts/find_config.py cy)
fx=$(python3 ${PROJECT_DIR}/scripts/find_config.py fx)
fy=$(python3 ${PROJECT_DIR}/scripts/find_config.py fy)

echo ${PROJECT_DIR}

if [ $? -eq 0 ] 
then
    source ${PROJECT_DIR}/devel/setup.bash;roslaunch ego_planner swarm_all_in_one.launch \
    drone_id:=$drone_id cx:=$cx cy:=$cy fx:=$fx fy:=$fy flight_type:=$flight_type max_vel:=$max_vel max_acc:=$max_acc && sleep 1;
    # gnome-terminal -- bash -c "source ${PROJECT_DIR}/devel/setup.bash;roslaunch ego_planner swarm_all_in_one.launch \
    # drone_id:=$drone_id cx:=$cx cy:=$cy fx:=$fx fy:=$fy flight_type:=$flight_type max_vel:=$max_vel max_acc:=$max_acc" && sleep 1;
else
    echo error
fi