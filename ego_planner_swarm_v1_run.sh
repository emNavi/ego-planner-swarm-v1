#!/bin/bash
echo "[START] Ego_planner_swarm_v1 "

# 通过本脚本文件路径来获取 x152b 项目文件根目录
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"

gnome-terminal -- bash -c "source ${PROJECT_DIR}/devel/setup.bash;roslaunch ego_planner swarm_all_in_one.launch"
