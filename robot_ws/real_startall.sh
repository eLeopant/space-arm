#!/bin/bash
# start_all.sh —— 一键启动 3 个节点

# ===== 环境 =====
source /opt/ros/jazzy/setup.bash
source install/setup.bash      # 改成你的工作空间

# ===== 记录所有子进程 PID，方便统一清理 =====
PIDS=()

cleanup() {
    echo -e "\n>>> 正在关闭所有节点..."
    for pid in "${PIDS[@]}"; do
        kill -INT "$pid" 2>/dev/null      # 先温柔地 SIGINT（等同 Ctrl+C）
    done
    sleep 1
    for pid in "${PIDS[@]}"; do
        kill -9 "$pid" 2>/dev/null        # 还在的强杀
    done
    wait 2>/dev/null
    echo ">>> 全部已关闭。"
    exit 0
}
trap cleanup INT TERM

# ===== 依次启动（后台） =====
echo ">>> 启动 Gazebo..."
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=false &
PIDS+=($!)

# 等 move_group 服务起来再启动 ctrl_node（ctrl_node 里 wait_for_service 也会等，但这样日志更清晰）
echo ">>> 启动 MoveIt move_group..."
ros2 launch openarm_moveit_config move_group.launch.py use_sim_time:=false &
PIDS+=($!)

sleep 3    # 给 move_group 一点启动时间，可调
echo ">>>  ctrl_node..."
ros2 run armctrl_py armctrl --ros-args -p use_sim_time:=false &
PIDS+=($!)

echo ">>> 全部节点已启动。Ctrl+C 一键关闭。"

# ===== 阻塞在这里，任何一个子进程退出脚本都还活着，等 Ctrl+C =====
wait