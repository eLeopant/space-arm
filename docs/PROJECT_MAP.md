# OpenArm 项目地图（v1.0 主线）

> 目的：让新同学在 5 分钟内知道「要改什么 → 打开哪个文件」，不用通读全部 xacro。
> 仓库：`/home/leo/space-arm_ws`；唯一 ROS 2 包：`robot_ws/src/openarm_description`
> 实物版本：**v1.0（`v10`）**；v2.0 并存，差异见 §9。

## 1. 顶层结构

```
space-arm_ws/
├── docs/
│   ├── PROJECT_MAP.md              ← 本文件（先看这个）
│   └── gazebo_simulation_guide.md  ← Gazebo 接入步骤 / 排查手册
├── robot_ws/
│   ├── src/openarm_description/    ← 唯一的 ROS 2 包
│   ├── build/ install/ log/        ← colcon 产物
├── .vscode/                        ← 仅 Python 解释器路径
└── README.md
```

## 2. 包结构（只列有意义的部分）

```
openarm_description/
├── CMakeLists.txt                  # 安装 assets/config/launch/rviz/scripts/worlds
├── package.xml                     # 依赖 xacro / robot_state_publisher / gz_ros2_control ...
├── assets/
│   ├── robot/
│   │   ├── openarm_v1.0/           # ★ 我们在用的这一版
│   │   │   ├── urdf/
│   │   │   │   ├── openarm_v10.urdf.xacro       # 入口（唯一对外 xacro）
│   │   │   │   ├── robot/openarm_robot.xacro    # 组装：body + 2×arm + 2×ee + ros2_control
│   │   │   │   ├── arm/openarm_macro.xacro      # 单关节/单连杆的宏
│   │   │   │   ├── arm/openarm_arm.xacro        # 串起 arm 宏 → joint1..7
│   │   │   │   ├── body/openarm_body*.xacro     # 机身
│   │   │   │   ├── ee/ee_dispatcher.xacro       # 末端分发
│   │   │   │   └── ros2_control/
│   │   │   │       ├── openarm.bimanual.ros2_control.xacro  # 双臂 hardware interface
│   │   │   │       └── openarm.ros2_control.xacro           # 单臂
│   │   │   └── config/
│   │   │       ├── arm/joint_limits.yaml        # 限位 / 速度 / 力矩
│   │   │       ├── arm/inertials.yaml           # 质量 / 惯量
│   │   │       ├── arm/control_gains.yaml       # kp / kd
│   │   │       ├── arm/kinematics*.yaml         # 尺寸 / 零位偏移（3 个文件配套）
│   │   │       └── body/*.yaml
│   │   └── openarm_v2.0/           # 另一版：结构同上，多 utils/ 与 config/robot_presets/
│   ├── end_effector/
│   │   ├── parallel_link/          # v1.0 的平行夹爪（prismatic 手指，行程 0~0.044 m）
│   │   └── pinch_gripper/          # v2.0 的捏取夹爪（revolute 手指，0~0.7854 rad）
│   └── sensor/zed/urdf_wrapper/openarm_zed_wrapper.xacro
├── config/gazebo/controllers.yaml   # ★ 仿真控制器配置（本次新增）
├── worlds/openarm_empty.sdf         # ★ 仿真世界（本次新增）
├── launch/
│   ├── display_openarm.launch.py    # RViz + 假关节（无物理）
│   └── gazebo_openarm.launch.py     # ★ Gazebo + RViz + 控制器（本次新增）
├── rviz/{bimanual,arm_only}.rviz    # Fixed Frame = world
└── dev/{README.md,release.sh}       # 发版脚本，日常用不到
```

## 3. 功能 → 文件 索引（最常用）

| 我要做… | 改哪里（v1.0 路径） | 备注 |
|---|---|---|
| 改关节限位/速度/力矩 | `assets/robot/openarm_v1.0/config/arm/joint_limits.yaml` | 改完重新展开 xacro 即生效 |
| 改连杆质量/惯量 | `.../config/arm/inertials.yaml`、`.../config/body/inertials.yaml` | 影响 Gazebo 动力学 |
| 改控制增益 kp/kd | `.../config/arm/control_gains.yaml` | 只给真机 CAN 用，仿真不读 |
| 改连杆长度/关节零位 | `.../config/arm/kinematics.yaml` + `kinematics_link.yaml` + `kinematics_offset.yaml` | 3 个文件必须配套改 |
| 加/删一个臂关节 | `.../urdf/arm/openarm_arm.xacro` + `openarm_macro.xacro` | 再同步 joint_limits / inertials |
| 改机身结构 | `.../urdf/body/openarm_body*.xacro` + `config/body/*.yaml` | |
| 换末端执行器 | `.../urdf/ee/ee_dispatcher.xacro` + `assets/end_effector/*` | v1.0 固定 `parallel_link` |
| 改手指 mimic 比例 | `assets/end_effector/pinch_gripper/config/joint/joint_mimics.yaml` | 只对 v2.0 生效 |
| 单臂 ↔ 双臂 | `.../urdf/robot/openarm_robot.xacro` 的 `bimanual` 参数 | |
| 改 ros2_control 接口 | `.../urdf/ros2_control/*.xacro` | 每个关节 3 个 command + 3 个 state |
| 挂相机 | `assets/sensor/zed/urdf_wrapper/openarm_zed_wrapper.xacro` | 需要 `zed_description` |
| RViz 看模型 | `launch/display_openarm.launch.py` + `rviz/bimanual.rviz` | 无物理、无 /clock |
| 跑 Gazebo 仿真 | `launch/gazebo_openarm.launch.py` | 默认 `arm_type:=v10` |
| 仿真控制器增删/调参 | `config/gazebo/controllers.yaml` | 只允许 `position` command interface |
| 仿真换场景 | `worlds/openarm_empty.sdf`，或 `world:=<绝对路径>` | |
| 加新安装目录 | `CMakeLists.txt` 的 `foreach(install_dir ...)` + `package.xml` | 漏了会「运行时找不到文件」 |

## 4. 三条关键链路

### 4.1 URDF 组装链（v1.0）

```
openarm_v10.urdf.xacro
└─ xacro:openarm_robot            urdf/robot/openarm_robot.xacro
   ├─ xacro:openarm_body          urdf/body/openarm_body.xacro   ← config/body/*
   ├─ xacro:openarm_arm  ×2       urdf/arm/openarm_arm.xacro     ← config/arm/*
   ├─ xacro:openarm_ee   ×2       urdf/ee/ee_dispatcher.xacro    ← end_effector/parallel_link/config/*
   └─ xacro:openarm_bimanual_ros2_control
                                  urdf/ros2_control/openarm.bimanual.ros2_control.xacro
```

改结构的入口永远是 `openarm_robot.xacro`，**不是**根 xacro。

### 4.2 仿真启动链（`launch/gazebo_openarm.launch.py`）

```
xacro(sim:=gz) ──► robot_state_publisher ──► /robot_description
                                              │
gz sim worlds/openarm_empty.sdf ◄─────────────┘
      │
ros_gz_sim create -topic robot_description   → 模型 openarm 出现
      │ (进程退出事件)
spawner joint_state_broadcaster
      │
spawner left_arm_controller + right_arm_controller
      │
spawner left_gripper_controller + right_gripper_controller
      │
ros_gz_bridge /clock  → 喂给所有 use_sim_time 节点
rviz2 -d rviz/bimanual.rviz
```

用 `OnProcessExit` 事件链而不是 `sleep`：Gazebo 冷启动慢也不会抢跑。

### 4.3 控制链

```
FollowJointTrajectory → /left_arm_controller   (JointTrajectoryController)
Float64MultiArray     → /left_gripper_controller/commands (JointGroupPositionController)
                              ↓ position command interface
                        gz_ros2_control/GazeboSimSystem
                              ↓
                          Gazebo 物理
                              ↓
/joint_states ← joint_state_broadcaster（18 个关节，含 2 个 mimic 手指）
```

## 5. 参数速查

### 5.1 launch 参数（`gazebo_openarm.launch.py`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `arm_type` | `v10` | `v10/v1.0` 或 `v20/v2.0` |
| `robot_preset` | `default_bimanual` | **仅 v2.0 生效** |
| `world` | `openarm_empty.sdf` | `worlds/` 下的文件名，或绝对路径 |
| `gui` | `true` | `false` = `gz sim -s`（服务器 / CI） |
| `rviz` | `true` | 是否同时起 RViz2 |

### 5.2 xacro 参数

| 版本 | 常用参数 |
|---|---|
| v1.0 | `ros2_control:=true`（默认 `false`，仿真必须给）、`bimanual:=true`、`sim:=gz`、`use_fake_hardware:=false`、`arm_type:=v10`、`body_type:=v10` |
| v2.0 | `robot_preset:=default_bimanual`、`sim:=gz`、`use_fake_hardware:=true`、`collapse_internal_empty_links:=true` |

`sim` 只有两个取值：`none`（默认，保持真机 / RViz 行为）、`gz`（Gazebo）。

## 6. 关节与控制器对照

| 控制器 | 类型 | 关节 |
|---|---|---|
| `joint_state_broadcaster` | JointStateBroadcaster | 全部 18 个（含 mimic 指） |
| `left_arm_controller` | JointTrajectoryController | `openarm_left_joint1..7` |
| `right_arm_controller` | JointTrajectoryController | `openarm_right_joint1..7` |
| `left_gripper_controller` | JointGroupPositionController | `openarm_left_finger_joint1` |
| `right_gripper_controller` | JointGroupPositionController | `openarm_right_finger_joint1` |

- `update_rate: 200`（与 `worlds/openarm_empty.sdf` 的 1 ms 物理步长无关，可独立调）
- 只用 `position` command interface：同一关节声明多个接口时 gz_ros2_control 只认第一个（[#343](https://github.com/ros-controls/gz_ros2_control/issues/343)）
- `finger_joint2` 只声明 state interface：mimic 关节不能下命令，gz-sim 会按 SDF 的 `<mimic>` 自动让它跟随

## 7. 常用命令（最小集）

```bash
source /opt/ros/jazzy/setup.bash
source /home/leo/space-arm_ws/robot_ws/install/setup.bash

# 编译（--symlink-install：改 xacro/yaml 后不用重编）
cd /home/leo/space-arm_ws/robot_ws
colcon build --packages-select openarm_description --symlink-install

# 仿真（默认 v10 + GUI + RViz）
ros2 launch openarm_description gazebo_openarm.launch.py
ros2 launch openarm_description gazebo_openarm.launch.py gui:=false rviz:=false   # headless
ros2 launch openarm_description gazebo_openarm.launch.py arm_type:=v20            # v2.0

# 只看模型（无物理、无 /clock）
ros2 launch openarm_description display_openarm.launch.py

# 控制器状态（本机没装 ros2controlcli，用 service）
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers "{}"

# 左臂动一下（joint1 → 0.3 rad）
ros2 action send_goal /left_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [openarm_left_joint1,openarm_left_joint2,openarm_left_joint3,openarm_left_joint4,openarm_left_joint5,openarm_left_joint6,openarm_left_joint7], points: [{positions: [0.3,0,0,0,0,0,0], time_from_start: {sec: 3}}]}}"

# 夹爪（v10 单位是米，行程 0~0.044）
ros2 topic pub --once /left_gripper_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.03]}"

# 只展开 URDF 看结果（X = v1.0 根 xacro 路径）
xacro $X ros2_control:=true sim:=gz | less
```

## 8. 已知坑（都踩过）

| 现象 | 原因 | 处理 |
|---|---|---|
| `name 'sim' is not defined` | v1.0 少一层 `openarm_robot` 宏，`sim` 只加到根 xacro | 根 → `openarm_robot` → `openarm_bimanual_ros2_control` 三层都要透传 |
| 一个控制器都没起来 | v1.0 的 `use_fake_hardware` 默认 `false`，真机分支也插入了 `OpenArmHW` | 真机分支条件写成 `or sim == 'gz'` |
| `finger_joint2` 不在 `/joint_states` | ros2_control 里没声明它 | 加 state-only joint（**不要** command interface） |
| 夹爪不动 | v10 手指是 prismatic，单位是**米**（0~0.044） | 别发 0.3 / 0.6 这种弧度值 |
| `ros2 control ...` 命令不存在 | 本机未装 `ros2controlcli` | 改用 `/controller_manager/*` service |
| `rqt_graph` 只看到自己 | DDS 发现不一致 / daemon 缓存 | `ros2 daemon stop && ros2 daemon start`，比对 `ROS_DOMAIN_ID` |
| 模型全是白块 | `GZ_SIM_RESOURCE_PATH` 没包含 `share/` | launch 已自动设置；手动跑 `gz sim` 时要自己 export |
| `/clock` 只有 800~970 Hz | 本机 RTF < 1 | 正常，不要按 1000 判失败 |
| 改了 xacro 不生效 | `$(find openarm_description)` 指向 install 目录 | 确认用 `--symlink-install`，或重新 build |

## 9. v1.0 ↔ v2.0 差异（只列会影响你的）

| 项 | v1.0（我们在用） | v2.0 |
|---|---|---|
| 入口 | `assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro` | `assets/robot/openarm_v2.0/urdf/openarm_v20.urdf.xacro` |
| 规格 | 26 link / 25 joint（14 rev + 4 pris + 7 fix） | 22 link / 21 joint（18 rev + 3 fix） |
| 手指 | prismatic，行程 `0~0.044 m` | revolute，行程 `0~0.7854 rad` |
| 末端 | `parallel_link`（写死） | `pinch_gripper`（可用 preset 换） |
| ros2_control 生成 | 需要 `ros2_control:=true`（默认 `false`） | 总是生成 |
| 宏层级 | 根 xacro → `openarm_robot` → bimanual 宏（3 层） | 根 xacro → bimanual 宏（2 层） |
| 配置方式 | `config/arm/*.yaml` 固定文件名 | `config/robot_presets/*.yaml` 可组合 |
| launch 用法 | `gazebo_openarm.launch.py`（默认） | `gazebo_openarm.launch.py arm_type:=v20` |

## 10. 本次改动记录（Gazebo 接入）

| 文件 | 改动 | 状态 |
|---|---|---|
| `openarm_v1.0/urdf/openarm_v10.urdf.xacro` | `sim` arg + property + `<gazebo>` 插件块 | 已验证 |
| `openarm_v1.0/urdf/robot/openarm_robot.xacro` | `sim` 参数 + 两级透传 | 已验证 |
| `openarm_v1.0/urdf/ros2_control/openarm.bimanual.ros2_control.xacro` | hardware 三选一 + `finger_joint2` state-only | 已验证 |
| `openarm_v1.0/urdf/ros2_control/openarm.ros2_control.xacro` | 单臂版同样处理 | 已验证 |
| `openarm_v2.0/urdf/openarm_v20.urdf.xacro` | 同样的 `sim` 支持 | 已在 commit `190f551` |
| `openarm_v2.0/urdf/ros2_control/openarm.bimanual.ros2_control.xacro` | 同上 | 已在 commit `190f551` |
| `config/gazebo/controllers.yaml` | 新增：5 个控制器 | 已验证 |
| `worlds/openarm_empty.sdf` | 新增：空世界 | 已验证 |
| `launch/gazebo_openarm.launch.py` | 新增：Gazebo + RViz 一键启动，`arm_type` 双版本 | 已验证 |
| `CMakeLists.txt` / `package.xml` | 安装 `config`/`worlds`，补运行依赖 | 已在 commit `190f551` |

### 实测基线（v1.0，headless）

```
5 个控制器 Configured and activated
ros2 topic hz /clock          → 869 Hz（RTF ≈ 0.87）
/joint_states                 → 18 个关节
FollowJointTrajectory(左臂)   → status: 4, error_code: 0
  left_joint1 = 0.3000, left_joint4 = 0.4000
夹爪发 0.03                   → finger_joint1 = 0.0300, finger_joint2 = 0.0300
```

自检命令：

```bash
# 1) v10 展开：期望 2
xacro .../openarm_v10.urdf.xacro ros2_control:=true sim:=gz | grep -c GazeboSimSystem
# 2) 默认行为不变：期望 2（OpenArmHW）
xacro .../openarm_v10.urdf.xacro ros2_control:=true | grep -c OpenArmHW
# 3) 仿真起来后：期望 18
ros2 topic echo /joint_states --once | grep -c "^- "
```

