# AI 交接文档 · OpenArm v1.0 Gazebo 仿真

> **给下一个 AI 会话：只读本文件就能开工**（约 250 行）。
> 只有要改某个具体文件时，才去读那个文件本身。**不要 `find` / `grep` 全仓扫描。**
> 最后更新：2026-11，对应 `HEAD=190f551` + 未提交的 v1.0 仿真改动。

## 0. TL;DR（30 秒读完）

- 仓库 `/home/leo/space-arm_ws`，唯一 ROS 2 包 `robot_ws/src/openarm_description`。
- **实物是 v1.0（`v10`）**；v2.0 的代码并存但**不使用**。
- 环境：Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic（gz-sim 8）+ gz_ros2_control 1.2.20。
- **没有 MoveIt，也不在本项目范围内**（`ros2 pkg list | grep moveit` 为空）——不要假设、不要引入。
- 已完成：v1.0 的 Gazebo 仿真（双臂 + 双夹爪 + 5 控制器 + RViz 同步），一键启动见 §8。
- 用户已有自写控制节点：**`armctrl_py_topic`** 包（纯 topic，4 个 publisher），**有 4 个待修问题**，见 §11。

## 1. 环境与既有事实（不要重复验证）

| 项 | 值 / 命令 |
|---|---|
| ROS | `/opt/ros/jazzy` |
| Gazebo | `gz sim` 8.x（`ros-jazzy-gz-sim-vendor`） |
| ros2_control | `ros-jazzy-gz-ros2-control 1.2.20`，插件 `libgz_ros2_control-system.so` |
| `ros2 control` | ✅ 可用（`ros2controlcli` 已安装） |
| MoveIt | ❌ 未安装，**不要引入** |
| 构建 | `cd robot_ws && colcon build --packages-select openarm_description --symlink-install`（约 1 s） |
| 环境 | `source /opt/ros/jazzy/setup.bash && source robot_ws/install/setup.bash` |
| 注意 | `/tmp` 会被系统清理，验证脚本需现写 |

## 2. 文件地图（改东西只看这张表）

包根：`robot_ws/src/openarm_description/`

| 路径 | 职责 | 何时改 |
|---|---|---|
| `assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro` | v1.0 入口；`sim` arg + `<gazebo>` 插件块 | 换插件 / 改入口参数 |
| `assets/robot/openarm_v1.0/urdf/robot/openarm_robot.xacro` | 组装 body + 2×arm + 2×ee + ros2_control；`sim` 透传 | 单/双臂、结构 |
| `assets/robot/openarm_v1.0/urdf/arm/openarm_arm.xacro` + `openarm_macro.xacro` | 关节 / 连杆宏 | 加关节 |
| `assets/robot/openarm_v1.0/urdf/body/*.xacro` | 机身 | 机身结构 |
| `assets/robot/openarm_v1.0/urdf/ee/ee_dispatcher.xacro` | 末端分发（v1.0 → `parallel_link`） | 换末端 |
| `assets/robot/openarm_v1.0/urdf/ros2_control/openarm.bimanual.ros2_control.xacro` | 双臂 hardware interface + 关节接口 | 改 ros2_control |
| `assets/robot/openarm_v1.0/urdf/ros2_control/openarm.ros2_control.xacro` | 单臂版 | 同上 |
| `assets/robot/openarm_v1.0/config/arm/{joint_limits,inertials,control_gains,kinematics*}.yaml` | 限位 / 惯量 / 增益 / 尺寸 | 调参 |
| `assets/end_effector/parallel_link/config/*` | v1.0 夹爪（**prismatic** 手指） | 夹爪参数 |
| `assets/sensor/camera/openarm_hand_camera.xacro` | ★ 手眼相机宏（gz camera sensor，`camera:=true` 时挂到 `openarm_right_hand`） | 改相机位置/分辨率 |
| `config/gazebo/controllers.yaml` | ★ 仿真控制器（5 个，`update_rate: 200`） | 加 / 调控制器 |
| `worlds/openarm_empty.sdf` | ★ 空世界（ground + sun + 4 个 system 插件） | 换场景 |
| `launch/gazebo_openarm.launch.py` | ★ Gazebo + RViz 一键启动 | 改启动流程 |
| `launch/display_openarm.launch.py` | 只 RViz + 假关节（无物理），默认 `arm_type=v20` | 改显示 |
| `rviz/bimanual.rviz` | **只有 Grid / RobotModel / TF**，没有 MotionPlanning | 加显示项 |
| `assets/robot/openarm_v2.0/**` | v2.0（多 `urdf/utils/`、`config/robot_presets/`） | 仅当用户明确要 v2.0 |
| `docs/PROJECT_MAP.md` | 人看的项目地图（AI 通常不用读） | 结构变化时同步 |
| `docs/gazebo_simulation_guide.md` | 仿真接入步骤 + 排查（918 行，v1.0 主线） | 流程出问题时按小节查 |
| `docs/AI_HANDOFF.md` | 本文件 | 每次重大变更后更新 §9/§10 |

**包外但同等重要**：

| 路径 | 职责 | 备注 |
|---|---|---|
| `robot_ws/src/armctrl_py_topic/`（**独立 ament_python 包**） | 用户自写的控制节点 `armctrl.py`：4 个 publisher，`ros2 run armctrl_py_topic armctrl` | 用户主线，问题清单见 §11 |

## 3. 三条链路（改任何东西前先看影响面）

### 3.1 URDF 组装链（v1.0 是 **3 层宏**，v2.0 只有 2 层）

```
openarm_v10.urdf.xacro                       ← 入口（sim arg / <gazebo> 插件）
└─ xacro:openarm_robot                       urdf/robot/openarm_robot.xacro
   ├─ xacro:openarm_body   ×1                urdf/body/*            ← config/body/*.yaml
   ├─ xacro:openarm_arm    ×2                urdf/arm/*             ← config/arm/*.yaml
   ├─ xacro:openarm_ee     ×2                urdf/ee/*              ← end_effector/parallel_link/config/*
   └─ xacro:openarm_bimanual_ros2_control     urdf/ros2_control/openarm.bimanual.ros2_control.xacro
```

**`sim` 必须逐层透传**：根 xacro → `openarm_robot` → `openarm_bimanual_ros2_control`。
只改第一层会报 `name 'sim' is not defined`。

### 3.2 仿真启动链（`launch/gazebo_openarm.launch.py`，**用事件不用 sleep**）

```
xacro(sim:=gz) ──► robot_state_publisher ──► /robot_description
gz sim worlds/openarm_empty.sdf  ──►  ros_gz_sim create -topic robot_description
   │                                        （spawn 进程退出事件）
   ├─► spawner joint_state_broadcaster
   ├─► spawner left_arm_controller + right_arm_controller
   ├─► spawner left_gripper_controller + right_gripper_controller
   └─► ros_gz_bridge /clock  +  rviz2 -d rviz/bimanual.rviz
```

Gazebo 冷启动约 8 s 后 5 个控制器全部 active。

### 3.3 控制链（Gazebo → ROS 是单向反馈）

```
用户节点 ──► /{l,r}_arm_controller/joint_trajectory (topic)  或  .../follow_joint_trajectory (action)
            /{l,r}_gripper_controller/commands (topic)
                     ↓ position command interface
            gz_ros2_control/GazeboSimSystem（跑在 gz sim 进程内）
                     ↓
                 Gazebo 物理
                     ↓
/joint_states ← joint_state_broadcaster（18 个关节，含 2 个 mimic 手指）
        ├─► robot_state_publisher ─► /tf /tf_static ─► RViz
        └─► 用户节点的订阅者
```

RViz **只读**：拖动 RViz 不会影响 Gazebo。

## 4. 接口清单（全部实测，可直接照抄）

### 4.1 节点（`ros2 node list`，共 9 个）

`/clock_bridge` `/controller_manager` `/gz_ros_control` `/joint_state_broadcaster`
`/left_arm_controller` `/right_arm_controller` `/left_gripper_controller` `/right_gripper_controller`
`/robot_state_publisher`

> 后 7 个里除 RSP 外都跑在 **gz sim 进程内部**（由 gz_ros2_control 创建），不是独立进程。

### 4.2 接口 + QoS

| 接口 | 类型 | 端点 | Reliability | Durability |
|---|---|---|---|---|
| `/{l,r}_arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | **订阅**（controller 收） | BEST_EFFORT | VOLATILE |
| `/{l,r}_arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | **action server** | — | — |
| `/{l,r}_arm_controller/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | 发布（controller 发） | RELIABLE | TRANSIENT_LOCAL |
| `/{l,r}_arm_controller/speed_scaling_input` | `control_msgs/msg/SpeedScalingFactor` | 订阅 | BEST_EFFORT | VOLATILE |
| `/{l,r}_gripper_controller/commands` | `std_msgs/msg/Float64MultiArray` | **订阅** | BEST_EFFORT | VOLATILE |
| `/joint_states` | `sensor_msgs/msg/JointState` | 发布（JSB） | RELIABLE | TRANSIENT_LOCAL |
| `/clock` | `rosgraph_msgs/msg/Clock` | 发布（bridge） | — | — |
| `/tf`、`/tf_static` | `tf2_msgs/msg/TFMessage` | 发布（RSP） | — | — |
| `/robot_description` | `std_msgs/msg/String` | 发布（RSP） | — | — |

**QoS 结论**：所有发布端用默认 QoS（RELIABLE / VOLATILE）都能连上，因为 DDS 只要求 offered ≥ requested。`/joint_states` 是 TRANSIENT_LOCAL，新订阅者能立刻收到最后一条。

### 4.3 查接口的固定套路

```bash
ros2 node info /right_arm_controller            # 某节点的全部接口
ros2 action list -t                              # action + 类型
ros2 topic info /joint_states -v                 # 类型 + QoS + 端点
ros2 interface show control_msgs/action/FollowJointTrajectory   # 完整字段
ros2 interface proto control_msgs/action/FollowJointTrajectory  # 生成可填的 YAML 模板
ros2 control list_controllers                    # 已装 ros2controlcli，可用
```

## 5. 关节、控制器、单位

| 控制器 | 类型 | 控制的关节 | 用户接口 |
|---|---|---|---|
| `joint_state_broadcaster` | JointStateBroadcaster | 全部 18 个（含 2 个 mimic 指） | 只发布 |
| `left_arm_controller` | JointTrajectoryController | `openarm_left_joint1..7` | action + topic |
| `right_arm_controller` | JointTrajectoryController | `openarm_right_joint1..7` | action + topic |
| `left_gripper_controller` | JointGroupPositionController | `openarm_left_finger_joint1` | **只有 topic** |
| `right_gripper_controller` | JointGroupPositionController | `openarm_right_finger_joint1` | **只有 topic** |

| 关节 | 类型 | 单位 / 范围 |
|---|---|---|
| `openarm_{left,right}_joint1..7` | revolute | **rad**（限位见 `config/arm/joint_limits.yaml`） |
| `openarm_{left,right}_finger_joint1` | **prismatic** | **米**，`0 ~ 0.044`，velocity 0.661，effort 150.75 |
| `openarm_{left,right}_finger_joint2` | prismatic + `<mimic>` | 同上；**不要给它发命令**。⚠️ 实测 gz-sim 8 下 mimic **不可靠**（多数情况停在 0），**别拿它做验收指标** |

- 整机规格：26 link / 25 joint（14 revolute + 4 prismatic + 7 fixed），`/joint_states` 里 **18 个**（7 固定关节不进 ros2_control）。
- `update_rate: 200`（`config/gazebo/controllers.yaml`）。
- 同一 URDF 里每个关节声明了 3 个 command interface（position/velocity/effort），但 **gz_ros2_control 只认第一个**，所以控制器统一用 `position`。

## 6. 参数速查

### launch（`gazebo_openarm.launch.py`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `arm_type` | `v10` | `v10/v1.0` 或 `v20/v2.0` |
| `robot_preset` | `default_bimanual` | **仅 v2.0 生效** |
| `world` | `openarm_empty.sdf` | `worlds/` 下文件名或绝对路径 |
| `gui` | `true` | `false` = `gz sim -s`（无窗口，CI 用） |
| `rviz` | `true` | 是否同时起 RViz2 |
| `camera` | `true` | 挂载手眼相机并桥接 `/hand_camera/image`（仅 v1.0，见 §14） |

### xacro

| 版本 | 必给参数 |
|---|---|
| v1.0 | `ros2_control:=true`（默认 false）、`bimanual:=true`、`sim:=gz`、`arm_type:=v10`、`body_type:=v10`、`use_fake_hardware:=false` |
| v2.0 | `robot_preset:=default_bimanual`、`sim:=gz`、`use_fake_hardware:=true` |

`sim` 只有两个值：`none`（默认，真机 / RViz 行为不变）、`gz`（Gazebo）。

## 7. 硬约束（违反必然出问题，改代码前先对照）

| # | 约束 | 违反后果 |
|---|---|---|
| 1 | 所有 xacro 的 `sim` 默认值保持 `none` | `display_openarm.launch.py` 与真机路径被污染 |
| 2 | `finger_joint2` **只能有 state interface** | 加 command interface 会让 gz_ros2_control 拒绝加载 |
| 3 | 控制器只用 `position` command interface | 多接口混用触发 [#343](https://github.com/ros-controls/gz_ros2_control/issues/343)，`controller_manager` 报接口冲突 |
| 4 | v1.0 必须显式 `ros2_control:=true` | 否则生成的 URDF 里根本没有 `<ros2_control>`，一个控制器都不会有 |
| 5 | 发臂轨迹必须覆盖该臂 **全部 7 个关节** | `allow_partial_joints_goal: false`，少给关节会直接不动（topic 方式不报错，极难查） |
| 6 | `use_sim_time` 必须为 true（RSP / RViz / 自写节点若用 TF） | TF 时间与 `/clock` 不匹配，RViz 报 "no transform" |
| 7 | 真机分支条件必须写 `or sim == 'gz'` | v1.0 的 `use_fake_hardware` 默认 false，会同时插入 `OpenArmHW` 去开 CAN 口，导致初始化失败 |
| 8 | 不要删除或重写 `openarm_v2.0/` | 用户会切换硬件版本 |

## 8. 3 分钟回归验证（改完必跑）

```bash
source /opt/ros/jazzy/setup.bash
cd /home/leo/space-arm_ws/robot_ws && source install/setup.bash
ros2 launch openarm_description gazebo_openarm.launch.py gui:=false rviz:=false
```

另一个终端跑冒烟测试（**topic 链路**，不依赖 action）：

```python
import time
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM = [f"openarm_left_joint{i}" for i in range(1, 8)]
rclpy.init()
n = Node("smoke")
cur = {}
n.create_subscription(
    JointState, "/joint_states",
    lambda m: cur.update(dict(zip(m.name, m.position))), 10)
pub_t = n.create_publisher(JointTrajectory, "/left_arm_controller/joint_trajectory", 10)
pub_g = n.create_publisher(Float64MultiArray, "/left_gripper_controller/commands", 10)


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)


spin(2)
print("joints:", len(cur))

# 等订阅端连上（BEST_EFFORT 无重传，太早发会丢）
while pub_t.get_subscription_count() == 0 or pub_g.get_subscription_count() == 0:
    rclpy.spin_once(n, timeout_sec=0.1)

tj = JointTrajectory()
tj.joint_names = ARM
p = JointTrajectoryPoint()
p.positions = [0.3, 0.0, 0.0, 0.4, 0.0, 0.0, 0.0]
p.time_from_start = Duration(sec=3)
tj.points = [p]
pub_t.publish(tj)
spin(5)
print("joint1=%.3f joint4=%.3f" % (cur["openarm_left_joint1"], cur["openarm_left_joint4"]))
g = Float64MultiArray()
g.data = [0.03]
pub_g.publish(g)
spin(3)
print("finger1=%.3f finger2=%.3f" % (
    cur["openarm_left_finger_joint1"], cur["openarm_left_finger_joint2"]))
rclpy.shutdown()
```

**期望输出**（这是验收线）：

```
joints: 18
joint1=0.300 joint4=0.400
finger1=0.030 finger2=-0.000
```

> ⚠️ `finger2` 是 mimic 手指，**gz-sim 8 下不可靠**（本次实测稳定停在 `-0.0000`，偶尔会跟随）。
> **验收只看 `finger1`**；`finger2` 不要写进断言，否则测试会偶发失败。
> 若必须要两指都动，唯一可靠办法是给 `finger_joint2` 也加 command interface 并让
> gripper 控制器控两个关节（需实测 gz_ros2_control 是否接受 mimic 关节的 command interface）。

其他基线：`ros2 control list_controllers` 5 个全 active；`ros2 topic hz /clock` ≈ 870 Hz（RTF≈0.87 属正常）。

## 9. 已知问题 / 现状（不要当成新 bug 去修）

| 项 | 状态 |
|---|---|
| `finger_joint2` 只有 state interface，不是 bug | gz-sim 8 对 URDF `<mimic>` 的支持**不可靠**，实测该关节长期停在 0。不要当成故障去修，也不要用它做断言 |
| `rviz/bimanual.rviz` 没有 MotionPlanning | 因为**没有 MoveIt**，属设计现状，不是 bug |
| `/clock` 约 870 Hz 而非 1000 | 本机 RTF < 1，正常 |
| commit `190f551` 里删除了 `.github/` 下 8 个文件 | 用户自己的提交，不是 AI 删的。要找回：`git checkout HEAD~1 -- robot_ws/src/openarm_description/.github` |
| `/tmp` 下的验证脚本会被清理 | 每次现写 |

## 10. 工作区状态

```
HEAD = ca49f11 "docs"（PROJECT_MAP / AI_HANDOFF / AI_PROMPT 首次提交）
     = 6bfbf77 "arm ctrl publisher"（新增 armctrl_py_topic 包）
     = f722e2a "v10 double arm with rviz gz2"（v1.0 仿真改动）
     = 190f551 "rviz + gz v02"（v2.0 仿真支持）

当前未提交（手眼相机功能，见 §14）：
  M  assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro     （camera arg + 挂载调用）
  M  launch/gazebo_openarm.launch.py                            （camera 参数 + camera_bridge）
  M  docs/AI_HANDOFF.md
  M  docs/PROJECT_MAP.md
  ?? assets/sensor/camera/                                      （新文件：openarm_hand_camera.xacro）
```

- `robot_ws/src/` 下现在有 **2 个包**：`openarm_description`、`armctrl_py_topic`。
- 构建全部：`colcon build`（不加 `--packages-select`）。
- ⚠️ 新增/修改 `assets/` 下的文件后**必须重新 build**：`install(DIRECTORY)` 是复制，不是符号链接。

## 11. 用户已有的控制节点（`armctrl_py_topic`）

独立包 `robot_ws/src/armctrl_py_topic/`（commit `6bfbf77`），入口：

```bash
ros2 run armctrl_py_topic armctrl          # 需要先 colcon build 整个工作空间
```

`armctrl_py_topic/armctrl.py` 现状：4 个 publisher（左/右臂 `joint_trajectory` + 左/右夹爪 `commands`），
1 Hz 定时器**重复发布**同一目标；**没有订阅者**。

### 接口设计（正确）

| 方向 | 接口 | 类型 |
|---|---|---|
| 发布 | `/{l,r}_arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` |
| 发布 | `/{l,r}_gripper_controller/commands` | `std_msgs/msg/Float64MultiArray` |
| 订阅（缺，建议加） | `/joint_states` | `sensor_msgs/msg/JointState` |
| 订阅（可选） | `/{l,r}_arm_controller/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` |

### 已发现的 4 个问题（接手先看这里）

| # | 问题 | 影响 | 修法 |
|---|---|---|---|
| 1 | 夹爪 `Rgripper_msg.data = [0.5]` | v1.0 手指行程 **0~0.044 m**，0.5 超限约 10 倍 | 改成 `0.03` 左右 |
| 2 | 1 Hz 定时器重发同一条 `time_from_start=2s` 轨迹 | 每 1 s 把轨迹起点重置一次 → 永远"刚起步"，末端缓慢爬行/过冲，日志每秒刷屏 | **只在目标变化时发一次**；或把定时器周期拉到 > 轨迹时间 |
| 3 | `msg.header.frame_id = "base_link"` | v1.0 没有 `base_link`（根是 `world`，机身是 `openarm_body_link0`）。JTC 忽略该字段，不报错但会误导 | 留空即可 |
| 4 | 没有订阅 `/joint_states` | 无法判断是否到位，也无法实现"只动一个关节、其余保持" | 见 §8 的订阅写法 |

臂目标 `[1.0, 0.5, -0.3, 1.0, 0.5, 0.0, 0.0]` 已核对**在 v1.0 限位内**
（joint1 `-1.396~3.491`、joint2 `±1.745`、joint3 `±1.571`、joint4 `0~2.443`），可以使用。

### 三个必知行为（topic 版）

1. topic 发轨迹是**覆盖式**（新消息替换当前目标），不是排队；**不需要高频重发**，JTC 会执行完。
2. 必须**覆盖该臂全部 7 个关节**（`allow_partial_joints_goal: false`），少给不会报错但不动。
3. 夹爪 `commands` 订阅端是 **BEST_EFFORT + 无重传**，脚本刚起就发可能丢；先等
   `publisher.get_subscription_count() > 0` 或连发 0.5 s。

### 下一步建议顺序

1. 修问题 1（夹爪单位）和 2（不要重复发）；
2. 加 `/joint_states` 订阅 + 到位判断；
3. 用 §8 的冒烟脚本对照验收；
4. 之后才考虑 MoveIt（当前项目**没有**，见 §0）。

## 12. 常用命令速查

```bash
source /opt/ros/jazzy/setup.bash && source /home/leo/space-arm_ws/robot_ws/install/setup.bash

# 构建（工作空间现在有 2 个包：openarm_description + armctrl_py_topic）
cd /home/leo/space-arm_ws/robot_ws && colcon build --symlink-install
# 只改 description 时：
colcon build --packages-select openarm_description --symlink-install

# 启动
ros2 launch openarm_description gazebo_openarm.launch.py                        # Gazebo + RViz
ros2 launch openarm_description gazebo_openarm.launch.py gui:=false rviz:=false # headless
ros2 launch openarm_description gazebo_openarm.launch.py arm_type:=v20          # v2.0
ros2 launch openarm_description display_openarm.launch.py                       # 只 RViz（假关节）

# 状态
ros2 control list_controllers
ros2 control list_hardware_components
ros2 node list
ros2 topic hz /clock
ros2 topic echo /joint_states --once
ros2 topic echo /right_arm_controller/controller_state --once

# 手发一条轨迹（topic）
ros2 topic pub --once /left_arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [openarm_left_joint1,openarm_left_joint2,openarm_left_joint3,openarm_left_joint4,openarm_left_joint5,openarm_left_joint6,openarm_left_joint7], points: [{positions: [0.3,0,0,0.4,0,0,0], time_from_start: {sec: 3}}]}"

# 夹爪（v10：米，0~0.044）
ros2 topic pub --once /right_gripper_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.03]}"

# 只展开 URDF 看结果
X=install/openarm_description/share/openarm_description/assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro
xacro $X ros2_control:=true sim:=gz | grep -cE "GazeboSimSystem"    # 期望 2
```

## 13. 文档路由（别乱读）

| 文档 | 什么时候读 |
|---|---|
| `docs/AI_HANDOFF.md` | **本文件**，开工必读 |
| `docs/AI_PROMPT.md` | 用户给新会话复制提示词用 |
| `docs/PROJECT_MAP.md` | 需要「功能 → 文件」速查时（人看的） |
| `docs/gazebo_simulation_guide.md` | 仿真流程出问题、要复现接入步骤时才按小节查（918 行，别整篇读） |

## 14. 手眼相机（Gazebo 专用）

**只在 `camera:=true`（launch 默认）时挂载**，`sim` 默认 `none` 时完全不出现，不影响真机 `robot_description`。

| 项 | 值 |
|---|---|
| 宏文件 | `assets/sensor/camera/openarm_hand_camera.xacro`（宏名 `openarm_hand_camera`） |
| 挂载点 | `openarm_right_hand`，`xyz="0 0 0.12"`，`rpy="0 -1.5708 0"`，`hfov=1.0` |
| 为什么 rpy 不是 0 | gz camera 光轴沿 link 的 **+X**；v1.0 的 hand 是 **+Z 朝前**，所以绕 Y 转 −90° |
| 为什么 xyz 是 0.12 | `0.06` 会正好卡在两根手指之间（实测画面两侧全是手指）；指端约 0.059，前伸到 0.12 才无遮挡 |
| 图像 | 640×480 `rgb8`，`update_rate=15`（实测 ~10 Hz） |

**gz 话题**（由 sensor 的 `<topic>` 固定，不随 model 路径变化）：
`/hand_camera/image`、`/hand_camera/camera_info`

**ROS 桥接**：launch 里的 `camera_bridge` 把它们桥成 `sensor_msgs/msg/Image` /
`sensor_msgs/msg/CameraInfo`，只在 `camera:=true` 时启动。

### 怎么看画面（三种，任选）

```bash
# 1) 独立窗口，最省事（无需 Gazebo GUI）
ros2 run rqt_image_view rqt_image_view        # 下拉选 /hand_camera/image
```

2) RViz 里 `Add → Image → Topic: /hand_camera/image`（与机械臂同窗口）
3) Gazebo GUI 里 `Plugins → Image Display → Topic: /hand_camera/image`（gz 侧，不经 ROS）

### 关键点

- `gui:=false`（headless）下相机**仍然出图**（gz-sim 8 的 ogre2 headless 渲染可用），实测 ~10 Hz。
- 图像话题 QoS 是 **sensor data（BEST_EFFORT）**：自写订阅者必须用
  `qos_profile_sensor_data`，否则一帧都收不到。
- 相机随末端运动（手眼已验证）：发右臂轨迹后画面随姿态变化。
- 当前 world 只有地面 + 机器人，所以画面多是灰色地面；要看"东西"就往
  `worlds/openarm_empty.sdf` 里加模型，或用 Gazebo 的 Insert 面板。
- 想改装到腕部：改根 xacro 里的 `parent_link="openarm_right_link7"` 并相应调 `xyz`。
- 关掉相机：`ros2 launch ... camera:=false`（同时不再挂载 link 和桥接）。



