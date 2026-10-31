# OpenArm v1.0 Gazebo 仿真接入指南（v2.0 兼容）

> 适用环境：Ubuntu 24.04 + **ROS 2 Jazzy** + **Gazebo Harmonic（gz-sim 8）**
> 适用仓库：`/home/leo/space-arm_ws/robot_ws/src/openarm_description`
> **本仓库实物是 v1.0（`v10`），本文主线按 v1.0 写**；v2.0（`v20`）的差异见附录 A。
> 目标：在 Gazebo 里跑 OpenArm v1.0（双臂 + 平行夹爪），用 `ros2_control` 控制器 / ROS action 控制 16 个主动关节（每臂 7 个 + 每手 1 个夹爪关节），并与 RViz 同步。

---

## 0. 先弄清楚：现在这套 launch 在干什么

`launch/display_openarm.launch.py` 只有三个节点：

```
xacro.process_file()  →  robot_state_publisher（发 /tf）
joint_state_publisher_gui（GUI 滑块发假 /joint_states）
rviz2（-d rviz/bimanual.rviz，Fixed Frame = world）
```

它**没有** Gazebo、没有 `controller_manager`、没有 `/clock`、没有物理。
所以 URDF 里的 `<ros2_control>`（当前是 `mock_components/GenericSystem`）只是一段“文字”，不会被加载。

实测确认（本机，v1.0）：

```bash
xacro openarm_v10.urdf.xacro ros2_control:=true
# → 26 个 link、25 个 joint：
#     · 14 个 revolute   = 每臂 joint1~joint7
#     · 4 个 prismatic   = 每手 finger_joint1 / finger_joint2
#     · 7 个 fixed       = 机身结构 + 把底座连到 world
# → openarm_{left,right}_finger_joint2 带 <mimic>（multiplier=1，行程 0~0.044 m）
# → ros2_control 默认 false，必须 ros2_control:=true 才会生成 left/right 两个 <ros2_control>
#   （默认 plugin = mock_components/GenericSystem）
```

也就是说：**16 个主动关节**（14 臂 + 2 个 `finger_joint1`）+ 2 个 mimic 指。

要在 Gazebo 里动起来，需要把 `<ros2_control>` 的 hardware plugin 换成
`gz_ros2_control/GazeboSimSystem`，并在 URDF 里加一个 gz 系统插件，让 Gazebo 内部起
`controller_manager`。这就是下面的改造。

### 改动清单（共 8 处）

| # | 文件 | 类型 | 作用 |
|---|---|---|---|
| 1 | `assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro` | 修改 | 加 `sim` 参数 + `<gazebo>` 插件块 |
| 2 | `assets/robot/openarm_v1.0/urdf/robot/openarm_robot.xacro` | 修改 | 把 `sim` 透传给 ros2_control 宏（v2.0 没有这一层） |
| 3 | `assets/robot/openarm_v1.0/urdf/ros2_control/openarm.bimanual.ros2_control.xacro` | 修改 | hardware plugin 条件切换 + mimic 指只读状态接口 |
| 4 | `assets/robot/openarm_v1.0/urdf/ros2_control/openarm.ros2_control.xacro` | 修改 | 单臂宏同样处理（`bimanual:=false` 时用） |
| 5 | `config/gazebo/controllers.yaml` | 新增 | controller_manager 与 4 个控制器配置（两版关节同名，共用） |
| 6 | `worlds/openarm_empty.sdf` | 新增 | 带地面/阳光/物理的空世界 |
| 7 | `launch/gazebo_openarm.launch.py` | 新增 | 一键起 gz + 机器人 + 控制器（`arm_type:=v10` 默认） |
| 8 | `CMakeLists.txt` / `package.xml` | 修改 | 安装新目录、声明运行依赖 |

改完后：`display_openarm.launch.py` 行为**完全不变**（`sim` 默认 `none`），Gazebo 走新 launch。

---

## 1. Step 0：安装依赖

### 1.1 关掉 conda（很重要）

`~/.bashrc` 里有 miniconda。conda 的 `LD_LIBRARY_PATH` / Qt / protobuf 会跟 Gazebo、RViz 打架。

```bash
conda deactivate 2>/dev/null || true
```

建议每开一个新终端都先执行这一句，或把 conda 的自动激活关掉（`conda config --set auto_activate_base false`）。

### 1.2 安装 Gazebo 与 ros2_control

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros-gz-sim-demos \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager
```

各包作用：

| apt 包 | 提供 |
|---|---|
| `ros-jazzy-ros-gz` | `ros_gz_sim`、`ros_gz_bridge` + Gazebo Harmonic（`gz sim`） |
| `ros-jazzy-gz-ros2-control` | `gz_ros2_control/GazeboSimSystem`、`libgz_ros2_control-system.so` |
| `ros-jazzy-ros2-controllers` | `joint_state_broadcaster`、`joint_trajectory_controller`、`position_controllers` |
| `ros-jazzy-controller-manager` | `controller_manager` 节点 / `spawner` 命令行 |

> 不需要手动装 `gz-sim8`：ROS 2 Jazzy 的 `ros_gz` 用 vendor 包自带 Harmonic。

### 1.3 自检

```bash
source /opt/ros/jazzy/setup.bash
gz sim --versions                    # 期望输出 8.x
ros2 pkg prefix ros_gz_sim           # 有输出即 OK
ros2 pkg prefix gz_ros2_control      # 有输出即 OK
ros2 pkg prefix joint_trajectory_controller
```

先单独验证 Gazebo 能启动（会打开一个空白世界，确认能关掉再继续）：

```bash
gz sim -r empty.sdf
# 或 headless：
# gz sim -s -r empty.sdf
```

> 如果 GUI 黑屏/崩溃：`LIBGL_ALWAYS_SOFTWARE=1 gz sim -r empty.sdf`，或直接用 `-s`。

---

## 2. Step 1：改 `openarm_v10.urdf.xacro`（加 `sim` 开关）

文件：
`robot_ws/src/openarm_description/assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro`

改动 3 个地方。

### 2.1 声明 `sim` 参数并暴露成 property

`<xacro:arg>` 不能直接在 `${...}` 里当变量用，所以要多写一行 `property`：

```xml
  <xacro:arg name="can_fd" default="true" />

  <!-- 新增：none = 保持原行为（mock_components / 真机），gz = Gazebo 仿真 -->
  <xacro:arg name="sim" default="none" />
  <xacro:property name="sim" value="$(arg sim)" />
```

### 2.2 把 `sim` 传给 `openarm_robot` 宏

v1.0 的根文件只有一个 `<xacro:openarm_robot ... />` 调用，参数列表末尾加一行：

```xml
  can_fd="$(arg can_fd)"
  sim="$(arg sim)"
  />
```

> v1.0 比 v2.0 多一层 `openarm_robot` 宏，`sim` 必须**逐层透传**：
> `openarm_v10.urdf.xacro` → `openarm_robot` → `openarm_bimanual_ros2_control`。
> 只改第一层会报 `name 'sim' is not defined`。

### 2.3 加 gz 系统插件（加在文件末尾 `</robot>` 之前）

```xml
  <xacro:if value="${sim == 'gz'}">
    <gazebo>
      <!-- Gazebo 内部由它起 controller_manager -->
      <plugin filename="libgz_ros2_control-system.so"
              name="gz_ros2_control::GazeboSimROS2ControlPlugin">
        <parameters>$(find openarm_description)/config/gazebo/controllers.yaml</parameters>
        <ros>
          <namespace>/</namespace>
        </ros>
      </plugin>
    </gazebo>
  </xacro:if>

</robot>
```

> `<parameters>` 里用的是 `$(find openarm_description)`，xacro 会把它展开成**安装目录**的绝对路径
> （`robot_ws/install/openarm_description/share/openarm_description/config/gazebo/controllers.yaml`），
> 所以 Step 5 必须把 `config` 目录安装进去，否则启动时找不到文件。

---


## 3. Step 1（续）：改 v1.0 的 ros2_control 宏

v1.0 有两个 ros2_control 宏文件，都要改：

| 文件 | 宏 | 用途 |
|---|---|---|
| `.../urdf/ros2_control/openarm.bimanual.ros2_control.xacro` | `openarm_bimanual_ros2_control` | 双臂（`bimanual:=true`，默认） |
| `.../urdf/ros2_control/openarm.ros2_control.xacro` | `openarm_arm_ros2_control` | 单臂（`bimanual:=false`） |

### 3.1 宏签名加 `sim` 参数

两个文件的参数表末尾（`can_fd:=^|true` 之后）都加一行：

```xml
             can_fd:=^|true
             sim:=^|none">
```

### 3.2 hardware 块：改成三选一

比"整块再包一层 `<xacro:unless>`"更省事、也更好读的写法是**给两个已有分支各加一个 `sim != 'gz'` 条件**，
再在最前面插一个 gz 分支。左右两侧同样处理（`left_` ↔ `right_`、`left_can_interface` ↔ `right_can_interface`）：

```xml
    <ros2_control name="openarm_left_hardware_interface" type="system">
        <hardware>
          <param name="arm_type">${arm_type}</param>

          <!-- 新增：仿真分支 -->
          <xacro:if value="${sim == 'gz'}">
            <plugin>gz_ros2_control/GazeboSimSystem</plugin>
          </xacro:if>

          <!-- 原有 mock 分支：加 and sim != 'gz' -->
          <xacro:if value="${use_fake_hardware and sim != 'gz'}">
            <plugin>mock_components/GenericSystem</plugin>
            ...
          </xacro:if>

          <!-- 原有真机分支：加 or sim == 'gz' -->
          <xacro:unless value="${use_fake_hardware or sim == 'gz'}">
            <plugin>openarm_hardware/OpenArmHW</plugin>
            ...
          </xacro:unless>
        </hardware>
```

> **为什么真机分支也必须加 `or sim == 'gz'`**：v1.0 的 `use_fake_hardware` 默认是 `false`。
> 如果只改 mock 分支，`sim:=gz` 会同时插入 `GazeboSimSystem` 和 `OpenArmHW`，
> 后者会去打开 CAN 口，直接让 `controller_manager` 初始化失败。

单臂宏（`openarm.ros2_control.xacro`）里两个分支的条件写法不同，改成：

```xml
          <xacro:if value="${use_fake_hardware and sim != 'gz'}">        <!-- mock -->
          <xacro:if value="${use_fake_hardware == 0 and sim != 'gz'}">   <!-- 真机 -->
```

### 3.4 给 mimic 指补"只读"状态接口

v1.0 的 ros2_control 里**原本没有** `finger_joint2`（只有 `finger_joint1`），
所以 joint_state_broadcaster 不会发布第二根手指 → RViz 里会缺关节。

gz_ros2_control 的规则：**mimic 关节不能有 command interface，但可以有 state interface**。
在左右两侧 `finger_joint1` 的 `<xacro:if value="${hand}">` 块里各补一个：

```xml
      <xacro:if value="${hand}">
        <xacro:configure_joint joint_name="openarm_${left_arm_prefix}finger_joint1" initial_position="0.0" />

        <!-- mimic 关节：只声明状态接口，绝对不要加 command_interface -->
        <joint name="openarm_${left_arm_prefix}finger_joint2">
          <state_interface name="position">
            <param name="initial_value">0.0</param>
          </state_interface>
        </joint>
      </xacro:if>
```

右侧把 `left_arm_prefix` 换成 `right_arm_prefix`。

> **实测结论（v10）**：gz-sim 8 **会**执行 SDF 里的 `<mimic>`（URDF 转 SDF 后生成
> `<mimic joint="openarm_left_finger_joint1"><multiplier>1</multiplier>...`）。
> 给 `finger_joint1` 发 0.03，`joint_states` 里 `finger_joint1 = 0.0300`、
> `finger_joint2 = 0.0300`，两指同步。
> 所以**不需要**给第二根手指发命令，也不要给它加 command interface。

### 3.5 让 `sim` 穿过 `openarm_robot` 宏（v1.0 特有）

`assets/robot/openarm_v1.0/urdf/robot/openarm_robot.xacro` 里做 3 件事：

1. 宏参数表末尾（`can_fd:='true'` 之后）加 `sim:='none'`；
2. 双臂分支的 `<xacro:openarm_bimanual_ros2_control .../>` 调用加 `sim="${sim}"`；
3. 单臂分支的 `<xacro:openarm_arm_ros2_control .../>` 调用也加 `sim="${sim}"`。

### 3.6 验证 xacro 输出

```bash
source /opt/ros/jazzy/setup.bash
source /home/leo/space-arm_ws/robot_ws/install/setup.bash

X=/home/leo/space-arm_ws/robot_ws/src/openarm_description/assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro

xacro $X ros2_control:=true sim:=gz \
  | grep -nE "GazeboSimSystem|gz_ros2_control-system|finger_joint2" | head -20
```

期望看到：

```
<plugin>gz_ros2_control/GazeboSimSystem</plugin>          ← 2 次（左右）
<plugin filename="libgz_ros2_control-system.so" ...>      ← 1 次
<joint name="openarm_left_finger_joint2">                 ← 带 state_interface
<joint name="openarm_right_finger_joint2">
```

再确认默认行为没被破坏（应输出 `OpenArmHW`，且没有 gz 插件）：

```bash
xacro $X ros2_control:=true sim:=none \
  | grep -cE "GazeboSimSystem|gz_ros2_control-system"
# 期望：0
```

---


## 4. Step 2：新增 `config/gazebo/controllers.yaml`

```bash
mkdir -p /home/leo/space-arm_ws/robot_ws/src/openarm_description/config/gazebo
```

文件内容（4 个控制器：双臂 JTC + 双夹爪位置控制器）：

```yaml
controller_manager:
  ros__parameters:
    # 必须带 use_sim_time，否则控制器里的时间戳和 /clock 对不上
    use_sim_time: true
    update_rate: 200  # Hz，与 Gazebo 物理步长同一量级

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    left_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    right_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    left_gripper_controller:
      type: position_controllers/JointGroupPositionController

    right_gripper_controller:
      type: position_controllers/JointGroupPositionController

left_arm_controller:
  ros__parameters:
    joints:
      - openarm_left_joint1
      - openarm_left_joint2
      - openarm_left_joint3
      - openarm_left_joint4
      - openarm_left_joint5
      - openarm_left_joint6
      - openarm_left_joint7
    command_interfaces:
      - position            # 只声明 position！见下方“为什么”
    state_interfaces:
      - position
      - velocity
    allow_partial_joints_goal: true
    open_loop_control: false
    # 与下方 URDF 里的 limit 相差不大即可
    constraints:
      stopped_velocity_tolerance: 0.05
      goal_time: 1.0
      openarm_left_joint1: { trajectory: 0.5, goal: 0.1 }
      openarm_left_joint2: { trajectory: 0.5, goal: 0.1 }
      openarm_left_joint3: { trajectory: 0.5, goal: 0.1 }
      openarm_left_joint4: { trajectory: 0.5, goal: 0.1 }
      openarm_left_joint5: { trajectory: 0.5, goal: 0.1 }
      openarm_left_joint6: { trajectory: 0.5, goal: 0.1 }
      openarm_left_joint7: { trajectory: 0.5, goal: 0.1 }

right_arm_controller:
  ros__parameters:
    joints:
      - openarm_right_joint1
      - openarm_right_joint2
      - openarm_right_joint3
      - openarm_right_joint4
      - openarm_right_joint5
      - openarm_right_joint6
      - openarm_right_joint7
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    allow_partial_joints_goal: true
    open_loop_control: false
    constraints:
      stopped_velocity_tolerance: 0.05
      goal_time: 1.0
      openarm_right_joint1: { trajectory: 0.5, goal: 0.1 }
      openarm_right_joint2: { trajectory: 0.5, goal: 0.1 }
      openarm_right_joint3: { trajectory: 0.5, goal: 0.1 }
      openarm_right_joint4: { trajectory: 0.5, goal: 0.1 }
      openarm_right_joint5: { trajectory: 0.5, goal: 0.1 }
      openarm_right_joint6: { trajectory: 0.5, goal: 0.1 }
      openarm_right_joint7: { trajectory: 0.5, goal: 0.1 }

left_gripper_controller:
  ros__parameters:
    joints:
      - openarm_left_finger_joint1
    # 这里是米/弧度级的“位置”指令；夹爪夹持力由 Gazebo 接触物理决定

right_gripper_controller:
  ros__parameters:
    joints:
      - openarm_right_finger_joint1
```

### 为什么只用 `position` command interface

URDF 的 `configure_joint` 宏里同时声明了 `position` / `velocity` / `effort` 三个 command interface。
而 gz_ros2_control 已知问题（[gz_ros2_control#343](https://github.com/ros-controls/gz_ros2_control/issues/343)）：
同一个关节同时声明多个 command interface 时，`GazeboSimSystem` 只认第一个，
但**多个控制器各用不同接口**会报 `Could not activate controller` 或 `interface is already claimed`。

所以本方案约定：**仿真模式下所有控制器只用 `position`**。
如果你确实要 velocity/effort 控制器，必须先去 URDF 里把多余的两个 `command_interface` 删掉（按 `sim` 分支处理）。

---

## 5. Step 3：新增 `worlds/openarm_empty.sdf`

```bash
mkdir -p /home/leo/space-arm_ws/robot_ws/src/openarm_description/worlds
```

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="openarm">

    <!-- 物理引擎 -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- 关键的 4 个系统插件：缺一个都会“看起来能动但其实不仿真” -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <gravity>0 0 -9.81</gravity>

    <!-- 地面 -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.85 0.85 0.85 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- 阳光 -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.3 -0.9</direction>
    </light>

  </world>
</sdf>
```

> 注意：世界名是 `openarm`，所以 Gazebo 的 `create` 服务是 `/world/openarm/create`。
> 本文档的 launch 里用 `ros_gz_sim create` 自动找默认世界，不需要手动指定。
> 机器人底座在 URDF 里通过固定 `mount` 关节焊到 `world`，所以它是**固定不动**的（台架式机械臂，符合预期）。

---


## 6. Step 4：新增 `launch/gazebo_openarm.launch.py`

完整实现见仓库真实文件（约 240 行，本文不再复制全文，避免与代码不同步）：

`robot_ws/src/openarm_description/launch/gazebo_openarm.launch.py`

核心结构：

```python
def launch_setup(context, *args, **kwargs):
    # 1) 按 arm_type 选 xacro，并用 sim:=gz 展开
    folder_name, file_name = resolve_arm_config(arm_type)   # v10 → openarm_v1.0/openarm_v10.urdf.xacro
    if folder_name == "openarm_v1.0":
        mappings = {
            "arm_type": "v10", "body_type": "v10", "bimanual": "true",
            "ros2_control": "true",        # v1.0 默认 false，仿真必须显式打开
            "use_fake_hardware": "false", "fake_sensor_commands": "false",
            "sim": "gz",
        }
    else:
        mappings = {"robot_preset": robot_preset, "sim": "gz"}
    robot_description = xacro.process_file(xacro_path, mappings=mappings).toprettyxml(indent="  ")

    # 2) gz sim（gui:=false 时加 -s 走 headless）
    gz_args = f"-r -v 4 {world_path}" if gui else f"-s -r -v 4 {world_path}"

    # 3) /clock 桥接：不桥接的话所有 use_sim_time 节点时间停在 0
    clock_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"])

    # 4) 从 /robot_description 话题 spawn 模型
    spawn_robot = Node(package="ros_gz_sim", executable="create",
                       arguments=["-topic", "robot_description", "-name", "openarm",
                                  "-x", "0.0", "-y", "0.0", "-z", "0.0"])

    # 5) 事件链：spawn 退出 → JSB → 双臂 → 双夹爪（不要用 sleep）
    RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[jsb]))
    RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[left_arm, right_arm]))
    RegisterEventHandler(OnProcessExit(target_action=left_arm,
                                       on_exit=[left_gripper, right_gripper]))
```

启动参数（全部有默认值）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `arm_type` | `v10` | `v10/v1.0` 或 `v20/v2.0` |
| `robot_preset` | `default_bimanual` | 仅 v2.0 生效 |
| `world` | `openarm_empty.sdf` | `worlds/` 下的文件名，或绝对路径 |
| `gui` | `true` | `false` = `gz sim -s`（服务器 / CI） |
| `rviz` | `true` | 是否同时起 RViz2 |

### 关键点（踩坑清单）

| 点 | 说明 |
|---|---|
| `GZ_SIM_RESOURCE_PATH` | 设成 `share/`（`pkg_share` 的父目录），否则 `package://openarm_description/meshes/...` 解析失败 → 模型全是白块 |
| spawner 超时 | `--controller-manager-timeout 60`，Gazebo 冷启动慢也不会失败 |
| `OnProcessExit` 链 | 用事件而不是 `sleep`：spawn → JSB → 臂 → 夹爪 |
| `use_sim_time` | RSP 和 RViz 都要开，否则 TF 时间与 `/clock` 不匹配 |
| v1.0 的 `ros2_control` | 必须显式传 `ros2_control:=true`，否则生成的 URDF 里**根本没有** `<ros2_control>` |

---

## 7. Step 5：改 `CMakeLists.txt` 与 `package.xml`

### 7.1 `CMakeLists.txt` 第 32 行

```diff
-foreach(install_dir assets launch rviz scripts)
+foreach(install_dir assets config launch rviz worlds scripts)
```

（`worlds` 和 `config` 目录存在才会安装，`if(EXISTS ...)` 已经做了保护。）

这一步至关重要：`$(find openarm_description)/config/gazebo/controllers.yaml`
和 `worlds/openarm_empty.sdf` 都在安装后的 share 目录里被引用。

### 7.2 `package.xml`：在第 31~37 行依赖块里补 4 行

```xml
  <depend>joint_state_publisher_gui</depend>
  <depend>ros_gz</depend>
  <depend>xacro</depend>
  <depend>rviz2</depend>
  <depend>robot_state_publisher</depend>
  <depend>joint_state_publisher</depend>
  <depend>zed_description</depend>

  <!-- 新增：Gazebo 仿真链路 -->
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>gz_ros2_control</exec_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>joint_trajectory_controller</exec_depend>
  <exec_depend>position_controllers</exec_depend>
```

---

## 8. Step 6：编译并启动

### 8.1 编译

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
cd /home/leo/space-arm_ws/robot_ws

colcon build --symlink-install --packages-select openarm_description
source install/setup.bash
```

> 本仓库的 `install/` 默认是 **COPY 模式**（不是符号链接）。即使你不加
> `--symlink-install` 也必须重新 build，否则改动的 xacro/config 不会进入 install 目录。
> 加上 `--symlink-install` 之后，改 yaml / launch 就不用再 build 了（xacro 也是软链）。

确认文件已安装：

```bash
ls install/openarm_description/share/openarm_description/config/gazebo/
ls install/openarm_description/share/openarm_description/worlds/
ls install/openarm_description/share/openarm_description/launch/ | grep gazebo
```

### 8.2 启动

```bash
source /home/leo/space-arm_ws/robot_ws/install/setup.bash
ros2 launch openarm_description gazebo_openarm.launch.py
```

常用变体：

```bash
# 只开 Gazebo，不开 RViz
ros2 launch openarm_description gazebo_openarm.launch.py rviz:=false

# 无 GUI（服务器 / CI）：gui:=false 等价于 gz sim -s
ros2 launch openarm_description gazebo_openarm.launch.py gui:=false rviz:=false

# 换世界
ros2 launch openarm_description gazebo_openarm.launch.py \
  world:=/absolute/path/to/your.sdf

# 跑 v2.0 硬件（本仓库默认是 v10）
ros2 launch openarm_description gazebo_openarm.launch.py arm_type:=v20
```

启动后期望看到（按时间顺序）：

```
[gz_sim.launch.py] ... Gazebo 窗口打开，地面 + 机械臂出现在原点
[spawn_openarm]  ... Successfully spawned entity [openarm]
[spawner_joint_state_broadcaster] ... Configured and activated joint_state_broadcaster
[spawner_left_arm_controller]     ... Configured and activated left_arm_controller
[spawner_right_arm_controller]    ...
[spawner_left_gripper_controller] ...
[spawner_right_gripper_controller]...
```

---


## 9. Step 7：验证（按顺序做，哪一步失败就直接看 Step 8 对应行）

新开一个终端：

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source /home/leo/space-arm_ws/robot_ws/install/setup.bash
```

### 9.1 时钟在跑

```bash
ros2 topic hz /clock
# 期望：average rate 约 1000（物理步长 1ms）
```

如果 `/clock` 没有数据 → `clock_bridge` 没起或桥接参数写错（Step 8 #7）。

### 9.2 硬件接口被 gz 接管

```bash
ros2 control list_hardware_components
# 期望两行 component：
#   openarm_left_hardware_interface   [gz_ros2_control/GazeboSimSystem]  active
#   openarm_right_hardware_interface  [gz_ros2_control/GazeboSimSystem]  active

ros2 control list_hardware_interfaces | head -30
# 期望形如：
#   joint/openarm_left_joint1/position    [available] [claimed]
#   joint/openarm_left_joint1/velocity    [available] [unclaimed]
#   joint/openarm_left_finger_joint2/position [available] [unclaimed]  ← mimic 只有 state
```

**如果这里什么都看不到，说明 `gz_ros2_control` 插件没加载**，别往下走，看 Step 8 #1/#2。

### 9.3 控制器全部 active

```bash
ros2 control list_controllers
# 期望 5 行全部 active：
#   joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
#   left_arm_controller[joint_trajectory_controller/JointTrajectoryController] active
#   right_arm_controller[...] active
#   left_gripper_controller[position_controllers/JointGroupPositionController] active
#   right_gripper_controller[...] active
```

### 9.4 关节状态反馈正常（含 mimic 指）

```bash
ros2 topic echo /joint_states --once | grep -E "name|position" -A 3
```

应包含 **18 个关节名**（每侧 9 个：`joint1..joint7` + `finger_joint1` + `finger_joint2`）。
`finger_joint2` 出现在这里，就说明 Step 3.4 的 state interface 生效了（否则 RViz 里第二根手指会不动）。

### 9.4.1 Gazebo 里“能不能动”的快速判断

```bash
ros2 topic hz /joint_states        # 期望与 update_rate 一致（约 200 Hz）
```

### 9.5 动起来：发一条轨迹（左臂 joint1 转 0.3 rad）

```bash
ros2 action send_goal /left_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
      joint_names: [openarm_left_joint1, openarm_left_joint2, openarm_left_joint3,
                    openarm_left_joint4, openarm_left_joint5, openarm_left_joint6,
                    openarm_left_joint7],
      points: [{positions: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]
   }}" --feedback
```

期望：Gazebo 里左臂明显抬起/转动，RViz 同步，action 返回
`result: {error_code: 0, error_string: ''}`，`--feedback` 里 `actual.positions` 逐渐逼近 0.3。

回零：

```bash
ros2 action send_goal /left_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
      joint_names: [openarm_left_joint1, openarm_left_joint2, openarm_left_joint3,
                    openarm_left_joint4, openarm_left_joint5, openarm_left_joint6,
                    openarm_left_joint7],
      points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]
   }}"
```

### 9.6 夹爪（+ 验证 mimic 联动）

v1.0 的手指是 **prismatic**（直线），命令单位是**米**，不是弧度。先看当前值，再发一个小位移：

```bash
ros2 topic echo /joint_states --once | grep -A 1 finger

ros2 topic pub --once /left_gripper_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.03]}"

# 看两指是否都在动
ros2 topic echo /joint_states --once | grep -A 1 finger
```

期望 `openarm_left_finger_joint1 = 0.03`，`openarm_left_finger_joint2` 同步变成 `0.03`。

本机实测这两个关节的配置（v1.0）：

```
limit: lower=0.0, upper=0.044 m, velocity=0.661, effort=150.75
<mimic joint="openarm_left_finger_joint1" multiplier="1" offset="0"/>
```

所以命令范围是 `0 ~ 0.044`（米），别发负数或超过 0.044，
否则控制器会报 `Goal position is out of bounds` 或直接饱和到限位。

> v2.0 的手指是 revolute，范围 `0 ~ 0.7854 rad`，命令单位是弧度，别照抄这里的 0.03。

### 9.7 Gazebo 侧确认（可选）

```bash
gz topic -l | head          # 有 /world/openarm/... 说明世界/模型都在
gz model -m openarm -p      # 打印模型位姿
```

---

## 10. Step 8：故障排查

| # | 现象 | 原因 | 处理 |
|---|---|---|---|
| 1 | `[gz_ros2_control] Failed to load plugin ... libgz_ros2_control-system.so` | 没装 `ros-jazzy-gz-ros2-control`，或 conda 的 `LD_LIBRARY_PATH` 污染 | `sudo apt install ros-jazzy-gz-ros2-control`；`conda deactivate`；`ldd` 检查 |
| 2 | 模型出现了但 `ros2 control list_hardware_components` 为空 / 报 `Controller manager not available` | URDF 里没有 `<gazebo><plugin>` 块（xacro 的 `sim` 没传到，或 launch 里没写 `sim: gz`） | `ros2 topic echo /robot_description --once \| grep gz_ros2_control-system` 应能搜到；搜不到重做 Step 1.3 |
| 3 | 报 `Unable to find file with URI [package://openarm_description/...]`，模型只剩几个方块 | `GZ_SIM_RESOURCE_PATH` 没设/设错层级 | 必须是 `.../share`（含 `openarm_description/meshes/...` 的那层）；在跑 `gz sim` 的同一 shell echo 一下确认 |
| 4 | `Could not find controller configuration file` / 参数全默认 | `config` 没安装进 share | 重做 Step 5.1 后 `colcon build`；`ls install/.../config/gazebo/` |
| 5 | `Could not activate controller ... interface is already claimed` | 同一关节多个 command interface 冲突（gz_ros2_control#343），或两个控制器抢同一关节 | 确认 `controllers.yaml` 里只有 `position`；确认左右控制器关节列表不重叠 |
| 6 | `spawner ... controller not found` | 该控制器没写在 yaml 的 `controller_manager.ros__parameters` 下，或实例名拼错 | 对照 Step 2 的 5 个 `type:` 条目 |
| 7 | RViz 里 `Message Filter dropping message ... earlier than all data in transform` / 时间戳 0 | 没 `use_sim_time` 或 `/clock` 没桥接 | 检查 `clock_bridge` 节点与各节点 `use_sim_time: true`；`ros2 param get /robot_state_publisher use_sim_time` |
| 8 | 关节软塌塌 / 重力下掉，控制器却显示 active | gz 系统插件缺失（physics/user-commands/scene-broadcaster），或 `position_proportional_gain` 太低 | 用 Step 3 的完整 world；把增益调到 0.5~1.0 |
| 9 | Gazebo GUI 黑屏 / 显卡报错 | 无 GPU / 驱动问题 | `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch ...` 或 headless（`gz_args` 加 `-s`） |
| 10 | `xacro: Invalid parameter "sim"` / `unexpected keyword` | 宏签名没加 `sim:=^|none` | 重做 Step 3.1 |
| 11 | 用 `right_arm` 这类单臂 preset 时，另一侧控制器激活失败 | yaml 里写了不存在的关节 | 用默认 `default_bimanual`，或复制一份 `controllers_single.yaml` 只留存在的一侧 |
| 12 | 第一次启动特别慢，spawner 抢跑 | Gazebo 首次加载慢 | 把 launch 里 `TimerAction(period=3.0)` 调到 5~8 |
| 13 | 改完 yaml 没生效 | `install/` 是 COPY 模式 | 重新 `colcon build`，或用 `--symlink-install` |

快速定位顺序：**看 gz 日志里有没有 `gz_ros2_control` → 看 `/clock` → 看 `list_hardware_components` → 看 `list_controllers`**，这四步基本能锁定问题所在。

---


## 附录 A：v2.0（`v20`）路径差异

`launch/gazebo_openarm.launch.py` 已经支持 `arm_type`，跑 v2.0 只需：

```bash
ros2 launch openarm_description gazebo_openarm.launch.py arm_type:=v20
```

两版的差别（`joint_states` 都是 18 条，`config/gazebo/controllers.yaml` 共用一份）：

| 项 | v1.0（本文主线） | v2.0 |
|---|---|---|
| 入口 xacro | `assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro` | `assets/robot/openarm_v2.0/urdf/openarm_v20.urdf.xacro` |
| 规格 | 26 link / 25 joint（14 revolute + 4 prismatic + 7 fixed） | 22 link / 21 joint（18 revolute + 3 fixed） |
| 宏层级 | 根 xacro → `openarm_robot` → `openarm_bimanual_ros2_control` | 根 xacro → `openarm_bimanual_ros2_control` |
| 关键 arg | `ros2_control:=true`（默认 `false`） | `robot_preset:=default_bimanual`（默认） |
| 手指 | prismatic，行程 `0 ~ 0.044 m` | revolute，行程 `0 ~ 0.7854 rad` |
| 末端 | `parallel_link`（固定） | `pinch_gripper`（可通过 preset 换） |

v2.0 需要的 xacro 改动与本文 Step 2/3 完全相同（`sim` arg + property、hardware 三选一、
`finger_joint2` 只留 state interface），只是路径换成 `openarm_v2.0/`，并且**少一层** `openarm_robot` 透传。

> 两个已知注意点：
> 1. v2.0 的 `finger_joint1` 是 **revolute**，夹爪命令单位是弧度。本机在 v2.0 上发 `0.6 rad`
>    时第二根手指没跟上；v1.0 的 prismatic 夹爪实测跟随正常。用 v2.0 时先小步验证
>    （`ros2 topic pub --once ... "{data: [0.2]}"`）。
> 2. v2.0 的 `<gazebo>` 插件块同样加在根 xacro 末尾 `</robot>` 之前。

---

## 附录 B：跑通之后可以继续做的

| 需求 | 做法 |
|---|---|
| 用键盘/手柄直接拖 | `ros2 run joint_state_publisher_gui` 改成发布到 `/left_arm_controller/commands`（Float64MultiArray + `JointGroupPositionController`） |
| 与 MoveIt 联调 | 把 `moveit_config` 里的 `ros2_control` 换成 `gz_ros2_control`，`use_sim_time: true`，并用 `ros2_control_node` 的 mock 版本换成 Gazebo 里的 CM（不要重复启动 CM） |
| 头部/相机 | `openarm_v20_sensor.xacro` 已引用 zed 描述；仿真需要 `ros_gz_bridge` 桥接 `camera_info` + `image`，并装 `ros-jazzy-zed-description` |
| 提高实时性 | `physics` 的 `max_step_size` 与 `controller_manager.update_rate` 保持 1:1 或 1:2；`real_time_factor` 设 0 表示不限速 |
| 记录/回放 | `ros2 bag record /joint_states /clock /tf` |

---

## 附录 C：一键还原

所有改动都在 `robot_ws/src/openarm_description`，用 git 可以干净回退：

```bash
cd /home/leo/space-arm_ws/robot_ws/src/openarm_description
git status
git diff -- assets/robot/openarm_v1.0/urdf
git checkout -- assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro \
                assets/robot/openarm_v1.0/urdf/robot/openarm_robot.xacro \
                assets/robot/openarm_v1.0/urdf/ros2_control/openarm.bimanual.ros2_control.xacro \
                assets/robot/openarm_v1.0/urdf/ros2_control/openarm.ros2_control.xacro \
                CMakeLists.txt package.xml
rm -rf config/gazebo worlds launch/gazebo_openarm.launch.py
cd /home/leo/space-arm_ws/robot_ws && colcon build --packages-select openarm_description
```

因为 `sim` 默认值是 `none`，**即使不改回来**，`display_openarm.launch.py`
和真机/`mock_components` 路径也完全不受影响——这是本方案刻意保留的安全默认值。

---

## 附录 D：命令速查

```bash
# 只生成 URDF 看结果
X=<pkg>/assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro
xacro $X ros2_control:=true sim:=gz > /tmp/openarm_gz.urdf
check_urdf /tmp/openarm_gz.urdf          # 需要 liburdfdom-tools

# 启动（默认 arm_type=v10）
ros2 launch openarm_description gazebo_openarm.launch.py
ros2 launch openarm_description gazebo_openarm.launch.py rviz:=false
ros2 launch openarm_description gazebo_openarm.launch.py gui:=false rviz:=false
ros2 launch openarm_description gazebo_openarm.launch.py arm_type:=v20

# 状态（本机没装 ros2controlcli，用 service 等价替代）
ros2 service call /controller_manager/list_hardware_components \
  controller_manager_msgs/srv/ListHardwareComponents "{}"
ros2 service call /controller_manager/list_hardware_interfaces \
  controller_manager_msgs/srv/ListHardwareInterfaces "{}"
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers "{}"

# 手动装卸控制器
ros2 service call /controller_manager/load_controller \
  controller_manager_msgs/srv/LoadController "{name: left_arm_controller}"
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: [left_arm_controller], strictness: 2}"

# 时钟 / 关节
ros2 topic hz /clock
ros2 topic echo /joint_states --once

# Gazebo CLI
gz topic -l
gz model -m openarm -p
gz service -l | grep create
```

---

## 附录 E：最终 checklist

- [ ] Step 0：`gz sim --version` 输出 8.x；`ls /opt/ros/jazzy/lib | grep gz_ros2_control` 有 `libgz_ros2_control-system.so`
- [ ] Step 1：`xacro $X ros2_control:=true sim:=gz | grep -c GazeboSimSystem` = 2
- [ ] Step 1：同一命令里 `grep -c "gz_ros2_control-system"` = 1
- [ ] Step 1：`sim:=none` 时上面两个计数都是 0，且 `OpenArmHW` = 2
- [ ] Step 2：`finger_joint2` 只有 `state_interface`，没有 `command_interface`
- [ ] Step 3：`controllers.yaml` 里控制器只用 `position` command interface
- [ ] Step 4：`python3 -m py_compile launch/gazebo_openarm.launch.py` 通过
- [ ] Step 5：`install/.../config/gazebo/` 与 `install/.../worlds/` 都存在
- [ ] Step 6：`ros2 launch ... gui:=false rviz:=false` 能起来（CI / 无显示器环境）
- [ ] Step 6：有显示器时 Gazebo 窗口里能看到完整双臂 + 双夹爪（不是白方块 = mesh 路径 OK）
- [ ] Step 7：`ros2 topic hz /clock` 在 850~1000 之间（RTF < 1 时偏低属正常）
- [ ] Step 7：`ListHardwareComponents` 两个组件都是 `GazeboSimSystem` + active
- [ ] Step 7：`ListControllers` 5 个控制器全 active
- [ ] Step 7：`FollowJointTrajectory` 返回 `status: 4` / `error_code: 0`，Gazebo + RViz 同步运动
- [ ] Step 7：夹爪发 `0.03` 后 `finger_joint1` 与 `finger_joint2` 同时 = `0.0300`

