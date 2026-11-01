# 给下一个 AI 的提示词模板

> 用法：新开一个对话，把下面「主模板」整段复制进去，把 `<...>` 换成你的任务即可。
> 目标是**让 AI 只读 1 个文件就开工**，不要让它全仓扫描。

## 一、主模板（新会话冷启动）

```text
项目：/home/leo/space-arm_ws （ROS 2 Jazzy + Gazebo Harmonic，OpenArm v1.0 双臂）
实物版本是 v1.0（v10），v2.0 代码并存但不使用。项目里没有 MoveIt，不要假设有、也不要引入。

第一步只做一件事：读 /home/leo/space-arm_ws/docs/AI_HANDOFF.md（约 350 行）。
不要执行 find / grep 全仓扫描，不要读其他文件，直到我给出具体任务。

硬性约束（违反会直接坏掉已验证的东西）：
1. 所有 xacro 的 sim 参数默认值必须是 none，不能改成 gz。
2. finger_joint2 只能有 state interface，不能加 command interface。
3. 控制器只能使用 position command interface。
4. v1.0 的 ros2_control 必须显式传 ros2_control:=true 才生成。
5. 发臂轨迹必须覆盖该臂全部 7 个关节。
6. 不要删除或重写 openarm_v2.0 目录。

我的任务：<在这里填>
验收标准：<在这里填可执行的检查命令或现象>

如果你认为需要改架构、加依赖或动 CMakeLists/package.xml，先说方案等我确认，不要直接动手。
改完之后：列出你改了哪些文件（绝对路径）、为什么改、以及你实际跑过的验证命令与输出（不要写"应该可以"）。
```

## 二、继续旧会话

```text
继续上次的 OpenArm 仿真任务。
先读 docs/AI_HANDOFF.md 的 §9（已知问题）和 §10（工作区状态），
再用 git status 确认工作区，然后等我给任务。
不要重复验证已经验证过的内容（§8 的基线）。
```

## 三、每次给新目标时（任务卡，填到主模板的「我的任务」位置）

```text
目标：<一句话>
涉及：<改哪个文件 / 新增哪个文件，不确定就写"你判断">
不做：<明确排除的范围，例如"不要动 launch"、"不要改控制器 yaml"、"不要装新依赖">
验收：<例如 "ros2 topic echo /joint_states 里 finger_joint1=0.03" 或 "colcon build 通过" 或 "§8 冒烟脚本输出 18/0.300/0.030">
```

## 四、为什么这么写（改模板时保留这几条）

| 设计 | 原因 |
|---|---|
| 只让读 `AI_HANDOFF.md` | 918 行的 guide 整篇读要几万 token；handoff 一页就够开工 |
| 把 6 条硬约束写进 prompt | 这几条都是踩过的坑，AI 不知道就一定会违反（尤其 `sim` 默认值、mimic 命令接口、7 关节全覆盖） |
| 先要方案再动手 | 防止 AI 一上来就重写 xacro |
| 要求贴验证输出 | 强制它真跑，而不是"应该可以" |
| 任务卡里写「不做」 | 比"做什么"更能收敛改动面 |
| 明确"没有 MoveIt" | 否则 AI 会热心推荐 MoveIt / 写 SRDF，浪费一整轮 |

## 五、可选补充条款（按需追加到主模板）

```text
- 不要新增 apt 依赖，只能使用当前已安装的包（ros2 pkg list 可查）。
- 不要修改 docs/ 以外的文件（只做文档任务时）。
- 修改 xacro 后必须重新 colcon build（--symlink-install 也要，因为 $(find ...) 指向 install）。
- 每次改完必须跑 §8 的冒烟脚本，并原样贴出输出。
- 如果发现文档与代码不一致，以代码为准，并在回答里指出文档哪一行过时了。
```
