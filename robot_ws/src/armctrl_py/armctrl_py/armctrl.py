# 1. 导包
import rclpy
from rclpy.node import Node
import math
from tf_transformations import quaternion_from_euler

from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

# MoveIt 相关的消息和服务
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

class CtrlNode(Node):
    def __init__(self):
        super().__init__("ctrl_node")
        self.get_logger().info("控制节点启动！")
        
        # 1. 创建发布者（发轨迹给 Gazebo 控制器）
        self.right_arm_publisher = self.create_publisher(JointTrajectory, "/right_arm_controller/joint_trajectory", 10)
        
        # 2. 订阅当前关节状态（为 MoveIt 提供起点）
        self.latest_joint_state = None
        self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)

        # 3. 创建 MoveIt 规划服务客户端
        self.plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.get_logger().info("等待 MoveIt 规划服务启动...")
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            pass
        self.get_logger().info("MoveIt 服务已连接！")
        
        # 订阅者监听posestamped话题，有关于位置姿态指令的消息就会回调来做ik并输出关节角度给底层
        self.target_sub = self.create_subscription(
            PoseStamped,"/target_pose", self.target_pose_callback, 10
        )

        self.isplanning = False

    def joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def target_pose_callback(self, msg:PoseStamped):
        # 防止上一次规划还没结束就触发新的
        if self.isplanning == True:
            self.get_logger().warn("已经在规划了...")
            return
        if self.latest_joint_state is None:
            self.get_logger().warn("没有看到起始点位置姿态，规划不了...")
            return
        
        self.isplanning = True
        self.get_logger().info("\n--- 开始请求 MoveIt 规划 ---")

        # ==========================================
        # 目标输入区 (这里输入你的 XYZ 和 Roll Pitch Yaw)
        # ==========================================
        target_x = 0.2
        target_y = -0.1
        target_z = 0.4
        
        # 注意：这里填的是弧度！
        roll = 0.0           
        pitch = math.pi / 2.0 # 90度 (1.57 rad)
        yaw = 0.0           

        # 将 RPY 转换为四元数
        q = quaternion_from_euler(roll, pitch, yaw)

        # ==========================================
        # 构造 MoveIt 规划请求
        # ==========================================
        req = GetMotionPlan.Request()
        mpr = req.motion_plan_request
        
        mpr.group_name = "openarm_right"
        mpr.num_planning_attempts = 10
        mpr.allowed_planning_time = 5.0
        # 速度加速度缩放 (注意：如果 yaml 中已经写了全局缩放，这里可以去掉，以免产生冲突)
        mpr.max_velocity_scaling_factor = 0.5
        mpr.max_acceleration_scaling_factor = 0.5
        
        # 【关键】告诉 MoveIt 起点就是当前获取到的所有关节状态
        mpr.start_state.joint_state = self.latest_joint_state

        # 1. 构造目标姿态 PoseStamped
        target_pose = msg
        if target_pose.header.frame_id == None:
            target_pose.header.frame_id == "openarm_body_link0"
        target_pose.header.stamp = self.get_clock().now().to_msg()


        # 2. 构造约束条件 (同时约束位置和姿态)
        constraints = Constraints()
        
        # 位置约束
        pos_con = PositionConstraint()
        pos_con.header = target_pose.header
        pos_con.link_name = "openarm_right_link7" # 你的末端执行器 Link
        # 容差设为 1 厘米 (0.01m)
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01])
        pos_con.constraint_region.primitives.append(box)
        pos_con.constraint_region.primitive_poses.append(target_pose.pose)
        pos_con.weight = 1.0
        constraints.position_constraints.append(pos_con)

        # 姿态约束
        ori_con = OrientationConstraint()
        ori_con.header = target_pose.header
        ori_con.link_name = "openarm_right_link7"
        ori_con.orientation = target_pose.pose.orientation
        # 姿态容差设为 0.05 弧度 (约 3 度)
        ori_con.absolute_x_axis_tolerance = 0.05
        ori_con.absolute_y_axis_tolerance = 0.05
        ori_con.absolute_z_axis_tolerance = 0.05
        ori_con.weight = 1.0
        constraints.orientation_constraints.append(ori_con)

        mpr.goal_constraints.append(constraints)

        # 3. 异步调用服务
        self.future = self.plan_client.call_async(req)
        self.future.add_done_callback(self.plan_response_callback)

    def plan_response_callback(self, future):
        self.isplanning = False
        
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"服务调用异常: {e}")
            return

        # 检查错误码 (1 表示 SUCCESS)
        if response.motion_plan_response.error_code.val != 1:
            self.get_logger().error(f"❌ 规划失败，错误码: {response.motion_plan_response.error_code.val}")
            self.get_logger().error("可能原因: 目标超出臂展、发生碰撞、或姿态无法实现。")
            return

        # 提取轨迹
        traj = response.motion_plan_response.trajectory.joint_trajectory
        if len(traj.points) == 0:
            self.get_logger().error("规划成功但没有轨迹点！")
            return

        self.get_logger().info(f"✅ 规划成功！共 {len(traj.points)} 个轨迹点。")

        # 【极其关键】把轨迹的时间戳更新为当前时间
        # 否则 Gazebo 控制器一看是“过去”的指令，会直接拒绝执行
        traj.header.stamp = self.get_clock().now().to_msg()
        
        # 一次性将整条平滑轨迹下发给 Gazebo
        self.right_arm_publisher.publish(traj)
        self.get_logger().info("🚀 已成功下发轨迹给 Gazebo 控制器！\n")

def main():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    ctrl_node = CtrlNode()
    try:
        rclpy.spin(ctrl_node)
    except KeyboardInterrupt:
        pass
    finally:
        ctrl_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()