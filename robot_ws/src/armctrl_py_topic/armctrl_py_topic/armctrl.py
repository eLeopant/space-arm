# 1.导包
import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
# arm ctrl topic msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# gripper ctrl topic msg
from std_msgs.msg import Float64MultiArray

from builtin_interfaces.msg import Duration


# 左右手臂 joints
LEFT_ARM_JOINTS = [f"openarm_left_joint{i}" for i in range(1, 8)]
RIGHT_ARM_JOINTS = [f"openarm_right_joint{i}" for i in range(1, 8)]

# 自定义节点类
class CtrlTalker(Node) :
    def __init__(self):
        super().__init__("ctrl_talker")
        self.get_logger().info("talker create !")
        # 创建发布者
        """
        消息类型， 话题名称， Qos队列长度, 返回发布方对象
        """
        self.left_arm_publisher = self.create_publisher(JointTrajectory, "/left_arm_controller/joint_trajectory", 10)
        self.right_arm_publisher = self.create_publisher(JointTrajectory, "/right_arm_controller/joint_trajectory", 10)
        self.left_gripper_publisher = self.create_publisher(Float64MultiArray, "/left_gripper_controller/commands", 10)
        self.right_gripper_publisher = self.create_publisher(Float64MultiArray, "/right_gripper_controller/commands", 10)
        self.timer = self.create_timer(1.0, self.time_callback)
    
        # 定时器回调函数
    def time_callback(self) :

        Rarm_msg = self.build_arm_msg(RIGHT_ARM_JOINTS,
                                    [1.0, 0.5, -0.3, 1.0, 0.5, 0.0, 0.0],)
        Rgripper_msg = Float64MultiArray()
        Rgripper_msg.data = [0.5]
        self.right_arm_publisher.publish(Rarm_msg)
        self.right_gripper_publisher.publish(Rgripper_msg)
        self.get_logger().info("published right arm & gripper")

    def build_arm_msg(self, joint_names, positions, time_lim = 2):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.joint_names = joint_names
        # JointTrajectoryPoint 嵌套在 JointTrajectory 用于表达关节位置
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec = time_lim, nanosec = 0)

        msg.points = [point] # 连续多点移动数组

        return msg





def main():
    # 初始化客户端
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])  #使用仿真时间
    ctrl_talker = CtrlTalker()
    rclpy.spin(ctrl_talker)
    # 资源释放
    rclpy.shutdown()

if __name__ == "__main__" :
    main()