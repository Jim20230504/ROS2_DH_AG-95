import rclpy
from rclpy.node import Node
import time
import math

from dh_gripper_msgs.msg import GripperCtrl, GripperState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class DHGripperDriver(Node):
    def __init__(self):
        super().__init__('dh_gripper_driver')

        # === 参数声明 ===
        self.declare_parameter('gripper_model', 'AG95_MB')
        self.declare_parameter('connect_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('gripper_id', 1)
        # 如果为 True，则不尝试连接串口，而是模拟运动，方便在 Rviz 中测试
        self.declare_parameter('use_sim', True) 

        self.model = self.get_parameter('gripper_model').value
        self.port = self.get_parameter('connect_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.use_sim = self.get_parameter('use_sim').value

        self.get_logger().info(f"初始化大寰夹爪驱动: Model={self.model}, Port={self.port}, SimMode={self.use_sim}")

        # === 内部状态变量 ===
        # 0 = Fully Open, 1000 = Fully Closed (DH Protocol default)
        self.current_position = 0.0 
        self.target_position = 0.0
        self.target_force = 50.0
        self.target_speed = 50.0
        # self.is_initialized = False
        # self.grip_state = 0 # 0: Ready, 1: Moving, 2: Stopped
        # 如果是仿真模式，直接默认初始化完成，方便调试
        if self.use_sim:
            self.is_initialized = True
            self.get_logger().info("仿真模式：已自动初始化夹爪")
        else:
            self.is_initialized = False
            
        self.grip_state = 0 # 0: Ready, 1: Moving, 2: Stopped

        # === ROS 通信接口 ===
        # 订阅控制指令
        self.sub_ctrl = self.create_subscription(
            GripperCtrl, 
            '/gripper/ctrl', 
            self.ctrl_callback, 
            10
        )

        # 发布夹爪状态
        self.pub_state = self.create_publisher(
            GripperState, 
            '/gripper/states', 
            10
        )
        
        # 发布关节状态 (用于 Rviz/Gazebo 可视化)
        self.pub_joint_state = self.create_publisher(
            JointState, 
            '/gripper/joint_states', 
            10
        )

        # 定时器 50Hz (与原 C++ 代码一致)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def ctrl_callback(self, msg):
        """处理控制指令"""
        if msg.initialize:
            self.get_logger().info("收到初始化指令")
            self.is_initialized = True
            # 初始化时通常归零或全开
            self.target_position = 0.0 
        else:
            # 限制范围 0-1000
            pos = max(0.0, min(1000.0, msg.position))
            force = max(0.0, min(100.0, msg.force))
            speed = max(0.0, min(100.0, msg.speed))
            
            self.target_position = pos
            self.target_force = force
            self.target_speed = speed
            
            self.get_logger().info(f"设定目标: Pos={pos}, Force={force}, Speed={speed}")

    def update_simulation_physics(self):
        """
        简单的模拟物理引擎。
        根据设定的速度，将当前位置逐步移动到目标位置。
        """
        if not self.is_initialized:
            return

        step = self.target_speed * 0.5 # 模拟步长
        diff = self.target_position - self.current_position

        if abs(diff) < step:
            self.current_position = self.target_position
            self.grip_state = 2 # Stopped/Reached
        else:
            self.grip_state = 1 # Moving
            if diff > 0:
                self.current_position += step
            else:
                self.current_position -= step

    def get_joint_position_radians(self, dh_pos):
        """
        将大寰的 0-1000 位置数值转换为 URDF 的关节弧度/米。
        参考原 C++ 代码: msg.position[0] = (1000-tmp_pos)/1000.0 * 0.637;
        注意：AG95 等型号通常是平行夹爪，单位可能是米或弧度，具体取决于 URDF 定义。
        这里沿用原代码的转换逻辑。
        """
        # 假设 0 是全开，1000 是全闭。
        # 原代码：(1000 - pos) / 1000 * 0.637
        # 这意味着 pos=0 时，val=0.637 (开)；pos=1000 时，val=0 (闭)
        # 如果你在 Rviz 发现运动方向反了，可以修改这里。
        return (1000.0 - dh_pos) / 1000.0 * 0.637

    def timer_callback(self):
        # 1. 更新物理状态 (模拟或真实读取)
        if self.use_sim:
            self.update_simulation_physics()
        else:
            # TODO: 这里填入真实的串口通信代码 (使用 pyserial)
            # 可以在未来扩展，目前仅提供仿真模式
            pass

        # 2. 发布 GripperState
        state_msg = GripperState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.is_initialized = self.is_initialized
        state_msg.grip_state = self.grip_state
        state_msg.position = float(self.current_position)
        state_msg.target_position = float(self.target_position)
        state_msg.target_force = float(self.target_force)
        self.pub_state.publish(state_msg)

        # 3. 发布 JointState (给 Rviz/Gazebo 看)
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        # 注意：这里名字必须对应 URDF 中的关节名。
        # 通常 AG95 的主关节叫 "gripper_finger1_joint" 或类似。
        # 如果你的 URDF 左右指是 mimic 关系，只需要发布主关节。
        js_msg.name = ['gripper_finger1_joint'] 
        
        sim_joint_pos = self.get_joint_position_radians(self.current_position)
        js_msg.position = [sim_joint_pos]
        
        # 很多夹爪 URDF 定义了第二个指头，如果是 mimic 可以不发，
        # 但为了保险，有时需要同时发左右指 (取决于 URDF 是否配置了 mimic 插件)。
        # 暂时只发一个，原 C++ 代码也只发了一个。
        
        self.pub_joint_state.publish(js_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DHGripperDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()