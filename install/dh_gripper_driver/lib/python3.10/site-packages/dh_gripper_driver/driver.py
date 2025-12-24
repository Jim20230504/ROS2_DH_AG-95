import rclpy
from rclpy.node import Node
import time
import struct
import serial
import threading
import os

from dh_gripper_msgs.msg import GripperCtrl, GripperState
from sensor_msgs.msg import JointState

class DHGripperDriver(Node):
    def __init__(self):
        super().__init__('dh_gripper_driver')

        # === 参数声明 ===
        self.declare_parameter('gripper_model', 'AG95_MB')
        self.declare_parameter('connect_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('gripper_id', 1)
        # 即使这里声明了参数，下面我们也强制覆盖它
        self.declare_parameter('use_sim', False) 

        self.port = self.get_parameter('connect_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.gripper_id = self.get_parameter('gripper_id').value
        
        # ==========================================
        # 【修改点 1】强制使用真机模式，忽略 Launch 文件参数
        # ==========================================
        self.use_sim = False 
        # 原来的写法是: self.use_sim = self.get_parameter('use_sim').value

        self.get_logger().info(f"配置: Port={self.port}, Baud={self.baud}, ID={self.gripper_id}, Sim={self.use_sim} (强制真机模式)")

        # === 内部状态 ===
        self.current_position = 0.0 
        self.target_position = 0.0
        self.target_force = 50.0
        self.target_speed = 50.0
        self.is_initialized = False
        
        self.ser = None
        self.lock = threading.Lock()

        # === 初始化硬件连接 ===
        if not self.use_sim:
            try:
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.1
                )
                self.get_logger().info(f"成功打开串口: {self.port}")
                
                # 尝试读取一次状态来确认连接
                self.update_real_state() 
                
            except serial.SerialException as e:
                # ==========================================
                # 【修改点 2】更详细的报错，不再自动切回仿真，方便排查问题
                # ==========================================
                self.get_logger().error(f"严重错误: 无法打开串口 {self.port}")
                self.get_logger().error(f"错误详情: {e}")
                if "Permission denied" in str(e):
                    self.get_logger().error("权限不足！请运行: sudo chmod 777 " + self.port)
                # 保持 False，让它报错，这样你知道连不上
                self.use_sim = False 
        
        # 仿真模式逻辑（这里只有在上方被手动改回True时才会运行）
        if self.use_sim:
            self.is_initialized = True
            self.get_logger().info("处于仿真模式")

        # === ROS 通信 ===
        self.sub_ctrl = self.create_subscription(GripperCtrl, '/gripper/ctrl', self.ctrl_callback, 10)
        self.pub_state = self.create_publisher(GripperState, '/gripper/states', 10)
        self.pub_joint_state = self.create_publisher(JointState, '/gripper/joint_states', 10)

        # 50Hz 控制循环
        self.timer = self.create_timer(0.02, self.timer_callback)

    def ctrl_callback(self, msg):
        """处理控制指令"""
        if msg.initialize:
            if self.use_sim:
                self.is_initialized = True
            else:
                self.send_command('INIT')
                self.get_logger().info("收到初始化指令，正在发送给夹爪...")
        else:
            self.target_position = max(0.0, min(1000.0, msg.position))
            self.target_force = max(20.0, min(100.0, msg.force))
            self.target_speed = max(1.0, min(100.0, msg.speed))
            
            if not self.use_sim:
                self.send_command('WRITE')

    # ================= Modbus RTU 核心逻辑 =================
    def calculate_crc(self, data):
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for i in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return struct.pack('<H', crc)

    def send_command(self, cmd_type):
        if self.ser is None or not self.ser.is_open:
            return

        with self.lock:
            try:
                if cmd_type == 'INIT':
                    # 大寰夹爪初始化指令
                    cmd = struct.pack('>BBHH', self.gripper_id, 0x06, 0x0100, 0x0001)
                elif cmd_type == 'WRITE':
                    # 写入力、速度、位置
                    self._write_single_register(0x0101, int(self.target_force))
                    self._write_single_register(0x0104, int(self.target_speed))
                    self._write_single_register(0x0103, int(self.target_position))
                    return 
                else:
                    return

                cmd += self.calculate_crc(cmd)
                self.ser.write(cmd)
                # 适当增加延时，防止指令太快堵塞
                time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f"串口发送错误: {e}")

    def _write_single_register(self, addr, val):
        try:
            cmd = struct.pack('>BBHH', self.gripper_id, 0x06, addr, val)
            cmd += self.calculate_crc(cmd)
            self.ser.write(cmd)
            time.sleep(0.005) # 给一点硬件响应时间
        except Exception as e:
             self.get_logger().error(f"寄存器写入错误: {e}")

    def update_real_state(self):
        if self.ser is None or not self.ser.is_open:
            return

        with self.lock:
            try:
                # 读取初始化状态(0x0200) 和 实际位置(0x0202)
                # 注意：你原来的代码只读了一个，这里尽量根据手册确认
                cmd = struct.pack('>BBHH', self.gripper_id, 0x03, 0x0200, 0x0003)
                cmd += self.calculate_crc(cmd)
                self.ser.write(cmd)
                
                # 增加等待时间，确保数据回传完整
                # 如果读不到数据，可以尝试把这个 sleep 稍微加大到 0.01
                # time.sleep(0.005) 
                
                resp = self.ser.read(5 + 2*3) 
                if len(resp) >= 11:
                    init_state = (resp[3] << 8) | resp[4]
                    # 检查是否真的读到了有效数据
                    curr_pos = (resp[7] << 8) | resp[8]
                    
                    self.is_initialized = (init_state == 1)
                    self.current_position = float(curr_pos)
            except Exception as e:
                pass

    def update_simulation_physics(self):
        """仿真运动逻辑"""
        if not self.is_initialized: return
        step = self.target_speed * 0.5
        diff = self.target_position - self.current_position
        if abs(diff) < step:
            self.current_position = self.target_position
        else:
            if diff > 0: self.current_position += step
            else: self.current_position -= step

    def timer_callback(self):
        # 1. 更新状态
        if self.use_sim:
            self.update_simulation_physics()
        else:
            self.update_real_state()

        # 2. 发布 ROS 状态
        state_msg = GripperState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.is_initialized = self.is_initialized
        state_msg.position = float(self.current_position)
        state_msg.target_position = float(self.target_position)
        state_msg.target_force = float(self.target_force)
        self.pub_state.publish(state_msg)

        # 3. 发布关节状态 (Rviz)
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = ['gripper_finger1_joint']
        
        # 映射逻辑
        sim_joint_pos = (1000.0 - self.current_position) / 1000.0 * 0.65
        js_msg.position = [sim_joint_pos]
        
        self.pub_joint_state.publish(js_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DHGripperDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser: node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()