import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. 获取包路径
    pkg_description = get_package_share_directory('dh_gripper_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'ag95_gripper.urdf.xacro')

    # 2. 解析 Xacro
    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toxml()
    
    # 3. 节点定义
    
    # Robot State Publisher: 读取 URDF 并发布 TF
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    # Joint State Publisher (GUI): 仅用于测试 URDF 模型本身时使用，
    # 但我们现在有真实的(或仿真的)驱动，所以我们使用下面的 dh_gripper_driver。
    # 如果你想纯粹测试模型不带驱动，可以取消注释这个，注释掉 driver。
    # node_joint_state_publisher_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui'
    # )

    # DH Gripper Driver: 我们的 Python 驱动
    # 注意：我们需要 remap 话题，因为 robot_state_publisher 默认听 /joint_states
    # 而我们的驱动发布在 /gripper/joint_states
    node_driver = Node(
        package='dh_gripper_driver',
        executable='dh_gripper_driver',
        name='dh_gripper_driver',
        output='screen',
        parameters=[{'use_sim': True}], # 开启仿真模式
        remappings=[
            ('/gripper/joint_states', '/joint_states') 
        ]
    )

    # Rviz2
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', os.path.join(pkg_description, 'rviz', 'view_model.rviz')] # 如果你有保存的配置
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_driver,
        node_rviz
    ])