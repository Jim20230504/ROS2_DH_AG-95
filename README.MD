运行：ros2 launch dh_gripper_description dh_gripper_bringup.launch.py

夹爪运动：ros2 topic pub --once /gripper/ctrl dh_gripper_msgs/msg/GripperCtrl "{initialize: false, position: 0, force: 50.0, speed: 50.0}"

连接硬件：将夹爪的 USB 转 485 模块插入电脑。

确认串口号： 执行 ls /dev/ttyUSB*，确认设备名（通常是 /dev/ttyUSB0）。


赋予权限（重要，否则会报错 Permission denied）：

Bash

sudo chmod 777 /dev/ttyUSB0
修改启动文件（可选，也可以在命令行覆盖参数）： 你可以直接在命令行运行 launch 时指定参数来连接硬件。

运行指令（连接硬件模式）：

Bash

ros2 launch dh_gripper_description dh_gripper_bringup.launch.py use_sim:=False connect_port:=/dev/ttyUSB0
开始控制： 打开新终端：

初始化（硬件上电后第一次必须做）：

Bash

ros2 topic pub --once /gripper/ctrl dh_gripper_msgs/msg/GripperCtrl "{initialize: true}"
现象：夹爪应该会动一下（进行自检）。

控制闭合：

Bash

ros2 topic pub --once /gripper/ctrl dh_gripper_msgs/msg/GripperCtrl "{initialize: false, position: 1000.0, force: 50.0, speed: 50.0}"