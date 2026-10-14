# velocity_servo_tag · q_dot

`single_version` 分支当前使用的 FR3 单目 AprilTag 视觉伺服版本。Python 负责相机检测和硬件启动；Simulink 完成控制、运动学求解及关节命令保护，直接发送七维关节速度。

```text
USB 相机 → AprilTag 检测 → Simulink q_dot → 关节速度控制器 → FR3
                               ↑
                        /franka/joint_states
```

## 1. 环境与文件

Ubuntu 24.04、ROS 2 Jazzy、已配置的 franka_ros2；MATLAB、Simulink、ROS Toolbox、Robotics System Toolbox。

| 文件／目录 | 用途 |
| --- | --- |
| `velocity_servo_tag/vision/` | `camera.py` 读取相机，`apriltag_detector.py` 发布检测结果 |
| `launch/detector_handware.launch.py` | 总入口，组合检测器与硬件启动 |
| `launch/apriltag_detector.launch.py`、`launch/hardware.launch.py` | 单独启动检测器／FR3 硬件与控制器 |
| `config/velocity_servo_tag.yaml` | 相机与检测参数，使用 `apriltag_detector` 配置段 |
| `config/controllers.yaml`、`config/urdf/fr3.urdf` | 底层控制器配置／Simulink 运动学模型 |
| `simulink_q_dot/config/init_single_camera_q_dot_tracking.m` | 控制、标定和安全参数初始化 |
| `simulink_q_dot/single_vision_q_dot/single_camera_q_dot.slx` | 当前控制模型 |
| `simulink_q_dot/test/` | 测试日志保存与绘图工具 |

## 2. 安装与编译

工作空间需已有 franka_ros2 及其依赖。检测器使用系统 `python3`，依赖 NumPy、OpenCV、`rclpy`、`std_msgs`、`pupil-apriltags`。

```bash
sudo apt install python3-numpy python3-opencv
python3 -m pip install --user --break-system-packages pupil-apriltags

cd ~/franka_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select velocity_servo_tag --symlink-install
source install/setup.bash
```

## 3. 启动

每个新终端先加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
```

仅检测，不连接机器人（默认）：

```bash
ros2 launch velocity_servo_tag detector_handware.launch.py
```

检测器＋实机硬件（先完成第 7 节检查）：

```bash
ros2 launch velocity_servo_tag detector_handware.launch.py \
  start_hardware:=true robot_ip:=172.16.0.2 use_rviz:=false
```

常用参数：`start_hardware=false`、`start_detector=true`、`load_gripper=false`、`use_rviz=false`、`robot_ip=172.16.0.2`；`params_file` 可指定检测器 YAML。查看参数：

```bash
ros2 launch velocity_servo_tag detector_handware.launch.py --show-args
```

**Simulink 不由 launch 启动。** MATLAB 当前文件夹切到仓库根目录后执行：

```matlab
run('simulink_q_dot/config/init_single_camera_q_dot_tracking.m')
open_system('simulink_q_dot/single_vision_q_dot/single_camera_q_dot.slx')
```

确认 ROS 2 网络与 `ROS_DOMAIN_ID` 一致后，在模型中点击 Run。模型 `InitFcn` 会再次运行初始化脚本，调参应修改脚本，避免仅修改工作区后被覆盖。

## 4. Topic 接口

| Topic | 消息类型 | 方向与数据 |
| --- | --- | --- |
| `/apriltag_detector/target_position` | `std_msgs/msg/Float64MultiArray` | 检测器 → Simulink：`[valid, u, v, depth_m]` |
| `/franka/joint_states` | `sensor_msgs/msg/JointState` | FR3 → Simulink：`name`、`position`、`velocity` |
| `/joint_velocity_example_controller/commands` | `std_msgs/msg/Float64MultiArray` | Simulink → 控制器：`[dq1, dq2, dq3, dq4, dq5, dq6, dq7]` |

- `valid=1` 表示有效；检测失败发布 `[0, 0, 0, 0]`。`u/v` 为像素，`depth_m` 为相机光轴 Z 方向深度（m）。
- 关节命令按 `fr3_joint1`～`fr3_joint7` 排列，单位 rad/s；反馈位置单位 rad，模型按关节名称匹配反馈顺序。
- 模型固定步长 `Ts=0.01 s`（100 Hz）；检测器请求 60 Hz，实际频率以测量为准。
- 模型 QoS：订阅为 Best effort，发布为 Reliable；均 Keep last、深度 1、Volatile。

## 5. 调参位置

- **相机／检测**：修改 YAML 的 `apriltag_detector` 段。当前 `/dev/video2`、1920×1080、60 Hz，`tag36h11`、ID 0、`tag_size_m=0.10`。
- **内参／手眼**：初始化脚本的 `fx/fy/cx/cy`、`T_link8_camera`；内参须与检测器一致，手眼矩阵方向为相机坐标系 → `fr3_link8`。
- **控制／KF**：`Kpx/Kpy`、`k_ff`、`task_velocity_filter_tau`、`Q_ekf/R_ekf/P0`；当前比例控制开启，KF 速度前馈关闭。
- **保护／使能**：`controller_enable`、输入超时、奇异值阈值、深度安全区、关节位置／速度／加速度限制。
- **底层滤波**：`config/controllers.yaml` 的 `filter_coefficient`。

## 6. 检查与测试结果

```bash
ros2 control list_controllers
ros2 topic hz /apriltag_detector/target_position
ros2 topic hz /franka/joint_states
ros2 topic echo /joint_velocity_example_controller/commands
ros2 topic info /joint_velocity_example_controller/commands --verbose
```

实机运行需确认 `joint_state_broadcaster`、`franka_robot_state_broadcaster`、`joint_velocity_example_controller` 为 active。

仿真结束并保留日志后，从仓库根目录在 MATLAB 执行：

```matlab
cd('simulink_q_dot/test')
testName = 'q_dot_test';
testType = 'closed_loop';
save_single_camera_test_results
```

输出到 `simulink_q_dot/test/test_results/`：日志、参数、指标、CSV、图像及 `summary.txt`。

## 7. 实机注意

- 初始化脚本默认 `controller_enable=true`。首次联调先在脚本设为 `false`，核对零命令、关节顺序、标定和运动方向后再使能；从低速、短时测试开始。
- 不同时运行多个相机采集节点或多个关节命令发布者。
- 验证输入异常、目标丢失和超时后的实际响应；停止 Simulink 不等于机器人急停。
- 保持工作空间无人、无障碍，急停可随时触发。

许可证：Apache-2.0。
