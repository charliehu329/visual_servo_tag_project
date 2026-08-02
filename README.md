# velocity_servo_tag

面向 **Franka Research 3（FR3）** 的 ROS 2 AprilTag 视觉伺服项目。

当前验证环境：

* Ubuntu 24.04
* ROS 2 Jazzy
* Franka FR3
* franka_ros2 v3.0.0
* libfranka 0.17.0

---

## 1. 概述

本项目将以下模块整合在同一个 ROS 2 包中：

* USB 相机读取
* AprilTag 检测
* Simulink 视觉控制
* 相机速度到关节速度映射
* 关节速度与加速度限制
* 通信超时保护
* Franka 硬件和速度控制器启动
* 七维关节速度安全转发

当前主要链路：

```text
USB Camera
    → AprilTag Detector
    → Simulink
    → Velocity Mapper
    → Velocity Command Node
    → Franka Joint Velocity Controller
    → FR3
```

仓库同时保留双目、EKF 和主动变焦 Simulink 模型。双目闭环和变焦驱动仍需继续验证。

本项目已经包含 Franka 底层启动和速度转发，不再依赖单独的 `franka_velocity_ctrl` 包。

---

## 2. 项目框架

```text
visual_servo_tag/
├── velocity_servo_tag/
│   ├── velocity_command_node.py
│   ├── velocity_mapper_node.py
│   ├── robot_kinematics.py
│   ├── safety.py
│   └── vision/
│       ├── apriltag_detector.py
│       └── camera.py
│
├── config/
│   ├── velocity_servo_tag.yaml
│   ├── controllers.yaml
│   └── urdf/
│       └── fr3.urdf
│
├── launch/
│   ├── velocity_servo_tag.launch.py
│   ├── fr3_hardware.launch.py
│   └── full_system.launch.py
│
├── simulinkv2/
├── package.xml
├── setup.py
└── README.md
```

Launch 文件职责：

| Launch                         | 启动内容                                |
| ------------------------------ | ----------------------------------- |
| `velocity_servo_tag.launch.py` | AprilTag 检测器、Velocity Mapper        |
| `fr3_hardware.launch.py`       | Franka 硬件、控制器、Velocity Command Node |
| `full_system.launch.py`        | 组合前两个 Launch                        |

> Launch 文件不会自动启动 MATLAB/Simulink。Simulink 模型需要单独运行。

---

## 3. 如何部署

### 3.1 下载仓库

```bash
cd ~/franka_ros2_ws/src

git clone -b dev \
  https://github.com/charliehu329/visual_servo_tag.git
```

已经下载过：

```bash
cd ~/franka_ros2_ws/src/visual_servo_tag

git checkout dev
git pull
```

### 3.2 安装依赖

```bash
cd ~/franka_ros2_ws

source /opt/ros/jazzy/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y
```

安装 AprilTag 检测库：

```bash
python3 -m pip install \
  --user \
  --break-system-packages \
  pupil-apriltags
```

### 3.3 编译

```bash
cd ~/franka_ros2_ws

source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select velocity_servo_tag \
  --symlink-install

source install/setup.bash
```

检查程序：

```bash
ros2 pkg executables velocity_servo_tag
```

应包含：

```text
velocity_servo_tag apriltag_detector
velocity_servo_tag velocity_mapper_node
velocity_servo_tag velocity_command_node
```

---

## 4. 如何启动

### 4.1 每个新终端先执行

```bash
cd ~/franka_ros2_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

查看 Launch 参数：

```bash
ros2 launch velocity_servo_tag \
  full_system.launch.py \
  --show-args
```

### 4.2 只启动 AprilTag 检测器

```bash
ros2 launch velocity_servo_tag \
  velocity_servo_tag.launch.py \
  start_detector:=true \
  start_mapper:=false
```

临时指定 `/dev/video2`、1080p、60 Hz：

```bash
ros2 run velocity_servo_tag apriltag_detector \
  --ros-args \
  -p camera_index:=2 \
  -p camera_width:=1920 \
  -p camera_height:=1080 \
  -p camera_fps:=60.0 \
  -p detector_threads:=4 \
  -p quad_decimate:=2.0
```

检查输出：

```bash
ros2 topic echo \
  /apriltag_detector/target_position

ros2 topic hz \
  /apriltag_detector/target_position
```

### 4.3 启动检测器和 Mapper，保持 dry-run

不连接真机，不发送非零关节速度；异常状态下仍允许发布安全零目标：

```bash
ros2 launch velocity_servo_tag \
  velocity_servo_tag.launch.py \
  start_detector:=true \
  start_mapper:=true \
  dry_run:=true
```

只测试 Simulink 到 Mapper 的后半段：

```bash
ros2 launch velocity_servo_tag \
  velocity_servo_tag.launch.py \
  start_detector:=false \
  start_mapper:=true \
  dry_run:=true
```

### 4.4 只启动 Franka 硬件，持续发送零速度

```bash
ros2 launch velocity_servo_tag \
  fr3_hardware.launch.py \
  robot_ip:=172.16.0.2 \
  command_mode:=zero \
  max_velocity_scale:=0.10 \
  use_rviz:=false
```

检查控制器：

```bash
ros2 control list_controllers
```

重点确认：

```text
joint_state_broadcaster
franka_robot_state_broadcaster
joint_velocity_example_controller
```

### 4.5 完整系统安全联调

连接真机，但 Mapper 不发送关节速度，底层也只发送零速度：

```bash
ros2 launch velocity_servo_tag \
  full_system.launch.py \
  robot_ip:=172.16.0.2 \
  start_hardware:=true \
  start_detector:=true \
  dry_run:=true \
  command_mode:=zero \
  max_velocity_scale:=0.10 \
  use_rviz:=false
```

此阶段检查：

* AprilTag 是否稳定
* `/franka/joint_states` 是否正常
* Simulink 是否持续输出
* Mapper 输出方向是否正确
* 手眼矩阵是否正确
* 超时后是否回到零速度

### 4.6 完整系统实机控制

只有完成 dry-run 和零速度测试后，才允许执行：

```bash
ros2 launch velocity_servo_tag \
  full_system.launch.py \
  robot_ip:=172.16.0.2 \
  start_hardware:=true \
  start_detector:=true \
  dry_run:=false \
  command_mode:=topic \
  max_velocity_scale:=0.10 \
  use_rviz:=false
```

实机运动必须同时满足：

```text
dry_run:=false
command_mode:=topic
```

### 4.7 启动 Simulink

Simulink 需要在 MATLAB 中单独运行。

单目链路主要接口：

```text
订阅：
/apriltag_detector/target_position
/franka/joint_states

发布：
/simulink/camera_velocity
```

检查 Simulink 输出：

```bash
ros2 topic echo \
  /simulink/camera_velocity

ros2 topic hz \
  /simulink/camera_velocity
```

### 4.8 重要参数

#### Launch 参数

| 参数                   |          默认值 | 作用                |
| -------------------- | -----------: | ----------------- |
| `robot_ip`           | `172.16.0.2` | FR3 IP            |
| `start_hardware`     |      `false` | 是否启动真实硬件          |
| `start_detector`     |       `true` | 是否启动 AprilTag 检测器 |
| `start_mapper`       |       `true` | 是否启动 Mapper       |
| `dry_run`            |       `true` | 是否禁止 Mapper 发布非零速度 |
| `command_mode`       |       `zero` | `zero` 或 `topic`  |
| `max_velocity_scale` |       `0.10` | FR3 官方速度上限比例      |
| `use_rviz`           |      `false` | 是否启动 RViz         |
| `params_file`        |      默认 YAML | 参数文件路径            |

#### 相机和 AprilTag

| 参数                             | 作用       |
| ------------------------------ | -------- |
| `camera_index`                 | 相机编号     |
| `camera_width`、`camera_height` | 图像分辨率    |
| `camera_fps`                   | 请求帧率     |
| `target_tag_id`                | 目标标签 ID  |
| `detector_threads`             | 检测线程数    |
| `quad_decimate`                | 检测降采样倍数  |
| `uv_filter_alpha`              | 中心坐标滤波系数 |
| `show_window`                  | 是否显示检测窗口 |

`quad_decimate`：

* `1.0`：精度高，计算慢
* `2.0`：适合 1080p 实时检测
* 数值越大，远距离小标签越容易丢失

#### Mapper

| 参数                                    | 作用                              |
| --------------------------------------- | --------------------------------- |
| `T_end_effector_camera`                 | 相机到末端的手眼变换              |
| `end_effector_frame`                    | URDF 末端坐标系                   |
| `joint_state_timeout_sec`               | 关节状态超时                      |
| `visual_velocity_timeout_sec`           | 相机速度命令超时                  |
| `mapper_watchdog_rate_hz`               | 独立输入看门狗检查频率            |
| `camera_velocity_zero_epsilon`          | 关闭主任务和零空间的零命令阈值    |
| `nullspace_activation_start_ratio`      | 零空间回中开始平滑启用的位置比例  |
| `nullspace_activation_full_ratio`       | 零空间回中完全启用的位置比例      |
| `nullspace_centering_gain`               | 零空间关节回中增益                |
| `nullspace_task_speed_fade_start`        | 主任务增强时零空间开始衰减的速度  |
| `nullspace_task_speed_disable`           | 完全关闭零空间的主任务速度        |
| `nullspace_sigma_disable`                | 关闭零空间的最小奇异值阈值        |
| `nullspace_sigma_full`                   | 完全开放零空间的最小奇异值阈值    |
| `nullspace_max_joint_velocity`           | 零空间项独立关节速度上限          |
| `nullspace_max_joint_acceleration`       | 零空间项独立关节加速度上限        |

Mapper 在尚未收到合法输入、任一输入超时、输入非法、运动学求解失败或
Simulink 发布六维零命令时，都会向下游发布七维零速度目标。下游命令节点
继续负责整组关节速度的最终限速和加速度限制。

#### 底层命令节点

| 参数                            | 作用               |
| ----------------------------- | ---------------- |
| `mode`                        | `zero` 或 `topic` |
| `max_velocity_scale`          | FR3 速度上限比例       |
| `max_joint_accelerations`     | 二次加速度限制          |
| `target_velocity_timeout_sec` | Mapper 命令超时      |

---

## 5. 数据流向

```text
USB Camera
    │
    ▼
apriltag_detector
    │
    │ /apriltag_detector/target_position
    │ [valid, u, v]
    ▼
Simulink Visual Servo
    │
    │ /simulink/camera_velocity
    │ [vx, vy, vz, wx, wy, wz]
    ▼
velocity_mapper_node
    ▲
    │ /franka/joint_states
    │
    │ /velocity_mapper_node/target_joints_velocities
    │ [dq1, dq2, dq3, dq4, dq5, dq6, dq7]
    ▼
velocity_command_node
    │
    │ /joint_velocity_example_controller/commands
    ▼
joint_velocity_example_controller
    │
    ▼
Franka FR3
```

主要 Topic：

| Topic                                            | 类型                  | 数据                    |
| ------------------------------------------------ | ------------------- | --------------------- |
| `/apriltag_detector/target_position`             | `Float64MultiArray` | `[valid,u,v]`         |
| `/simulink/camera_velocity`                      | `Float64MultiArray` | `[vx,vy,vz,wx,wy,wz]` |
| `/franka/joint_states`                           | `JointState`        | FR3 关节状态              |
| `/velocity_mapper_node/target_joints_velocities` | `Float64MultiArray` | 七维关节速度                |
| `/joint_velocity_example_controller/commands`    | `Float64MultiArray` | 七维底层速度命令              |

单位：

* 相机线速度：`m/s`
* 相机角速度：`rad/s`
* 关节速度：`rad/s`

---

## 6. ROS 节点

| 节点                                  | 作用               | 输入                                                 | 输出                                               |
| ----------------------------------- | ---------------- | -------------------------------------------------- | ------------------------------------------------ |
| `apriltag_detector`                 | 读取相机并检测 AprilTag | USB Camera                                         | `/apriltag_detector/target_position`             |
| `velocity_mapper_node`              | 相机速度转换为关节速度      | `/simulink/camera_velocity`、`/franka/joint_states` | `/velocity_mapper_node/target_joints_velocities` |
| `velocity_command_node`             | 限速、平滑并转发速度       | Mapper 七维速度                                        | `/joint_velocity_example_controller/commands`    |
| `joint_state_broadcaster`           | 发布关节状态           | FR3 Hardware                                       | `/franka/joint_states`                           |
| `franka_robot_state_broadcaster`    | 发布 Franka 状态     | FR3 Hardware                                       | Franka 状态 Topic                                  |
| `joint_velocity_example_controller` | 执行关节速度命令         | 七维速度命令                                             | FR3 Hardware                                     |

常用检查：

```bash
ros2 node list
ros2 topic list
ros2 control list_controllers
```

---

## 7. 安全规则

1. 首次运行必须使用：

   ```text
   dry_run:=true
   command_mode:=zero
   ```

2. 当前 YAML 中的 `T_end_effector_camera` 是占位值。手眼标定完成前，禁止执行非零实机控制。

3. 必须先验证相机坐标系、末端坐标系以及六维速度正负方向。

4. `valid=0`、目标丢失、图像异常或 Simulink 输入无效时，必须输出零速度。

5. 不要随意增大：

   ```text
   max_joint_velocities
   max_joint_accelerations
   max_velocity_scale
   visual_velocity_timeout_sec
   target_velocity_timeout_sec
   ```

6. 初次实机建议保持：

   ```text
   max_joint_velocities: 0.05 rad/s
   max_joint_accelerations: 0.20 rad/s²
   max_velocity_scale: 0.10
   ```

7. 同一时间只能有一个节点向以下 Topic 发布：

   ```text
   /joint_velocity_example_controller/commands
   ```

8. 不要同时启动两个占用同一 USB 相机的节点。

9. Topic 超时、关节状态超时、NaN、Inf 或消息长度错误时，必须进入零速度或平滑减速状态。

10. 真机运行时必须保证：

    * 急停可随时触发
    * 操作者离开机器人工作空间
    * 周围没有人员和障碍物
    * 先低速、短时间、单方向测试
    * 方向异常立即停止

11. 修改代码或配置后重新编译：

    ```bash
    cd ~/franka_ros2_ws

    source /opt/ros/jazzy/setup.bash

    colcon build \
      --packages-select velocity_servo_tag \
      --symlink-install

    source install/setup.bash
    ```

12. 真机控制前至少确认：

    ```bash
    ros2 control list_controllers
    ros2 topic hz /franka/joint_states
    ros2 topic hz /simulink/camera_velocity
    ros2 topic echo /velocity_mapper_node/target_joints_velocities
    ```

---

## License

Apache-2.0
