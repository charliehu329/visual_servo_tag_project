# 项目目标

说明：记录整个项目的目标

构建一套工程化的 Franka FR3 双目主动变焦视觉伺服系统：

- 双目相机检测目标并输出图像特征；
- Simulink 完成特征处理、机器人运动学、视觉控制、状态估计和算法层安全保护；
- Simulink 直接输出 `7×1` 关节速度 `qDot`；
- Python ROS 2 节点负责最终速度/加速度限制、watchdog、消息检查和底层转发；
- 同一套 `stereo_ibvs_core.slx` 同时服务于离线仿真、ROS 2 联调和真机部署；
- 后续逐步加入双目逆深度、关节限位零空间任务、目标 EKF、主动变焦和高速跟踪。

当前目标环境：Ubuntu 24.04、ROS 2 Jazzy、MATLAB R2025b。

安全分层：Simulink 负责算法层保护和失效输出零速度，Python 负责最终硬限速、断流保护和底层发送。

# 文件架构

说明：把当前文件夹的架构记录，项目树记录在下面

当前主要文件：

```text
visual_servo_tag/
├── AGENTS.md
├── memory.md
├── README.md
├── config/
│   ├── controllers.yaml
│   ├── velocity_servo_tag.yaml
│   └── urdf/
│       └── fr3.urdf
├── launch/
│   ├── fr3_hardware.launch.py
│   ├── full_system.launch.py
│   ├── vision_double.launch.py
│   └── velocity_servo_tag.launch.py
├── simulink/
│   ├── build/
│   │   └── sim/build_stereo_ibvs_sim_stage1.m
│   ├── config/
│   │   └── stereo_ibvs_config.m
│   ├── core/
│   │   └── stereo_ibvs_core.slx
│   ├── ros2/
│   │   └── stereo_ibvs_ros2_stage1.slx
│   └── sim/
│       ├── stereo_ibvs_sim_stage1.slx
│       └── plot_stereo_ibvs_sim_stage1_results.m
├── velocity_servo_tag/
│   ├── robot_kinematics.py
│   ├── safety.py
│   ├── velocity_command_node.py
│   └── vision/
│       ├── apriltag_detector.py
│       ├── camera.py
│       ├── stereo_features.py
│       └── vision_double_node.py
├── test/
│   └── test_stereo_features.py
├── package.xml
├── setup.cfg
└── setup.py
```

架构约定：

- `simulink/core/`：不直接收发 ROS 2 消息的统一核心算法；
- `simulink/ros2/`：ROS 2 包装模型，通过 Model 块引用同一个 core；
- `simulink/build/`：可重复生成模型的 MATLAB 脚本；
- `simulink/sim/`：Stage 1 离线仿真模型和结果绘图脚本；
- `*.slxc`、`slprj/`、`*.autosave`、`*.asv` 和 ROS 总线转换文件是自动生成缓存，已经加入 `.gitignore`。

# 已完成的文件状态

说明：每次做完都要修改这里，把当前做了什么，什么已经做完了，下一步要做什么都记录明白

## `simulink/core/stereo_ibvs_core.slx`

Stage 1 核心模型已经完成，并通过 MATLAB R2025b Update Diagram。当前使用固定相机内参和固定逆深度，执行左相机图像中心 IBVS，输出 `7×1` 关节速度；EKF 和主动变焦暂为后续阶段接口。顶层端口编译类型均为 `double`，采样周期为 `cfg.Ts = 1/120 s`。

Core 的 `InitFcn` 仅在基础工作区缺少完整 `cfg` 时加载正式配置；已有完整 `cfg` 会直接使用。因此 ROS 2 模型仍加载真实标定许可，离线仿真可临时设置 `stage1CalibrationReady=true`，两者不再互相覆盖。

常用顶层接口：

```text
输入：
qRaw                   7×1
visionFeatureRaw       8×1  [validL; validR; uL; vL; uR; vR; scaleL; scaleR]
zoomPositionStepsRaw   2×1
controllerEnableRaw    标量
resetRaw               标量

输出：
jointVelocityCmd       7×1
zoomStepRateCmd        2×1
controllerStatus       12×1
```

`controllerStatus` 顺序：

```text
[inputDataValid; cameraModelValid; kinematicsValid;
 validLeft; validRight; depthMeasurementValid; centerTaskValid;
 safetyValid; ekfValid; zoomControllerValid;
 controllerEnable; jConditionMetric]
```

核心子系统：

```text
01_Input_Validity
02_Camera_Model
03_Feature_Processing
04_FR3_Camera_Kinematics
05_Target_EKF
06_Inverse_Depth_Filter
07_Arm_Priority_Controller
08_Zoom_Controller
09_Safety_Supervisor
10_Diagnostics
```

## `simulink/ros2/stereo_ibvs_ros2_stage1.slx`

Stage 1 ROS 2 包装模型已经完成。Model 块接口与 core 的 `5` 个输入、`3` 个输出一致，包装模型使用 `4` 个 ROS 2 订阅和 `3` 个发布；消息收发已经验证，尚未与真实双目相机进行联合实验。

| 方向 | Topic | 消息类型 / 有效维度 |
|---|---|---|
| 订阅 | `/franka/joint_states` | `sensor_msgs/JointState`，当前取 `position` 前 7 项 |
| 订阅 | `/vision_double/target_features` | `std_msgs/Float64MultiArray`，8 维 |
| 订阅 | `/vision_double/zoom_position_steps` | `std_msgs/Float64MultiArray`，2 维 |
| 订阅 | `/simulink/reset` | `std_msgs/Bool`，新消息且为 true 时产生单采样复位脉冲 |
| 发布 | `/simulink/target_joints_velocities` | `std_msgs/Float64MultiArray`，7 维 |
| 发布 | `/simulink/zoom_step_rate_cmd` | `std_msgs/Float64MultiArray`，2 维 |
| 发布 | `/simulink/controller_status` | `std_msgs/Float64MultiArray`，12 维 |

已实现关节/视觉/变焦数组最小长度 `7/8/2` 检查；长度不满足时禁止控制。三个输出按 ROS 总线的 128 元素缓存生成，并设置正确的有效长度。

ROS 输入安全监督已经完成：

- 使用 JointState 和 Vision Subscribe 的 `IsNew` 监控新鲜度，超时均为 `0.10 s`；
- Stage 1 仅以左相机 `validL` 判断目标是否丢失，右相机不触发停车；
- 运行中连续第 3 帧左目标无效时关闭内部使能，短暂丢失期间保持最后一次合法视觉特征；
- 初始启动和故障恢复均要求连续 3 帧左目标有效；
- 从使能切换到停止时自动产生一次 Core 复位脉冲；人工 `/simulink/reset` 还会清除监督器保存的视觉特征和计数；
- `/simulink/controller_enable` 已删除，内部使能完全由包装模型自动生成；
- Simulink 只负责切换到零目标，最终平滑减速由 Python `velocity_command_node` 的 `0.20 rad/s²` 加速度限制完成。

MATLAB R2025b Update Diagram 和临时动态测试均通过；动态测试覆盖三帧丢失、三帧恢复、Vision 超时、JointState 超时和人工复位。

## `simulink/config/stereo_ibvs_config.m`

已移除机器人模型中的 `fr3_leftfinger` 和 `fr3_rightfinger` 分支，Jacobian 已由错误的 `6×9` 修正为 `6×7`，并增加 7 自由度检查。

## `simulink/sim/stereo_ibvs_sim_stage1.slx`

Stage 1 离线仿真已由 `simulink/build/sim/build_stereo_ibvs_sim_stage1.m` 重新生成。模型根据自身 `.slx` 位置查找 `core/` 和 `config/`，不再保存本机绝对路径；离线标定许可只在当前 MATLAB 工作区临时开启，不修改正式配置文件。

验证结果：启动关闭、目标居中和目标丢失阶段的关节速度均为零；目标偏离阶段最大绝对关节速度为 `0.03 rad/s`。

## Python ROS 2 节点

当前保留单目/双目视觉检测、通用安全函数和 `velocity_command_node`。`velocity_mapper_node.py` 已删除，Stage 1 关节速度由 Simulink 直接计算。setup、launch、YAML 和 Python 默认 Topic 中的旧 Mapper 配置已经清理。

`full_system.launch.py` 当前组合 `vision_double_node` 和可选 FR3 硬件链路；默认 `start_hardware:=false`、`command_mode:=zero`。MATLAB/Simulink 仍需单独运行。

### `velocity_servo_tag/vision/vision_double_node.py`

Stage 1 双目视觉节点代码已经完成：

- 左右普通 USB 相机 ID、分辨率和帧率由 YAML 配置，默认 `640×480 @ 60 Hz`；
- 左右相机各使用独立采集线程和独立 AprilTag 检测器；
- 发布 `/vision_double/target_features`：`[validL; validR; uL; vL; uR; vR; scaleL; scaleR]`；
- `scale` 定义为 AprilTag 四角像素面积的平方根；
- 检测结果超时后对应侧 `valid` 置零；双目时间差超限时较旧一侧置零；
- 以目标 `60 Hz` 发布 `/vision_double/zoom_position_steps = [0;0]`，仅作为 Stage 1 占位，不控制电机；
- 已增加 `vision_double.launch.py`、YAML 参数、setup 入口和 6 项纯算法单元测试。

代码语法和纯算法测试已经通过；尚未连接两台真实 USB 相机验证实际帧率、设备 ID 和 USB 带宽。

## 当前限制

- 真实标定许可仍为 `false`，所以安全模块会把实际关节速度锁为零；
- `JointState.position` 尚未根据 `JointState.name` 重排，真机前必须核对关节顺序；
- Stage 1 目标丢失只监控左相机，右相机仅用于数据记录和诊断；
- ROS 2 块当前 QoS 为 Reliable、Volatile、Keep last、Depth 10，联调时需确认发布端兼容；
- 全新 MATLAB 会话首次加载时可能先提示找不到 core，模型 PostLoad 加入路径后执行 Ctrl+D 可以通过；
- ROS 2 通信已经验证，但新双目节点尚未与真实双相机和 Simulink 联合运行；
- 普通 USB 相机没有硬件同步，当前使用软件单调时钟检查双目检测时间差；
- Zoom 位置当前固定为 `[0;0]`，没有真实位置反馈；
- 当前自定义 C++ `JointVelocityExampleController` 没有命令 freshness watchdog，不能用关闭终端代替实体急停。

## 下一步

1. 按 README 的实验顺序，对全部代码、Launch、Topic、配置和安全链路进行一次实验前全面检查；
2. 在 Ubuntu 24.04 / ROS 2 Jazzy 上重新构建包，验证 Launch 参数和 `full_system.launch.py`；
3. 设置左右相机 ID，验证两台 USB 相机、实际检测帧率和输出 Topic；
4. 将 `vision_double_node` 与 Simulink 联合运行，并采集真实 `/franka/joint_states` 核对关节顺序；
5. 启动控制器后核对命令 Topic 订阅者和 `filter_coefficient=0.01`；
6. 完成真实标定后，对接 Python 最终安全转发链路并进行零速到低速真机验证。

# 工作日志

说明：每次做完工作后记录到分钟，每次改完代码后都要在Memory.md里记录本次工作的内容和时间，精确到分钟，格式为：（按逐个文件说）
1:修改什么文件
2:修改了什么内容（简要概括）
3:修改的原因，目的，作用
4:备注（可选写和不写）


## 2026-07-21：完成 Stage 1 ROS 2 包装模型

修改文件：

```text
simulink/ros2/stereo_ibvs_ros2_stage1.slx
simulink/config/stereo_ibvs_config.m
config/velocity_servo_tag.yaml
Memory.md
```

完成内容：创建 ROS 2 包装模型；接入 5 个订阅和 3 个发布；加入输入长度门控、单周期复位和 ROS 数组有效长度处理；修正 FR3 模型为 7 自由度；通过 MATLAB R2025b 编译和短时 ROS 2 仿真测试。

备注：Python 节点未修改；真机联调前仍需完成标定许可、JointState 顺序确认和输入超时保护。

## 2026-07-21：重构项目 Memory

修改文件：`Memory.md`

完成内容：按固定六段格式整理项目记忆，保留常用接口、当前状态、后续计划和配置分工，删除重复的子系统说明。

## 2026-07-22：重写工程 README 并校正项目记录

修改文件：

```text
README.md
memory.md
```

完成内容：按照工程项目 README 结构重新记录项目目标、当前状态、系统架构、Ubuntu 24.04 / ROS 2 Jazzy 环境、构建方式、Stage 1 ROS 2 使用方法、接口、安全机制、验证范围和路线图；不包含未完成的 sim 使用方法。

备注：同步将实际关节速度输出 Topic 修正为 `/simulink/target_joints_velocities`，采样周期修正为 `1/120 s`，并记录 ROS 通信已经验证、双目相机联合实验尚未完成。Mapper 遗留配置将在下一步单独清理。

## 2026-07-22：完成 Stage 1 双目视觉节点和 Zoom 零占位

修改文件：

```text
velocity_servo_tag/vision/stereo_features.py
velocity_servo_tag/vision/vision_double_node.py
launch/vision_double.launch.py
config/velocity_servo_tag.yaml
setup.py
test/test_stereo_features.py
README.md
memory.md
```

完成内容：实现左右 USB 相机独立采集和 AprilTag 检测，发布 8 维双目特征；加入检测新鲜度和双目时间差检查；以 60 Hz 目标频率发布 `[0;0]` Zoom 位置占位；增加 YAML、launch、安装入口、README 使用说明和纯算法测试。

验证：新增 Python 文件和 launch 通过语法检查；尺度、有效双目、检测超时、双目时间差和 Zoom 占位共 6 项单元测试通过。

备注：当前没有 Zoom 电机控制；真实双 USB 相机、实际 60 Hz 性能和 Simulink 联合运行仍待 Ubuntu 24.04 / ROS 2 Jazzy 环境验证。

## 2026-07-22：清理 Mapper 遗留配置并重构完整启动入口

修改文件：

```text
setup.py
package.xml
config/velocity_servo_tag.yaml
launch/velocity_servo_tag.launch.py
launch/full_system.launch.py
velocity_servo_tag/velocity_command_node.py
README.md
memory.md
```

完成内容：删除已不存在的 Mapper executable 和整段 YAML 参数；将 `velocity_command_node` 默认输入统一为 `/simulink/target_joints_velocities`；将 YAML 中 Simulink 接口记录同步为 `1/120 s` 和实际关节速度 Topic；上层 launch 改为启动 `vision_double_node`；`full_system.launch.py` 改为组合双目视觉和可选 FR3 硬件链路。

默认安全行为：`start_hardware:=false`、`start_vision:=true`、`command_mode:=zero`。Simulink 仍需单独启动，非零真机运动仍受标定、JointState 顺序和输入 watchdog 限制。

## 2026-07-22：加入 Stage 1 ROS 输入安全监督并整理工程缓存

修改文件：

```text
.gitignore
simulink/config/stereo_ibvs_config.m
simulink/ros2/stereo_ibvs_ros2_stage1.slx
simulink/build/sim/build_stereo_ibvs_sim_stage1.m
config/velocity_servo_tag.yaml
README.md
memory.md
```

完成内容：删除外部 `/simulink/controller_enable`，由包装模型根据 Joint/Vision freshness、数组长度和左相机 `validL` 自动产生内部使能；加入连续 3 帧丢失停止、连续 3 帧有效恢复、合法视觉保持和故障单脉冲复位；保留人工 `/simulink/reset`。平滑停车继续由 Python 最终命令节点负责，不在 Simulink 重复实现加速度限制。

配置：JointState 和 Vision 超时均为 `0.10 s`，目标判断只使用左相机。整理离线生成脚本路径和函数名，并将 Python、ROS 2、MATLAB/Simulink 和 macOS 缓存统一加入 `.gitignore`。

验证：包装模型通过 MATLAB R2025b Update Diagram；临时动态 Simulink 测试验证第 3 帧丢失停机、连续 3 帧恢复、Vision 超时、JointState 超时和人工复位；项目文件未进行真实 ROS 2 双相机或 FR3 非零运动测试。

## 2026-07-22：按实验流程精简重写 README

修改文件：

```text
README.md
memory.md
```

完成内容：按照“概述、项目框架、部署、运行、数据流向、ROS 2 接口、配置、安全、分阶段目标、测试”的结构重写 README；将编译和 source 命令分开，补充四个 Launch 的启动示例、全部可传入参数，以及标明节点、Simulink、Topic 和消息尺寸的 Mermaid 数据流图；测试章节按双相机、零速度全链路、JointState、标定和 FR3 低速实验排列。

下一项工作：开始实验前全面检查，确认代码和配置是否适合按照 README 的测试顺序进入真实设备实验。

## 2026-07-22：修正离线仿真配置覆盖并补充控制器依赖

修改文件：

```text
README.md
simulink/build/sim/build_stereo_ibvs_sim_stage1.m
simulink/core/stereo_ibvs_core.slx
simulink/sim/stereo_ibvs_sim_stage1.slx
memory.md
```

完成内容：Core 只在缺少完整 `cfg` 时加载正式配置；离线仿真改为按 `.slx` 位置寻找 core 和 config，并修正生成脚本的候选路径维度和首次保存顺序；README 明确依赖 `sunflower050105/franka_ros2` 的 `jazzy` 分支及其自定义 `JointVelocityExampleController`，补充控制器、Topic 订阅者和滤波参数检查命令，并记录 C++ 控制器当前没有命令 freshness watchdog。

验证：Core 和 ROS 2 包装模型通过 MATLAB R2025b Update Diagram；离线仿真通过 6 秒场景测试，偏移阶段最大绝对关节速度为 `0.03 rad/s`，关闭、居中和丢失阶段均为零；正式标定配置仍保持关闭。

## 2026-07-23 02:26：精简 README 运行与配置说明

修改文件：

```text
README.md
memory.md
```

完成内容：精简四个 Launch 的启动说明，只列出实际启动的节点或组件；在 MATLAB/Simulink 启动部分区分终端命令和 MATLAB 命令窗口，并补充每次启动及修改不同文件后的操作；配置表改为“配置文件、谁读取、控制哪些文件或节点”。保留用户已修改的 Python 虚拟环境说明，不修改 Launch 代码。

## 2026-07-22 工作总结

记录时间：2026-07-23 02:44

### `AGENTS.md`

- 修改：增加修改完成后提供中文 Commit 内容和文件分组的协作要求。
- 目的：方便后续按功能整理和提交修改。

### `.gitignore`

- 修改：补充 Python、ROS 2、MATLAB/Simulink 和 macOS 自动生成文件的忽略规则。
- 目的：避免缓存、构建目录和临时文件进入 Git。

### `README.md`

- 修改：重写项目概述、部署、运行、数据流、ROS 2 接口、配置、安全机制、阶段目标和测试流程；补充 sunflower `franka_ros2` 依赖、控制器检查命令、四个 Launch 的启动组件、MATLAB 启动和文件修改后的操作。
- 目的：形成可以直接用于 Stage 1 部署和实验的工程说明。
- 备注：真实双相机和 FR3 非零实验尚未完成。

### `memory.md`

- 修改：重构项目目标、文件架构、文件状态、实现计划和配置关系；同步当天全部修改和验证结果；规定后续日志按文件逐个记录。
- 目的：保证下一次工作能够直接接续当前状态。

### `setup.py`

- 修改：增加双目视觉节点安装入口并删除 Mapper 遗留入口。
- 目的：使 ROS 2 能够安装和启动 `vision_double_node`，同时清理已删除节点。

### `package.xml`

- 修改：清理已删除 Mapper 相关内容并同步当前运行依赖。
- 目的：让包描述与实际节点和启动方式一致。

### `config/velocity_servo_tag.yaml`

- 修改：增加双相机、AprilTag、60 Hz 发布、特征超时、Zoom 零占位和最终速度命令参数；统一 Simulink Topic 与安全参数记录。
- 目的：集中配置 `vision_double_node` 和 `velocity_command_node`。
- 备注：其中 `simulink_ros2` 段只记录接口，不会自动修改 Simulink 模型。

### `launch/vision_double.launch.py`

- 修改：新增双目视觉独立启动入口。
- 目的：便于单独启动和测试 `vision_double_node`。

### `launch/velocity_servo_tag.launch.py`

- 修改：改为启动 `vision_double_node`，并保留 `start_vision` 开关。
- 目的：作为 `full_system.launch.py` 的视觉入口，清理旧 Mapper 链路。

### `launch/full_system.launch.py`

- 修改：组合双目视觉和可选 FR3 硬件链路，默认 `start_hardware=false`、`command_mode=zero`。
- 目的：提供当前 Stage 1 的统一 ROS 2 启动入口。
- 备注：MATLAB/Simulink 仍需单独启动。

### `velocity_servo_tag/vision/stereo_features.py`

- 修改：新增双目特征组合、检测新鲜度、左右时间差和 Zoom 零占位算法。
- 目的：把左右 AprilTag 检测结果整理为 Simulink 使用的 8 维输入。

### `velocity_servo_tag/vision/vision_double_node.py`

- 修改：实现左右 USB 相机独立采集和 AprilTag 检测，发布双目特征与 `[0,0]` Zoom 位置。
- 目的：完成 Stage 1 双目视觉 ROS 2 输入节点。
- 备注：相机 ID、USB 带宽和真实 60 Hz 性能待 Ubuntu 实机验证。

### `velocity_servo_tag/velocity_command_node.py`

- 修改：统一接收 `/simulink/target_joints_velocities`；保留 zero/topic 模式、七维检查、速度限制、加速度限制、超时和平滑停车。
- 目的：作为 Simulink 与 FR3 C++ 速度控制器之间的最终安全转发层。

### `test/test_stereo_features.py`

- 修改：新增尺度、有效双目、检测超时、双目时间差和 Zoom 占位测试。
- 目的：验证不依赖真实相机的双目特征算法。
- 备注：共 6 项测试已经通过。

### `simulink/config/stereo_ibvs_config.m`

- 修改：加入 Joint/Vision freshness、目标丢失和恢复帧数等包装层参数，并保持真实标定许可为 `false`。
- 目的：统一 Core、ROS 2 包装和离线仿真的 Stage 1 参数。

### `simulink/ros2/stereo_ibvs_ros2_stage1.slx`

- 修改：删除外部 controller enable；加入 Joint/Vision 超时、三帧目标丢失停止、三帧恢复、合法视觉保持、自动内部使能和故障复位。
- 目的：在 Simulink 包装层完成输入有效性和目标丢失保护。
- 备注：Update Diagram 和动态安全场景测试已经通过。

### `simulink/build/sim/build_stereo_ibvs_sim_stage1.m`

- 修改：整理生成脚本名称和路径；改用相对模型位置查找 Core 与配置；修正候选路径维度和首次保存顺序。
- 目的：让离线仿真模型能够在不同用户名和工作区路径下重新生成。

### `simulink/core/stereo_ibvs_core.slx`

- 修改：调整 InitFcn，仅在缺少完整 `cfg` 时加载正式配置。
- 目的：避免 Core 覆盖离线仿真的临时标定许可，同时保持真实 ROS 2 模型默认锁零。
- 备注：MATLAB R2025b Update Diagram 已通过。

### `simulink/sim/stereo_ibvs_sim_stage1.slx`

- 修改：重新生成模型并移除 `/home/harry/...` 绝对路径。
- 目的：恢复可移植的 Stage 1 离线仿真。
- 备注：偏移阶段最大绝对关节速度为 `0.03 rad/s`，关闭、居中和目标丢失阶段均为零。

### 下一步

1. 在 Ubuntu 24.04 / ROS 2 Jazzy 编译项目并检查自定义速度控制器；
2. 设置左右相机 ID，验证 AprilTag 检测和实际发布频率；
3. 保持 `command_mode=zero`，完成双相机、Simulink、Python 和控制器全链路联调；
4. 核对 `/franka/joint_states` 前七项的关节顺序；
5. 完成左相机内参和手眼标定；
6. 准备实体急停后进行 FR3 低速实验；
7. Stage 1 实验验证完成后再进入双目深度等 Stage 2 内容。

## 2026-07-23 03:11：补充首次使用配置说明

### `README.md`

- 修改：在配置文件章节增加“首次使用需要配置的参数”，记录相机编号、采集模式、AprilTag、机器人 IP、相机内参、手眼矩阵、固定目标深度和标定许可的位置及获取方法；原配置关系和常用参数分别整理为 7.2、7.3。
- 目的：让首次部署和标定时能够直接确认需要修改哪些参数，以及每个参数从哪里获得。
- 备注：Stage 1 暂不要求双目标定和 Zoom 标定，速度、安全超时及底层滤波继续保留当前默认值。

### `memory.md`

- 修改：按逐文件格式记录本次 README 配置说明更新和时间。
- 目的：保证后续工作能够追踪配置文档的修改原因。


## 2026-07-24 03:13：记录 V2 重构对齐方案与待办

### 1：修改什么文件

`memory.md`

### 2：修改了什么内容

记录以下已确认、尚未实施的待办：

1. Core 关节速度限制改为七维整组同比例缩放，算法层上限设为每轴 `0.03 rad/s`，保持控制器给出的关节速度方向；参数注释同时写明 FR3 官方全局关节速度上限为 `[2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26] rad/s`，官方上限继续作为硬件能力边界。
2. Core 新增七维整组同比例加速度限制，算法层上限设为每轴 `0.20 rad/s²`；按照 `cfg.Ts` 和上一周期实际输出计算允许的速度增量。
3. Python 保留 `max_velocity_scale = 0.10` 作为比 Core 宽松的最终速度安全保护，并继续按 FR3 官方关节速度上限计算各轴最终上限，再对七维速度整组同比例缩放。
4. Python 加速度上限改为每轴 `0.40 rad/s²`，加速度限制也改为七维整组同比例缩放。Core 为 `60 Hz` 时，`0.20 / 60 ≈ 0.00333 rad/s`；Python 为 `120 Hz` 时，`0.40 / 120 ≈ 0.00333 rad/s`，因此正常运行时 Python 基本不会对 Core 输出进行二次限幅。
5. 新建独立 ROS 2 自定义接口包，视觉消息包含左右特征、左右帧序号和左右采集时间；Python 只在相机实际处理新帧时发布，未检测到 Tag 的新帧也发布 `valid=false`，相机断流时停止发布并交给 watchdog。
6. 左右相机独立更新：左相机新帧可以单独更新中心控制任务；只有左右都出现新的合格配对时，才更新双目深度和 EKF。
7. ROS 2 包装层按照 `JointState.name` 将 `position` 和 `velocity` 重排为 FR3 固定关节顺序，并检查消息时间戳、长度和新鲜度；`qMeasured` 用于运动学，`qDotMeasured` 用于相机速度与 `rhoDot`，Python 最终命令只用于安全转发和诊断。

### 3：修改的原因、目的、作用

使当前 ROS 架构尽量复现已通过仿真验证的 V2 控制逻辑，同时避免逐关节裁剪改变七维速度方向；明确 Core 与 Python 两层速度、加速度保护的职责，并保证 EKF 只使用真实的新视觉测量和机器人实际速度反馈。

### 4：备注

本次只记录方案，没有修改 MATLAB、Simulink、Python、ROS 2 消息或配置文件；后续实施前仍需再次确认并由用户明确开始。

## 2026-07-24 03:28：Core 关节速度改为整组同比例限幅

### `simulink/config/stereo_ibvs_config.m`

- 修改：保留 Core 每轴 `0.03 rad/s` 算法层速度上限，并在参数旁注明 FR3 官方全局关节速度上限 `[2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26] rad/s`；明确 `cfg.qDotMax` 继续作为硬件能力边界。
- 原因、目的、作用：区分算法实验速度与机器人官方硬件速度能力，避免后续调参时混淆两层限制。

### `simulink/build/core/build_stereo_ibvs_core.m`

- 修改：将 09 安全模块的关节速度限制由逐关节裁剪改为七维整组同比例缩放；先计算各轴超限比例，再用全组最大比例统一缩小七维速度。
- 原因、目的、作用：在未触发关节软限位等安全例外时保持控制器输出的七维速度方向，避免逐轴裁剪改变机械臂运动方向。
- 备注：按用户要求只维护完整 Core Build；`build_09_safety_and_saturation.m` 未保留本次修改。

### `simulink/core/stereo_ibvs_core.slx`

- 修改：使用完整 `build_stereo_ibvs_core.m` 重新生成 Core 模型，使 09 安全模块采用新的整组同比例限速逻辑。
- 原因、目的、作用：保证当前可直接加载的 Core 模型与唯一完整构建脚本一致。
- 备注：MATLAB R2025b 完整构建和 Update Diagram 均通过；09 子系统 `10` 个输入、`15` 个输出全部保持连接。超限测试缩放系数为 `2.0`，最大绝对关节速度为 `0.03 rad/s`，归一化方向误差为 `0`。

### `memory.md`

- 修改：记录本次配置、完整构建脚本、生成模型和验证结果。
- 原因、目的、作用：保证后续工作能够追踪 Core 限速逻辑从逐轴裁剪改为整组同比例缩放的原因和验证范围。
- 备注：完整 Build 自动生成了两个未跟踪备份文件 `stereo_ibvs_core_backup_20260724_032051.slx` 和 `stereo_ibvs_core_backup_20260724_032606.slx`，用于恢复，不纳入本次建议提交。

## 2026-07-24 03:37：Core 与 Python 加速度改为整组同比例限制

### `simulink/config/stereo_ibvs_config.m`

- 修改：新增 `cfg.qDDotAlgorithmMax = 0.20 * ones(7,1)`，作为 Core 算法层关节加速度上限。
- 原因、目的、作用：集中配置 Core 的七维速度增量限制，保证每个控制周期按 `cfg.Ts` 正确换算允许增量。

### `simulink/build/core/build_stereo_ibvs_core.m`

- 修改：为 09 安全模块增加上一周期 `qDotApplied` 反馈输入和 `qDDotAlgorithmMax` 常量；速度限幅后计算目标速度增量，并按七轴最大加速度比例统一缩放；关节软限位与笛卡尔速度保护继续作为更高优先级安全约束。
- 原因、目的、作用：在正常控制过程中限制速度变化率，同时保持七维速度增量方向；使用已有 Unit Delay 显式反馈，避免在 MATLAB Function 内隐藏状态。
- 备注：只修改完整 Core Build，没有修改任何单模块 Build 文件。

### `simulink/core/stereo_ibvs_core.slx`

- 修改：使用完整 Build 重新生成模型；09 安全模块增加第 `11` 个输入，用于接收上一周期实际输出速度。
- 原因、目的、作用：使当前 Core 模型实际执行 `0.20 rad/s²` 的整组同比例加速度限制。
- 备注：MATLAB R2025b 完整构建和 Update Diagram 通过；09 子系统为 `11` 个输入、`15` 个输出，MATLAB Function 为 `30` 个输入。测试最大加速度为 `0.20 rad/s²`，速度增量方向误差为 `0`。

### `velocity_servo_tag/velocity_command_node.py`

- 修改：默认加速度上限由每轴 `0.20 rad/s²` 改为 `0.40 rad/s²`；删除逐轴 `np.clip`，改为根据七轴最大超限比例统一缩放速度增量；同步更新文件说明和函数注释。
- 原因、目的、作用：保持七维速度增量方向，并让 Python 作为比 Core 更宽松的最终安全转发层。
- 备注：`python3 -m py_compile` 通过；静态公式测试最大加速度为 `0.40 rad/s²`。

### `config/velocity_servo_tag.yaml`

- 修改：七个 `max_joint_accelerations` 参数全部改为 `0.40`，并注明 Python `120 Hz` 与 Core `60 Hz` 下的单周期允许速度增量均约为 `0.00333 rad/s`。
- 原因、目的、作用：使运行时 YAML 参数与 Python 默认值和 Core/Python 分层限幅设计一致。
- 备注：YAML 解析与七维参数值检查通过。

### `memory.md`

- 修改：记录本次 Core、Python、YAML、生成模型和验证结果。
- 原因、目的、作用：明确速度与加速度限幅均已改为整组同比例缩放，并记录两层加速度参数的配合关系。
- 备注：完整 Build 新生成未跟踪备份 `stereo_ibvs_core_backup_20260724_033541.slx`，用于恢复，不纳入建议提交。

# 实现计划

说明：如果有分步实行的stage1，2，3，可以记录

- Stage 1：固定内参与固定深度的左相机中心 IBVS。core、ROS 2 包装、消息收发、双目视觉节点和 ROS 输入安全监督已完成；真实双相机联合实验、标定和硬件接入未完成。
- Stage 2：加入双目逆深度滤波与深度闭环任务。
- Stage 3：加入关节限位零空间任务和任务优先级控制。
- Stage 4：加入世界坐标系 9 状态目标 EKF `[p; v; a]` 和目标速度前馈。
- Stage 5：加入双目主动变焦，将累计步数通过标定曲线转换为焦距和相机内参，并处理回零、限位和编码器反馈。
- Stage 6：加入预测、时延补偿、更完整的故障恢复、奇异性监控和真机验证。

# 配置文件

说明：把配置文件，.yaml，.m配置文件简单记录

| 文件 | 作用 |
|---|---|
| `config/velocity_servo_tag.yaml` | 单目/双目检测节点和最终关节速度命令节点的 Topic、相机、watchdog、发布频率及安全限制参数 |
| `config/controllers.yaml` | `ros2_control` 控制器配置 |
| `simulink/config/stereo_ibvs_config.m` | Simulink 的采样时间、FR3 模型、相机参数、控制增益、阻尼、软限位、Joint/Vision 超时、目标丢失/恢复帧数和阶段参数 |
| `config/urdf/fr3.urdf` | `stereo_ibvs_config.m` 当前实际加载的 FR3 模型 |

配置关系：

- Simulink 的主要工作区参数由 `stereo_ibvs_config.m` 设置；模型块内部仍可能包含自身参数；
- Python 节点读取 `velocity_servo_tag.yaml`，Simulink `.slx` 当前不会读取这个 YAML；
- YAML 中现有 `simulink_ros2` 段只是接口记录，不会自动创建或修改 Simulink 接口；当前记录已与 `.slx` 的 `1/60 s`、自定义双目特征接口和 `/simulink/target_joints_velocities` 对齐；
- ROS 包装安全参数实际由 `stereo_ibvs_config.m` 加载：Joint/Vision 超时 `0.10 s`、丢失 `3` 帧停止、恢复 `3` 帧使能，目标来源为左相机 `validL`；
- 当前 `cameraMountCalibrated`、`cameraIntrinsicsCalibrated`、`stereoCalibrationValid` 和 `zoomCalibrationValid` 均为 `false`；
- 后续手眼、双目和变焦标定结果计划独立保存在 `simulink/calibration/`。

## 2026-07-24 13:22：搭建 Core 配套 ROS 2 层与自定义视觉接口

### `simulink/config/stereo_ibvs_config.m`

1. 修改什么文件：`simulink/config/stereo_ibvs_config.m`
2. 修改了什么内容：新增 JointState、自定义双目特征、复位、关节速度命令和控制状态的 Topic 与消息类型；增加 FR3 期望关节名、双目最大配对时间差和 JointState 超时帧数。
3. 修改的原因、目的、作用：集中管理 ROS 2 包装模型的全部接口参数，并为按名称重排关节状态和判断新双目配对提供配置。
4. 备注：Core 仍为 `60 Hz`；视觉超时与 JointState 超时均为 `0.10 s`。

### `simulink/build/core/build_stereo_ibvs_core.m`

1. 修改什么文件：`simulink/build/core/build_stereo_ibvs_core.m`
2. 修改了什么内容：Core 顶层输入扩展为 `10` 个；新增 `qDotMeasuredRaw`、左相机真实新帧和新双目配对事件；01 模块保存实测关节速度并按左帧监督视觉新鲜度；05 EKF 只接收新双目配对事件；06 逆深度动态使用 `JointState.velocity`；09 加速度限幅继续使用 Core 自身上一周期 `qDotApplied`。
3. 修改的原因、目的、作用：让需要机器人运动反馈的估计使用真实关节速度，同时避免将经过真实机器人和底层滤波的速度错误用于 Core 自身命令加速度限幅。
4. 备注：JointState 或视觉超时后，06 的 `motionFeedbackValid` 由 `controllerEnableSafe` 拉低，不再使用保存的旧速度估计 `rhoDot`。

### `simulink/core/stereo_ibvs_core.slx`

1. 修改什么文件：`simulink/core/stereo_ibvs_core.slx`
2. 修改了什么内容：使用唯一完整 Core Build 重新生成 `10` 输入、`3` 输出模型，并包含新的实测速度和视觉事件链路。
3. 修改的原因、目的、作用：保证当前可加载模型与完整构建脚本一致。
4. 备注：MATLAB R2025b 完整构建和 Update Diagram 通过；结构断言确认 06 的第 7 输入来自实测 `qDot`、第 8 输入来自安全使能，`qDot Applied Delay` 仅连接 09 的第 11 输入。

### `simulink/build/ros/build_stereo_ibvs_ros.m`

1. 修改什么文件：`simulink/build/ros/build_stereo_ibvs_ros.m`
2. 修改了什么内容：新增完整 ROS 2 包装模型构建入口；订阅 JointState、自定义双目特征、焦距和复位；按照 `JointState.name` 重排 `position/velocity`；根据左右帧序号和采集时刻产生左帧事件与新双目配对事件；封装并发布关节速度、焦距速度和 13 维状态。
3. 修改的原因、目的、作用：将 Core 接入真实 ROS 2 数据流，同时保持 ROS 消息处理与控制算法分层。
4. 备注：关节名先在 Simulink 块层转换为固定 `uint8` 字节和长度向量，再进入 MATLAB Function；避免把 ROS 字符串总线数组直接送入算法函数。`checkcode` 无问题。

### `simulink/ros2/stereo_ibvs_ros.slx`

1. 修改什么文件：`simulink/ros2/stereo_ibvs_ros.slx`
2. 修改了什么内容：由新 ROS Build 生成最终包装模型，引用 `stereo_ibvs_core`，连接 `10` 个输入和 `3` 个输出，并包含四个订阅接口与三个发布接口。
3. 修改的原因、目的、作用：提供可以直接打开检查、后续部署和联调的 ROS 2 模型。
4. 备注：完整构建和 Update Diagram 均通过；结构断言确认 Topic、消息类型、Core 端口数及关键估计连线正确。

### `velocity_servo_tag_interfaces/msg/StereoFeatures.msg`

1. 修改什么文件：`/Users/hlc/Desktop/Sustech/Franka_python/velocity_servo_tag_interfaces/msg/StereoFeatures.msg`
2. 修改了什么内容：定义左右 `uint32` 帧序号、左右 `builtin_interfaces/Time` 采集时刻、左右有效位、中心像素和尺度特征；不保留多余的总 Header。
3. 修改的原因、目的、作用：让 Simulink 能区分真实新帧、左右独立更新和新的时间合格双目配对；`uint32` 避免 MATLAB 将 ROS `uint64` 转为 `double`。
4. 备注：`60 Hz` 连续运行约 `2.27` 年后序号才回绕；Python 发布端本次未修改。

### `velocity_servo_tag_interfaces/CMakeLists.txt`

1. 修改什么文件：`/Users/hlc/Desktop/Sustech/Franka_python/velocity_servo_tag_interfaces/CMakeLists.txt`
2. 修改了什么内容：建立 `ament_cmake`/`rosidl_default_generators` 消息生成配置，并声明 `builtin_interfaces` 依赖。
3. 修改的原因、目的、作用：使自定义视觉消息能够作为独立 ROS 2 接口包构建。
4. 备注：MATLAB `ros2genmsg` 调用 colcon 构建成功。

### `velocity_servo_tag_interfaces/package.xml`

1. 修改什么文件：`/Users/hlc/Desktop/Sustech/Franka_python/velocity_servo_tag_interfaces/package.xml`
2. 修改了什么内容：新增独立接口包清单、rosidl 生成/运行依赖和 `rosidl_interface_packages` 组声明。
3. 修改的原因、目的、作用：让 ROS 2 工作区正确识别并按接口包方式构建。
4. 备注：XML 语法检查通过。

### `package.xml`

1. 修改什么文件：`package.xml`
2. 修改了什么内容：主包新增 `velocity_servo_tag_interfaces` 依赖。
3. 修改的原因、目的、作用：为后续 Python 视觉节点切换到自定义消息建立依赖关系。
4. 备注：本次没有修改 Python 发布逻辑。

### `config/velocity_servo_tag.yaml`

1. 修改什么文件：`config/velocity_servo_tag.yaml`
2. 修改了什么内容：更新 `simulink_ros2` 接口记录为 `60 Hz`、自定义视觉 Topic/类型、JointState、焦距 mm 接口、焦距速度 mm/s 接口、双目配对阈值和 13 维状态长度。
3. 修改的原因、目的、作用：使 YAML 中的接口说明与新 Core/ROS 模型保持一致。
4. 备注：Ruby YAML 解析和关键字段断言通过；该段仍是接口记录，不会自动配置 Simulink。

### `memory.md`

1. 修改什么文件：`memory.md`
2. 修改了什么内容：记录本次 Core、ROS Build、自定义接口包、生成模型、接口约定和验证结果。
3. 修改的原因、目的、作用：为后续 Python 端按新帧发布自定义消息和真实 ROS 2 联调保留可追踪依据。
4. 备注：本机离线 ROS 2 烟雾仿真中，MathWorks DDS 服务在 macOS 上先出现线程亲和性和互斥锁崩溃，随后 Subscriber 报总线错误；最小 JointState 模型也出现同一 DDS 后端崩溃，因此未把该项计为模型通过或失败。生成的 `matlab_msg_gen/`、`+bus_conv_fcns/` 和 Build 备份均不建议纳入源码提交。

## 2026-07-24 13:30：忽略 MATLAB ROS 自动生成目录

### `.gitignore`

1. 修改什么文件：`.gitignore`
2. 修改了什么内容：新增仓库根目录 `/+bus_conv_fcns/` 和 `/matlab_msg_gen/` 忽略规则；未添加任何 backup 忽略规则。
3. 修改的原因、目的、作用：避免 MATLAB ROS Toolbox 自动生成的消息转换和构建缓存进入版本控制，同时保留 Core 与 ROS 模型备份文件供提交。
4. 备注：当前实际的 `matlab_msg_gen/` 位于本仓库上一级，本规则用于防止以后在本仓库内生成同名目录。

### `memory.md`

1. 修改什么文件：`memory.md`
2. 修改了什么内容：记录本次 `.gitignore` 调整及 backup 保持可提交的约定。
3. 修改的原因、目的、作用：保留生成文件管理规则的变更记录，方便后续提交与维护。

## 2026-07-24 14:14：调整 ROS Build 的仓库内消息路径

### `simulink/build/ros/build_stereo_ibvs_ros.m`

1. 修改什么文件：`simulink/build/ros/build_stereo_ibvs_ros.m`
2. 修改了什么内容：将自定义消息源目录和 `matlab_msg_gen` 生成目录改为新的 Git 仓库根目录；首次缺少生成目录时自动执行 `ros2genmsg(projectRoot)`；模型初始化回调同步从仓库根目录加载生成消息。
3. 修改的原因、目的、作用：适配主 Python 包与 `velocity_servo_tag_interfaces` 接口包在同一仓库中平级存放的新目录结构，避免继续依赖仓库上一级路径。
4. 备注：按要求仅修改，未执行 MATLAB 构建或其他验证。

### `memory.md`

1. 修改什么文件：`memory.md`
2. 修改了什么内容：记录本次 ROS Build 路径调整。
3. 修改的原因、目的、作用：保留目录重构后的构建入口变更记录。

## 2026-07-24 15:03：Python视觉端切换到真实新帧自定义消息

### `velocity_servo_tag/velocity_servo_tag/vision/vision_double_node.py`

1. 修改什么文件：`velocity_servo_tag/velocity_servo_tag/vision/vision_double_node.py`
2. 修改了什么内容：改为发布 `StereoFeatures`；左右相机分别维护 `uint32` 帧序号和近似采集时间；真实新帧无Tag时发布 `valid=false`；断流时不增加序号；只有左右序号变化才发布；删除旧8维数组和Zoom占位发布。
3. 修改的原因、目的、作用：禁止固定频率重复快照被Simulink当作新测量，并支持左相机独立更新及新双目帧对触发EKF。
4. 备注：采集时间使用成功取出图像时的系统时间；焦距由独立节点发布到 `/stereo/focal_length`。

### `velocity_servo_tag/velocity_servo_tag/vision/stereo_features.py`

1. 修改什么文件：`velocity_servo_tag/velocity_servo_tag/vision/stereo_features.py`
2. 修改了什么内容：`CameraFeature` 增加帧序号和纳秒采集时间；新增uint32序号回绕及ROS Time字段拆分函数；删除旧8维快照新鲜度、双目裁剪和Zoom占位算法。
3. 修改的原因、目的、作用：为自定义消息提供无ROS依赖的帧状态与时间转换工具，将双目配对和超时判断统一交给Simulink ROS层。

### `velocity_servo_tag/config/velocity_servo_tag.yaml`

1. 修改什么文件：`velocity_servo_tag/config/velocity_servo_tag.yaml`
2. 修改了什么内容：视觉Topic参数改为 `/vision_double/stereo_features`；删除Python端特征超时、双目时间差和Zoom占位参数；注明焦距由独立节点发布。
3. 修改的原因、目的、作用：使Python运行参数与新的自定义消息链路和Simulink职责一致。
4. 备注：YAML解析通过。

### `velocity_servo_tag/launch/vision_double.launch.py`

1. 修改什么文件：`velocity_servo_tag/launch/vision_double.launch.py`
2. 修改了什么内容：输出说明改为 `StereoFeatures`，并注明本Launch不发布Zoom占位消息。
3. 修改的原因、目的、作用：避免启动说明继续引用已删除的旧Topic。

### `velocity_servo_tag/launch/velocity_servo_tag.launch.py`

1. 修改什么文件：`velocity_servo_tag/launch/velocity_servo_tag.launch.py`
2. 修改了什么内容：输出说明改为 `StereoFeatures`，并明确焦距反馈节点不由该Launch启动。
3. 修改的原因、目的、作用：使上层视觉启动入口与当前ROS接口一致。

### `velocity_servo_tag/test/test_stereo_features.py`

1. 修改什么文件：`velocity_servo_tag/test/test_stereo_features.py`
2. 修改了什么内容：删除旧8维快照和Zoom占位测试，新增uint32序号递增、回绕、越界及纳秒时间戳拆分测试。
3. 修改的原因、目的、作用：覆盖新视觉消息使用的纯算法边界。
4. 备注：共7项单元测试通过。

### `memory.md`

1. 修改什么文件：`memory.md`
2. 修改了什么内容：记录Python真实新帧发布、旧Zoom接口清理和验证结果。
3. 修改的原因、目的、作用：为后续Ubuntu ROS 2联调和焦距反馈接入保留依据。
4. 备注：修改的Python文件语法检查通过，`git diff --check`通过；未连接真实相机或ROS 2运行时。

## 2026-07-24 16:10：更新双目 V2 ROS 架构说明

### `README.md`

1. 修改什么文件：`README.md`
2. 修改了什么内容：保留原有章节、表格和 Mermaid 风格，更新双包仓库结构、自定义 `StereoFeatures`、真实新帧语义、JointState 重排与实测速度反馈、V2 Core/ROS 数据流、焦距接口、13维状态、分层速度和加速度整形、部署构建方法、安全机制及测试顺序。
3. 修改的原因、目的、作用：使使用文档与当前 Core、ROS 包装层、Python 节点和接口包保持一致，删除已经失效的 Stage 1、旧视觉数组和 Zoom 占位接口说明。
4. 备注：明确记录 YAML 双目节点 `640×480` 与 Simulink 当前 `1920×1080` 的待统一事项，以及焦距反馈节点当前由外部提供。

### `velocity_servo_tag/launch/fr3_hardware.launch.py`

1. 修改什么文件：`velocity_servo_tag/launch/fr3_hardware.launch.py`
2. 修改了什么内容：将 `max_velocity_scale` Launch 默认值从 `0.10` 同步为 YAML 当前使用的 `0.50`。
3. 修改的原因、目的、作用：避免 Launch 默认参数覆盖 YAML 后造成文档、配置和实际运行值不一致。
4. 备注：Python 语法检查通过。

### `velocity_servo_tag/launch/full_system.launch.py`

1. 修改什么文件：`velocity_servo_tag/launch/full_system.launch.py`
2. 修改了什么内容：将统一入口的 `max_velocity_scale` 默认值从 `0.10` 同步为 `0.50`。
3. 修改的原因、目的、作用：保证统一入口向硬件 Launch 传递的默认最终速度比例与项目当前配置一致。
4. 备注：Python 语法检查通过。

### `memory.md`

1. 修改什么文件：`memory.md`
2. 修改了什么内容：记录本次 README 和两处 Launch 默认参数同步。
3. 修改的原因、目的、作用：保留项目文档与运行默认值变更的可追踪记录。
4. 备注：`git diff --check` 通过，未运行 Simulink 构建或真实 ROS 2/相机/FR3 联调。

## 2026-07-24 17:35：精简部署配置功能开关

### `simulink/config/stereo_ibvs_config.m`

1. 修改什么文件：`simulink/config/stereo_ibvs_config.m`
2. 修改了什么内容：日常功能开关精简为 `armControlEnable`、`depthTaskEnable`、`zoomControlEnable` 和 `nullspaceEnable`；保留并详细说明五项标定状态；删除中心任务、左相机许可、鲁棒项、Zoom 调度、右相机预测、默认使能和 EKF 策略等重复或固定逻辑开关及其 `cfg` 字段。
3. 修改的原因、目的、作用：减少相互依赖和含义重复的开关，使日常配置只表达 Arm、Depth、Zoom 和 Nullspace 四项功能选择，同时继续在文件开头直观显示标定完成状态。
4. 备注：MATLAB R2025b 配置加载通过；字段断言确认四个功能开关和五个标定状态存在、九个旧开关不存在，未标定状态下 `fullDeploymentReady=0`。Core Build 尚未同步，本次未生成或修改 SLX。

### `memory.md`

1. 修改什么文件：`memory.md`
2. 修改了什么内容：记录本次配置开关精简范围、设计规则和验证结果。
3. 修改的原因、目的、作用：明确 Config 已完成而 Core Build 尚待同步，避免后续工作误判当前过渡状态。

## 2026-07-24 17:39：同步精简后的 Core 完整构建

### `simulink/build/core/build_stereo_ibvs_core.m`

1. 修改什么文件：`simulink/build/core/build_stereo_ibvs_core.m`
2. 修改了什么内容：中心任务固定启用；EKF 固定只使用真实新双目测量；Arm 和 Zoom 鲁棒补偿固定进入计算并由对应 `beta` 数值决定是否产生补偿；左相机有效即可执行中心任务；Zoom 优先级调度直接跟随 `zoomControlEnable`。
3. 修改的原因、目的、作用：消除对九个已删除 `cfg` 开关字段的依赖，同时采用固定逻辑保持现有模型端口和 MATLAB Function 签名不变，降低本次修改范围。
4. 备注：完整 Build、Update Diagram 和七项关键 Constant 结构断言通过；MATLAB `checkcode` 仅提示脚本原有的 `datestr`、`now` 建议及一个未使用变量。

### `simulink/core/stereo_ibvs_core.slx`

1. 修改什么文件：`simulink/core/stereo_ibvs_core.slx`
2. 修改了什么内容：使用更新后的完整 Build 重新生成 Core；顶层继续保持 `10` 个输入和 `3` 个输出。
3. 修改的原因、目的、作用：使生成模型与精简后的 Config 和完整 Build 保持一致。
4. 备注：模型内未检出九个旧 `cfg` 字段；构建自动生成备份 `simulink/core/backup/stereo_ibvs_core_backup_20260724_173834.slx`。

### `memory.md`

1. 修改什么文件：`memory.md`
2. 修改了什么内容：记录完整 Core Build、生成模型、固定逻辑和验证结果。
3. 修改的原因、目的、作用：完成 Config 精简后的 Core 同步记录，明确本次没有维护已删除的模块化 Build 文件。
