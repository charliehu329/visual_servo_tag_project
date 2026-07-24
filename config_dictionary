## Stereo IBVS 配置参数速查表

说明：

* 表中名称省略了 `cfg.` 前缀。
* “推荐”主要对应当前项目的初始测试配置。
* 标注“自动生成/不建议手改”的参数应由其他参数推导。
* 标定类开关只有完成真实测量和验证后才能设为 `true`，不能为了得到非零速度而强行打开。

### 1. 配置版本与运行频率

| 参数名称                    | 含义、作用与推荐值                                                    |
| ----------------------- | ------------------------------------------------------------ |
| `configurationName`     | 配置名称，用于区分不同版本。推荐保持 `v2_full_deployment_mm_interface`。        |
| `configurationVersion`  | 配置文件版本号。当前为 `3`；配置结构变化时再递增。                                  |
| `stage`                 | 项目部署阶段标识。当前为 `5`，主要用于版本管理，不直接参与控制。                           |
| `cameraFps`             | 期望相机帧率。当前 `60 Hz`；应与相机真实输出能力一致。                              |
| `visionRateHz`          | 期望视觉特征更新频率，自动等于 `cameraFps`。不建议单独修改。                         |
| `controlRateHz`         | Core计算频率。推荐 `60 Hz`。                                         |
| `Ts`                    | Core采样周期，自动为 `1/controlRateHz`。60 Hz时为 `0.016667 s`，不建议单独修改。 |
| `simulinkPublishRateHz` | Simulink关节速度和状态发布频率，自动等于 `controlRateHz`。推荐 `60 Hz`。         |
| `pythonSafetyRateHz`    | Python安全转发节点的输出频率。推荐 `120 Hz`。                               |

### 2. ROS 2接口参数

| 参数名称                              | 含义、作用与推荐值                                                           |
| --------------------------------- | ------------------------------------------------------------------- |
| `jointStateTopic`                 | 机械臂关节状态Topic。当前 `/franka/joint_states`。                             |
| `jointStateMessageType`           | 关节状态消息类型。固定为 `sensor_msgs/JointState`。                              |
| `expectedJointNames`              | 期望接收的7个FR3关节名称及顺序。必须保持 `fr3_joint1` 到 `fr3_joint7`。                 |
| `stereoFeatureTopic`              | 双目视觉特征Topic。当前 `/vision_double/stereo_features`。                    |
| `stereoFeatureMessageType`        | 双目视觉特征消息类型。固定为 `velocity_servo_tag_interfaces/StereoFeatures`。      |
| `stereoMaxPairSkewSec`            | 左右相机时间戳允许的最大差值。当前 `0.05 s`；测试推荐 `0.02～0.05 s`，正式同步双目建议尽量减小。         |
| `resetTopic`                      | Simulink复位Topic。当前 `/simulink/reset`。                               |
| `resetMessageType`                | 复位消息类型。固定为 `std_msgs/Bool`；`true/1`请求复位，`false/0`不复位。               |
| `jointVelocityCommandTopic`       | Simulink关节速度输出Topic。当前 `/simulink/target_joints_velocities`。        |
| `jointVelocityCommandMessageType` | 关节速度消息类型。固定为 `std_msgs/Float64MultiArray`。                          |
| `controllerStatusTopic`           | 控制器状态输出Topic。当前 `/simulink/controller_status`。                      |
| `controllerStatusMessageType`     | 控制器状态消息类型。固定为 `std_msgs/Float64MultiArray`。                         |
| `focalLengthTopic`                | 左右相机焦距反馈Topic。当前 `/stereo/focal_length`。                            |
| `focalLengthMessageType`          | 焦距反馈消息类型。固定为 `std_msgs/Float64MultiArray`。                          |
| `focalLengthMessageLength`        | 焦距反馈数组长度。固定为 `2`，顺序为 `[fL_mm; fR_mm]`。                              |
| `focalLengthRateHz`               | 期望焦距反馈频率。推荐 `60 Hz`。                                                |
| `focalLengthInputUnit`            | 焦距反馈单位。固定为 `mm`。                                                    |
| `focalLengthTimeoutFrames`        | 连续多少个Core周期没有新焦距后判定焦距不新鲜。当前 `5`帧。                                   |
| `focalLengthTimeoutSec`           | 焦距超时时间，自动为 `focalLengthTimeoutFrames/controlRateHz`。当前约 `0.0833 s`。 |
| `focalRateCommandTopic`           | Zoom焦距速度命令Topic。当前 `/simulink/focal_rate_cmd`。                      |
| `focalRateCommandMessageType`     | Zoom命令消息类型。固定为 `std_msgs/Float64MultiArray`。                        |
| `focalRateMessageLength`          | Zoom命令数组长度。固定为 `2`，顺序为左右镜头。                                         |
| `focalRateCommandUnit`            | Zoom命令单位。固定为 `mm/s`。                                                |
| `focalLengthStateInitialMm`       | 焦距状态记忆的启动初值。固定推荐 `[0;0]`；该值不代表有效焦距。                                 |

### 3. FR3机器人模型

| 参数名称                   | 含义、作用与推荐值                                                   |
| ---------------------- | ----------------------------------------------------------- |
| `urdfPath`             | FR3 URDF文件路径。应指向项目中的真实 `fr3.urdf`。                          |
| `fr3RobotName`         | 从URDF读取的机器人名称。自动生成，不建议手改。                                   |
| `fr3OriginXYZ`         | 从URDF读取的各关节平移参数。自动生成，不建议手改。                                 |
| `fr3OriginRPY`         | 从URDF读取的各关节旋转参数。自动生成，不建议手改。                                 |
| `fr3Axis`              | 从URDF读取的各关节旋转轴。自动生成，不建议手改。                                  |
| `fr3Joint8OriginXYZ`   | 从URDF读取的 `fr3_joint8` 平移参数。自动生成。                            |
| `fr3Joint8OriginRPY`   | 从URDF读取的 `fr3_joint8` 旋转参数。自动生成。                            |
| `fr3QMinURDF`          | URDF中的7个关节位置下限。自动生成。                                        |
| `fr3QMaxURDF`          | URDF中的7个关节位置上限。自动生成。                                        |
| `fr3QDotMaxURDF`       | URDF中的7个关节速度上限。自动生成。                                        |
| `robot`                | MATLAB导入的 `rigidBodyTree` 机器人对象。自动生成，不直接修改。                 |
| `robotBaseName`        | 机器人基坐标系名称。自动从URDF读取。                                        |
| `cameraBodyName`       | 添加到机器人模型中的左相机坐标系名称。当前 `left_camera_optical`。                |
| `cameraParentBodyName` | 左相机安装所依附的机器人Link。当前 `fr3_link8`；必须与真实安装关系一致。                |
| `jointNames`           | Core内部使用的7个关节名称和顺序。必须保持 `fr3_joint1` 到 `fr3_joint7`。        |
| `q0`                   | 启动和离线计算使用的初始关节位置。当前 `[0;-π/4;0;-3π/4;0;π/2;π/4]`，必须位于关节限位内。 |
| `qMin`                 | 实际采用的关节位置下限。自动从URDF读取。                                      |
| `qMax`                 | 实际采用的关节位置上限。自动从URDF读取。                                      |
| `qMid`                 | 各关节中位位置，自动为 `(qMin+qMax)/2`，用于零空间回中。                        |
| `qDotMax`              | FR3硬件关节速度上限。自动从URDF读取，不建议人为增大。                              |
| `jointTorqueMax`       | FR3关节力矩上限。自动从URDF读取。                                        |
| `qInitial`             | `q0`的兼容别名。自动等于 `q0`，不建议单独修改。                                |

### 4. 坐标系、手眼关系与双目外参

| 参数名称                       | 含义、作用与推荐值                                                            |
| -------------------------- | -------------------------------------------------------------------- |
| `T_W_B`                    | 机器人基坐标系相对于世界坐标系的齐次变换。若世界系与基座系重合，推荐 `eye(4)`。                         |
| `T_link8_CL`               | 左相机坐标系相对于 `fr3_link8` 的手眼变换。必须替换为真实标定结果。                             |
| `T_CL2L8`                  | `T_link8_CL`的兼容别名。自动生成，不建议单独修改。                                      |
| `cameraMountCalibrated`    | 手眼标定完成声明。`true/1`表示已真实标定并验证；`false/0`表示仍为占位值。未标定时必须保持 `false`。       |
| `cameraMountIsPlaceholder` | 手眼参数是否为占位值。自动为 `~cameraMountCalibrated`；`true/1`表示占位，`false/0`表示已标定。 |
| `baseline`                 | 左右相机光心间距，单位m。当前 `0.12 m`；必须替换为真实双目标定值。                               |
| `B`                        | `baseline`的兼容别名。自动生成。                                                |
| `stereoBaseline`           | `baseline`的兼容别名。自动生成。                                                |
| `R_CL_CR`                  | 右相机相对于左相机的旋转矩阵。当前 `eye(3)`仅适用于理想校正后的平行双目；正式使用真实标定结果。                 |
| `p_CL_CR`                  | 右相机相对于左相机的平移向量。当前 `[baseline;0;0]`；正式使用真实标定结果。                       |
| `T_CL_CR`                  | 右相机相对于左相机的完整齐次变换，由 `R_CL_CR`和`p_CL_CR`组成。自动生成。                       |
| `stereoCalibrationValid`   | 双目外参有效声明。`true/1`表示左右外参和baseline已标定验证；`false/0`表示不能启用双目深度。           |

### 5. 图像尺寸与相机内参

| 参数名称                             | 含义、作用与推荐值                                                      |
| -------------------------------- | -------------------------------------------------------------- |
| `imageWidthPx`                   | 图像宽度，单位pixel。当前 `1920`，必须与视觉节点真实输出一致。                          |
| `imageHeightPx`                  | 图像高度，单位pixel。当前 `1080`，必须与视觉节点真实输出一致。                          |
| `imageWidth`                     | `imageWidthPx`的兼容别名。自动生成。                                      |
| `imageHeight`                    | `imageHeightPx`的兼容别名。自动生成。                                     |
| `cxL`                            | 左相机主点横坐标。当前暂用 `imageWidthPx/2`；正式使用标定值。                        |
| `cyL`                            | 左相机主点纵坐标。当前暂用 `imageHeightPx/2`；正式使用标定值。                       |
| `cxR`                            | 右相机主点横坐标。当前暂用图像中心；正式使用标定值。                                     |
| `cyR`                            | 右相机主点纵坐标。当前暂用图像中心；正式使用标定值。                                     |
| `outputPixelPitchXmm`            | 水平方向等效输出像元尺寸，单位 `mm/pixel`。当前 `2.90e-3`为占位值，必须通过真实焦距和像素焦距关系确定。 |
| `outputPixelPitchYmm`            | 垂直方向等效输出像元尺寸，单位 `mm/pixel`。当前 `2.90e-3`为占位值，必须真实测量。            |
| `pixelPitchCalibrated`           | 等效像元尺寸标定声明。`true/1`表示X/Y像元换算已验证；`false/0`表示仍为占位值。              |
| `cameraIntrinsicsCalibrated`     | 相机内参标定声明。`true/1`表示主点、畸变和成像模型已标定；`false/0`表示未完成。               |
| `focalMmToPixelsIsPlaceholder`   | mm焦距到pixel焦距换算是否为占位参数。自动生成；`true/1`表示不可用于真机控制。                 |
| `cameraIntrinsicsArePlaceholder` | 相机内参整体是否仍包含占位参数。自动生成；`true/1`表示不能通过相机模型安全锁。                    |

### 6. 焦距范围与Zoom执行器

| 参数名称                                 | 含义、作用与推荐值                                                     |
| ------------------------------------ | ------------------------------------------------------------- |
| `focalLengthHardwareMinMm`           | 左右镜头硬件允许的最小焦距。当前 `[5;5] mm`；必须依据镜头规格确认。                       |
| `focalLengthHardwareMaxMm`           | 左右镜头硬件允许的最大焦距。当前 `[99;99] mm`；必须依据镜头规格确认。                     |
| `focalLengthWorkingMinMm`            | 控制器允许使用的最小工作焦距。当前 `[10;10] mm`，应比硬件极限保守。                      |
| `focalLengthWorkingMaxMm`            | 控制器允许使用的最大工作焦距。当前 `[90;90] mm`，应给硬件极限保留余量。                    |
| `focalRateGuaranteedMmPerSec`        | 镜头能够稳定达到的焦距变化速度。当前 `15.5 mm/s`；必须通过硬件实验验证。                    |
| `focalRateAbsoluteMaxMmPerSec`       | Zoom焦距变化速度硬上限。当前 `18.75 mm/s`；不得超过硬件能力。                       |
| `focalRateUnit`                      | 焦距速度单位。固定为 `mm/s`。                                            |
| `etaZoom`                            | Zoom设计速度相对稳定能力的利用比例。当前 `0.60`；推荐先用 `0.3～0.6`。                 |
| `focalRateDesignMmPerSec`            | 控制器正常使用的焦距设计速度，自动为 `etaZoom × focalRateGuaranteedMmPerSec`。   |
| `rightReacquireZoomRateMmPerSec`     | 右相机重新搜索目标时的Zoom速度。当前等于设计速度；正式测试建议从较低速度开始。                     |
| `focalRateCommandInterfaceValidated` | Zoom底层接口验证声明。`true/1`表示单位、方向、限速和停止均已验证；`false/0`表示禁止真实Zoom命令。 |
| `zoomCalibrationValid`               | Zoom标定是否有效。自动等于 `focalRateCommandInterfaceValidated`。         |
| `zoomCalibrationIsPlaceholder`       | Zoom标定是否为占位状态。自动生成；`true/1`表示未验证。                             |
| `zoomRateLimitIsPlaceholder`         | Zoom速度限制是否仍未验证。自动生成；`true/1`表示不能用于真机Zoom。                     |

### 7. 视觉测量与逆深度范围

| 参数名称                          | 含义、作用与推荐值                                                       |
| ----------------------------- | --------------------------------------------------------------- |
| `numericalEpsilon`            | 通用数值保护小量，用于避免除零。推荐保持 `1e-8`。                                    |
| `visibilityEpsilon`           | 可见性判断数值保护小量。推荐保持 `1e-6`。                                        |
| `visibilityZMin`              | 目标允许的最小正深度，单位m。当前 `0.10 m`；用于排除相机后方或过近目标。                       |
| `targetDepthMin`              | 任务允许的目标最小深度。当前 `0.50 m`；根据实验工作距离调整。                             |
| `targetDepthMax`              | 任务允许的目标最大深度。当前 `1.00 m`；根据实验工作距离调整。                             |
| `Zd`                          | 期望目标深度，单位m。当前 `0.75 m`；应位于 `targetDepthMin`和`targetDepthMax`之间。 |
| `rhoD`                        | 期望逆深度，自动为 `1/Zd`。当前约 `1.333 m⁻¹`。                               |
| `rhoEstimateMin`              | EKF允许的最小逆深度。当前 `0.80 m⁻¹`，对应最大深度约 `1.25 m`。                     |
| `rhoEstimateMax`              | EKF允许的最大逆深度。当前 `2.20 m⁻¹`，对应最小深度约 `0.455 m`。                    |
| `rhoMin`                      | `rhoEstimateMin`的兼容别名。自动生成。                                     |
| `rhoMax`                      | `rhoEstimateMax`的兼容别名。自动生成。                                     |
| `disparityMin`                | 双目视差的最小保护值，防止除零和异常大深度。推荐保持 `1e-4`。                              |
| `rightVisibilityMarginPx`     | 右相机目标距离图像边缘的安全余量。当前 `80 px`；分辨率变化后按比例调整。                        |
| `rightVisibilityHysteresisPx` | 右相机可见性判断滞回宽度。当前 `20 px`，用于避免状态频繁切换。                             |
| `rightReacquireValidSamples`  | 右目标连续有效多少帧后确认重新捕获。当前 `5`帧。                                      |
| `targetCharacteristicSize`    | AprilTag目标的实际特征尺寸，单位m。当前 `0.10 m`；必须填写真实标签尺寸。                   |
| `scaleDesired`                | 左右相机期望Tag尺度，定义为四角面积平方根，单位pixel。当前 `[700;700]`；应根据期望距离和焦距实测。     |

### 8. Arm视觉控制器

| 参数名称                 | 含义、作用与推荐值                                                                |
| -------------------- | ------------------------------------------------------------------------ |
| `centerDesired`      | 左相机期望归一化图像中心。推荐 `[0;0]`，表示目标位于光轴中心。                                      |
| `Kc`                 | 图像中心误差反馈增益矩阵。当前 `diag([2.5,2.5])`；初次真机建议从较低增益开始。                         |
| `kRho`               | 逆深度误差反馈增益。当前 `1.5`；仅在Depth任务启用时生效。                                       |
| `lambdaC`            | 中心任务阻尼伪逆系数。当前 `0.02`；接近奇异位形时增大可提高稳定性但降低响应。                               |
| `lambdaRho`          | 逆深度任务阻尼系数。当前 `0.02`。                                                     |
| `betaC`              | 中心任务鲁棒补偿强度。`0`表示关闭；大于0表示启用。当前推荐保持 `0`，验证基础控制后再调。                         |
| `betaRho`            | 逆深度任务鲁棒补偿强度。`0`表示关闭；大于0表示启用。当前推荐保持 `0`。                                  |
| `epsilonC`           | 中心鲁棒项的平滑小量。当前 `1e-3`；仅 `betaC>0`时明显生效。                                   |
| `epsilonRho`         | 逆深度鲁棒项的平滑小量。当前 `1e-3`；仅 `betaRho>0`时明显生效。                                |
| `nullspaceEnable`    | 关节中位零空间任务开关。`true/1`开启关节回中；`false/0`关闭。检查纯视觉方向时建议关闭，正常运行可开启。             |
| `kNull`              | 零空间关节回中增益。当前 `0.05`；过大会干扰低优先级运动，建议从 `0.01～0.05`开始。                       |
| `armControlEnable`   | 机械臂控制总开关。`true/1`允许生成关节速度；`false/0`强制最终关节速度归零。无机械臂测试可保持开启但由标定锁归零，也可直接关闭。 |
| `depthTaskEnable`    | 双目逆深度任务开关。`true/1`启用Depth次任务；`false/0`仅执行中心任务。单相机测试必须设为 `false`。         |
| `qDotAppliedInitial` | 反馈环中已施加关节速度的Unit Delay初值。固定推荐7维全零。                                       |
| `depthErrorInitial`  | 深度误差反馈环的初值。固定推荐 `0`。                                                     |

### 9. 目标EKF

| 参数名称                    | 含义、作用与推荐值                                              |
| ----------------------- | ------------------------------------------------------ |
| `pixelNoiseStd`         | AprilTag像素测量噪声标准差，单位pixel。当前 `0.5 px`；应根据静止目标数据估计。     |
| `Rpixel`                | 视觉测量像素协方差，自动为 `pixelNoiseStd² × I₄`。不建议单独修改。           |
| `sigmaAcceleration`     | EKF目标加速度过程噪声标准差。当前 `0.5`；增大后跟踪更快但估计更抖。                 |
| `sigmaJerk`             | EKF目标加加速度过程噪声标准差。当前 `1.0`；增大后对快速运动响应更快。                |
| `ekfCovarianceJitter`   | 协方差矩阵正定性保护小量。推荐保持 `1e-12`。                             |
| `ekfSConditionMin`      | EKF创新协方差允许的最小条件阈值。推荐保持 `1e-12`。                        |
| `ekfInitializationMode` | EKF初始化模式。当前 `1`表示使用第一帧有效双目测量初始化；推荐保持 `1`。              |
| `pWCL0`                 | 初始左相机世界坐标位置。由机器人模型、`q0`和手眼关系自动计算。                      |
| `RWCL0`                 | 初始左相机世界坐标旋转矩阵。自动计算。                                    |
| `target0CL`             | 首次测量前目标在左相机系中的后备位置。当前 `[0;0;Zd]`。                      |
| `target0`               | 首次测量前目标在世界系中的后备位置。自动计算。                                |
| `ekfX0`                 | EKF九维初始状态 `[位置;速度;加速度]`。自动根据 `target0`构造，初始速度和加速度推荐为0。 |
| `ekfP0`                 | EKF初始协方差矩阵。当前值适合作为初始测试；应根据初始位置、速度和加速度不确定度调整。           |

### 10. Zoom控制器

| 参数名称                | 含义、作用与推荐值                                                            |
| ------------------- | -------------------------------------------------------------------- |
| `Kf`                | 左右Zoom尺度误差反馈增益。当前 `diag([1.5,1.5])`；硬件验证后从较低增益开始。                    |
| `betaF`             | Zoom鲁棒补偿强度。`[0;0]`表示关闭；非零表示开启对应镜头鲁棒补偿。当前推荐保持0。                       |
| `epsilonF`          | Zoom鲁棒项平滑小量。当前 `[1e-3;1e-3]`。                                        |
| `zoomControlEnable` | Zoom控制总开关。`true/1`允许生成焦距速度；`false/0`强制Zoom命令归零。没有Zoom硬件时必须为 `false`。 |

### 11. Zoom优先级调度

| 参数名称                         | 含义、作用与推荐值                                    |
| ---------------------------- | -------------------------------------------- |
| `scaleErrorEnterThreshold`   | 尺度误差超过该值时进入Zoom优先模式。当前 `0.04`。               |
| `scaleErrorExitThreshold`    | 尺度误差低于该值时退出Zoom优先模式。当前 `0.015`；应小于进入阈值以形成滞回。 |
| `scaleSettledHoldTime`       | 尺度误差稳定后需要持续满足的时间。当前 `0.25 s`。                |
| `disturbanceConfirmTime`     | 扰动持续多久后才确认发生。当前 `0.10 s`，用于过滤短时噪声。           |
| `zoomOnlyMaxTime`            | 只允许Zoom动作的最长时间。当前 `1.00 s`，防止机械臂任务长期被阻塞。     |
| `armDepthRampTime`           | Arm深度任务恢复时的渐入时间。当前 `0.50 s`。                 |
| `zoomLimitMarginFraction`    | Zoom工作范围边界保护比例。当前 `0.02`，即预留约2%范围。           |
| `depthErrorLoggingThreshold` | 深度误差超过该值时记录或判定具有明显响应。当前 `0.02`。              |
| `zoomResponseThreshold`      | 判断Zoom产生有效响应的最小命令阈值。当前 `1e-3`。               |
| `armDepthResponseThreshold`  | 判断Arm深度任务产生有效响应的最小阈值。当前 `1e-4`。              |

### 12. 安全限制

| 参数名称                      | 含义、作用与推荐值                                                               |
| ------------------------- | ----------------------------------------------------------------------- |
| `qLimitSoftMargin`        | 关节位置软限位距离。当前 `5°`；推荐保留，不应设为0。                                           |
| `cartesianLinearSpeedMax` | 相机/末端笛卡尔线速度上限，单位 `m/s`。当前 `2.0`主要作为硬保护；实际速度还会受到更严格的关节速度限制。真机初期应采用更保守限制。 |
| `qDotAlgorithmMax`        | Core算法层关节速度上限。当前每个关节不超过 `0.03 rad/s`，并受URDF硬件上限约束；推荐初次真机保持该值或更低。        |
| `qDDotAlgorithmMax`       | Core算法层关节加速度上限。当前每个关节 `0.20 rad/s²`；推荐低速测试保持该值。                         |

### 13. ROS输入监督与消息长度

| 参数名称                            | 含义、作用与推荐值                                                                                 |
| ------------------------------- | ----------------------------------------------------------------------------------------- |
| `jointStateTimeoutSec`          | 关节状态超时时间。当前 `0.10 s`；120 Hz或60 Hz输入下均较宽松。                                                 |
| `visionTimeoutSec`              | 左视觉输入超时时间。当前 `0.10 s`；当前约52 Hz视觉输入可以满足。                                                   |
| `visionTimeoutFrames`           | 视觉超时对应的Core周期数。自动由 `visionTimeoutSec/Ts`计算，当前为6帧。                                         |
| `targetLossFrameLimit`          | 连续多少帧目标无效后确认目标丢失。当前 `3`帧。                                                                 |
| `targetRecoveryFrameCount`      | 连续多少帧目标有效后确认恢复。当前 `3`帧。                                                                   |
| `visionMessageLength`           | 内部视觉特征向量长度。固定为 `8`。                                                                       |
| `jointPositionMessageLength`    | 关节位置向量长度。固定为 `7`。                                                                         |
| `jointVelocityMessageLength`    | 关节速度向量长度。固定为 `7`。                                                                         |
| `controllerStatusMessageLength` | 控制器状态向量长度。固定为 `13`。                                                                       |
| `jointStateTimeoutFrames`       | 关节状态超时对应的Core周期数。自动由 `jointStateTimeoutSec/Ts`计算。                                         |
| `controllerStatusOrder`         | 13维状态的固定顺序：输入有效、相机模型有效、焦距新鲜、运动学有效、左右目标有效、双目有效、EKF状态、深度权重、调度模式、安全状态和控制许可。不要改变顺序，除非同步修改订阅端。 |
| `visionFeatureOrder`            | 8维内部视觉向量顺序：`[validL;validR;uL;vL;uR;vR;scaleL;scaleR]`。不要改变顺序。                            |

### 14. 标定许可与自动安全状态

| 参数名称                           | 含义、作用与推荐值                                                            |
| ------------------------------ | -------------------------------------------------------------------- |
| `cameraModelCalibrationReady`  | 相机模型许可，自动由内参标定和像元尺寸标定共同决定。`true/1`表示两项都完成；`false/0`表示相机模型不可用于真机控制。   |
| `armControlCalibrationReady`   | Arm控制标定许可，自动要求手眼标定和相机模型均有效。`true/1`允许通过Arm标定锁；`false/0`禁止非零Arm输出。    |
| `depthControlCalibrationReady` | Depth控制标定许可，自动要求Arm许可和双目标定均有效。                                       |
| `zoomControlCalibrationReady`  | Zoom控制标定许可，自动要求Depth许可和Zoom底层接口验证均有效。                                |
| `fullDeploymentReady`          | 当前启用功能是否全部满足对应标定要求。`true/1`表示当前配置具备完整部署条件；`false/0`表示至少一项启用功能尚未完成标定。 |
| `stage1CalibrationReady`       | `armControlCalibrationReady`的兼容别名。自动生成。                              |
| `controllerCalibrationReady`   | `fullDeploymentReady`的兼容别名。自动生成。                                     |
