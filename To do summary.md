
# 7.23 To do 
1. 真机实验stage1，
    1. 先测试各层通讯，用vision_double.launch只测试双目发送topic /vision_double/target_features
    2. 设置为zero模式，运行full.launch，运行simulink，记得打开config的false开关，测试simulink接受vision feature后能否输出关节topic /simulink/target_joints_velocities

# 7.24 调试记录
1. 相机在
    detector_threads_per_camera: 6
    quad_decimate: 1.0
    quad_sigma: 0.0
    refine_edges: true
    decode_sharpening: 0.25
    当前参数下可以稳定在55hz
2. 当前设置    
    检查左右相机新帧序号并刷新预览窗口的频率
    因为现在是左右相机任意一帧更新就会发布，我们把发布定时器发送频率提高可以让simulink尽快接收最新信息
    且simulink会自己去进行sequence配对，来匹配哪两帧是同一时刻的。
    publish_rate_hz: 120.0
3. 在simulink的配置文件里
    cfg.stereoMaxPairSkewSec = 0.05;，表示双目相机相差时间在50ms时，便不进行EKF和深度测量
4. 深度测量和逆深度估计
   1. 深度测量就是用双目的视差来计算深度，用于EKF测量输入
   2. 逆深度估计主要是EKF状态估计，根据历史信息，推算下一刻可能会到什么位置，用于深度控制的输入
