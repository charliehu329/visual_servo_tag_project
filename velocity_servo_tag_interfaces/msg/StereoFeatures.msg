# 左右相机各自处理完成的帧序号。
uint32 left_sequence
uint32 right_sequence

# 当前左右特征所对应图像的近似采集时刻。
builtin_interfaces/Time left_capture_stamp
builtin_interfaces/Time right_capture_stamp

# 当前左右帧是否检测到目标Tag。
bool valid_left
bool valid_right

# AprilTag中心像素坐标。
float64 u_left
float64 v_left
float64 u_right
float64 v_right

# 由Tag四边形面积平方根得到的尺度特征。
float64 scale_left
float64 scale_right
