# 前三个 3-dof 是目标值
# 第四个是 Mid360-Fastlio
# 第五六个是 目标的球的值 如果没有吸上就全是0
# 第七个是是否吸到球的目标


float32 cmd_vx
float32 cmd_vy
float32 desire_yaw
float32 measure_yaw
float32 ball_x
float32 ball_y
# 机械臂状态对应，
# 0对应抬起
# 1对应找
# 3对应放球
int32 arm
int32 pump
# 0对应速度模式，1对应位置模式
int32 mode