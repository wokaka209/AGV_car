CDIO_AGV 项目说明
这是一个基于 STM32 的自动导引车（AGV）控制项目，实现了小车的直立控制、循迹导航和避障功能。

主要功能模块
1. 电机控制
使用 TIM2 的两个通道控制两个直流电机
支持正反转控制和 PWM 调速
包含电机限幅保护（最大PWM值限制为4500）
c
void Set_Pwm(int Moto1, int Moto2)
2. 编码器反馈
使用 TIM1 和 TIM8 作为编码器接口
实时获取左右轮速度反馈
c
void get_encoder(int *left, int *right)
3. 循迹功能
支持8路灰度传感器输入
PD控制算法实现路径跟踪
c
float Gray_PID(int bias)
4. 速度控制
PI控制器维持设定速度
默认目标速度为70
c
int GetVelocity(int Encoder_left, int Encoder_right)
5. 超声波避障
使用超声波传感器检测前方障碍物
距离小于20cm时停车并触发蜂鸣器报警
6. 用户交互
5个按键控制不同功能：
按键1：停止电机
按键2：启动电机
按键3：减少循迹KP值
按键4：增加循迹KP值
按键5：预留功能
OLED显示屏显示循迹参数和距离信息
LED指示灯和蜂鸣器提供状态反馈
定时器配置
定时器	功能
TIM2	电机PWM输出
TIM1/TIM8	编码器输入
TIM6	按键扫描(10ms)
TIM7	超声波触发(10ms)
TIM9	超声波输入捕获
TIM10	主控制循环(10ms)
启动流程
初始化所有外设
显示"begin"欢迎信息
等待按键启动
开始循迹和避障功能
注意事项
确保硬件连接正确
根据实际环境调整PID参数
定期清洁灰度传感器以保证循迹精度
