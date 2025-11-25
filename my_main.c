#include "my_main.h"

GPIO_PinState AIN1 = GPIO_PIN_RESET, AIN2 = GPIO_PIN_RESET; // 控制电机正反转
GPIO_PinState BIN1 = GPIO_PIN_RESET, BIN2 = GPIO_PIN_RESET;

/*超声波参数*/
float distances       = 30;
uint32_t tt           = 0;
uint32_t high_time[2] = {0};
uint8_t c_values      = 0;

/* 按键结构体*/
struct key {
    /* data */
    uint8_t key_std;
    uint8_t key_flag;
    uint8_t key_bit;
    uint16_t key_time;
    uint8_t key_long_bit;
};
struct key key_five[5] = {0};
uint8_t i              = 0;

// gray********
extern uint8_t xun[8];
extern int8_t arr_bit[8];
int8_t err_gray = 0;

// led********
__IO uint32_t uwTick_led;

// oled
int8_t buf[20];
uint8_t Data[] = " ";

// 控制********
int Vertical_out, Velocity_out, Turn_out; // 直立环&速度环&转向环的输出变量
int Encoder_Left, Encoder_Right, Balance_PWM;
int Moto1, Moto2;
int PWM_out, Turn_Pwm = 0; // 闭环输出
uint8_t Moto_Flag  = 0;    // 电机控制标志
uint8_t Start_Flag = 0;
uint8_t Dis_Flag   = 0;

//----速度环-----//
float Velocity, Encoder_Err, Encoder, Encoder_last; // 速度，误差，编码器
float Encoder_Integral;                             // 编码器数值积分
#define kp_val -10
float kp = kp_val, ki = kp_val / 200;
int AimVelocity   = 70;
uint8_t SpeedMode = 0;

//----循迹-----
int Gray_KP   = -2;
float Gray_KD = 0.05;
int last_bias;

// buzzer
uint8_t buzzer_flag = 0;
__IO uint32_t uwTick_buzzer;

void delay_us(uint32_t us)
{
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 8000000 * us);
    while (delay--);
}

void setup(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 电机
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // 编码器
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

    HAL_TIM_Base_Start_IT(&htim6); // 定时10ms
    HAL_TIM_Base_Start_IT(&htim9);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim10);

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "begin", 16);
    OLED_Refresh();

    if (Start_Flag == 0) {
        Moto1 = 0;
        Moto2 = 0;
        Set_Pwm(Moto1, Moto2);
    }
    // 蜂鸣器
    if (buzzer_flag == 0) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
    }
}

void loop(void)
{
    led_show();
    // OLED_ALL_NUM(0, 48, yaw, 3, 2, 16);
    // OLED_INT_NUM(0, 0, left_front, 16);
    // OLED_INT_NUM(48, 0, right_front, 16);
    // OLED_INT_NUM(0, 16, left_back, 16);
    // OLED_INT_NUM(48, 16, right_back, 16);
    // OLED_Showdecimal(0, 32, angle_ki, 3, 5, 16);
    key_func();
    oled_show();
    if (buzzer_flag == 1) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
        buzzer_flag = 0;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6) {
        // OLED_Refresh();
        key_process();
    }
    if (htim == &htim10) {
        // OLED_Clear();
        if (Start_Flag == 1 && Dis_Flag == 1) {
            /* code */
            get_encoder(&Encoder_Left, &Encoder_Right);
            Velocity_out = GetVelocity(Encoder_Left, Encoder_Right); // 速度环输出误差
            err_gray     = Read_8PIN();
            Moto1        = Velocity_out + Gray_PID(err_gray);
            Moto2        = Velocity_out - Gray_PID(err_gray);
            //				Moto1 = Theturn;
            //				Moto2 = -Theturn;
            Limit(&Moto1, &Moto2); // PWM限幅
            Set_Pwm(Moto1, Moto2); // 加载到电机上
            ReadBlag();
        } else if (Start_Flag == 0 || Dis_Flag == 0) {
            Moto1 = 0;
            Moto2 = 0;
            Set_Pwm(Moto1, Moto2); // 吴浩鑫
        }
    }

    if (htim == &htim7) {
        // 判断前方障碍物
        if (distances <= 20) {
            Dis_Flag = 0;
            if (buzzer_flag == 0) {
                buzzer_flag = 1;
            }
        } else {
            Dis_Flag = 1;
        }
        // 发送trig脚信号并开启捕获
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
        delay_us(20);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
        //__HAL_TIM_SET_CAPTUREPOLARITY(&htim9,TIM_CHANNEL_1,TIM_ICPOLARITY_RISING);
        HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_1);
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim9) {
        switch (c_values) {
                // 上升沿捕获
            case (0):
                high_time[0] = HAL_TIM_ReadCapturedValue(&htim9, TIM_CHANNEL_1);
                //__HAL_TIM_SET_CAPTUREPOLARITY(&htim9,TIM_CHANNEL_1,TIM_ICPOLARITY_FALLING);
                c_values++;
                break;

            // 下降沿捕获并计算时间及距离
            case (1):
                high_time[1] = HAL_TIM_ReadCapturedValue(&htim9, TIM_CHANNEL_1);
                HAL_TIM_IC_Stop_IT(&htim9, TIM_CHANNEL_1);
                c_values = 0;
                __HAL_TIM_SET_COUNTER(&htim9, 0);
                tt        = high_time[1] - high_time[0];
                distances = tt * 0.017;
                break;

            default:
                break;
        }
    }
}
void key_process(void)
{
    // 读取GPIOD端口的五个引脚状态并存储在key_five数组中
    key_five[0].key_flag = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0);
    key_five[1].key_flag = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1);
    key_five[2].key_flag = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2);
    key_five[3].key_flag = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3);
    key_five[4].key_flag = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4);
    // 遍历key_five数组中的每个按键状态
    for (i = 0; i < 5; i++) {
        /* code */
        // 根据按键的标准状态进行处理
        switch (key_five[i].key_std) {
            case 0: { // 按键未按下状态
                if (key_five[i].key_flag == 0) {
                    key_five[i].key_std  = 1; // 检测到按键按下，进入确认状态
                    key_five[i].key_time = 0; // 重置按键计时器
                } else {
                    key_five[i].key_std = 0; // 保持未按下状态
                }
            } break;
            case 1: { // 确认按键按下状态
                if (key_five[i].key_flag == 0) {
                    key_five[i].key_std = 2; // 确认按键持续按下，进入保持状态
                    //						key_five[i].key_bit = 1;
                    //						key_five[i].key_time = 0;
                } else {
                    key_five[i].key_std = 0; // 检测到按键松开，回到未按下状态
                }
            } break;
            case 2: { // 按键保持按下状态
                if (key_five[i].key_flag == 1 && key_five[i].key_time >= 200) {
                    key_five[i].key_std      = 0; // 按键松开，回到未按下状态
                    key_five[i].key_long_bit = 1; // 设置长按标志
                } else if (key_five[i].key_flag == 0) {
                    key_five[i].key_time++; // 按键持续按下，增加计时器
                } else if (key_five[i].key_flag == 1 && key_five[i].key_time < 200) {
                    key_five[i].key_std = 0; // 按键松开，回到未按下状态
                    key_five[i].key_bit = 1; // 设置短按标志
                }
            } break;
        }
    }
}

void key_func(void)
{
    // 检查并重置 key_five 数组中第一个元素的 key_bit 位，如果其值为1
    if (key_five[0].key_bit == 1) {
        key_five[0].key_bit = 0;
        Start_Flag          = 0;
        buzzer_flag         = 0;
        Moto1               = 0;
        Moto2               = 0;
        Set_Pwm(Moto1, Moto2);

        // 检查并重置 key_five 数组中第二个元素的 key_bit 位，如果其值为1
    } else if (key_five[1].key_bit == 1) {
        key_five[1].key_bit = 0;
        Start_Flag          = 1;

        // 检查并重置 key_five 数组中第三个元素的 key_bit 位，如果其值为1
    } else if (key_five[2].key_bit == 1) {
        key_five[2].key_bit = 0;
        Gray_KP -= 2;

        // 检查并重置 key_five 数组中第四个元素的 key_bit 位，如果其值为1
    } else if (key_five[3].key_bit == 1) {
        key_five[3].key_bit = 0;
        Gray_KP += 2;
        // 检查并重置 key_five 数组中第五个元素的 key_bit 位，如果其值为1
    } else if (key_five[4].key_bit == 1) {
        key_five[4].key_bit = 0;
    }
}

/***************************************************************************
函数功能：控制电机
******************************************************************************/
void Contrl(void)
{

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, AIN1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, AIN2);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, BIN1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, BIN2);
}

/**************************************************************************
函数功能：电机转动控制函数
入口参数：闭环控制最终输出值
**************************************************************************/
void Set_Pwm(int Moto1, int Moto2)
{

    if (Moto1 > 0) {
        AIN1 = GPIO_PIN_RESET;
        AIN2 = GPIO_PIN_SET;
    } else {
        AIN1 = GPIO_PIN_SET;
        AIN2 = GPIO_PIN_RESET;
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, my_abs(Moto1));

    if (Moto2 > 0) {
        BIN1 = GPIO_PIN_RESET;
        BIN2 = GPIO_PIN_SET;
    } else {
        BIN1 = GPIO_PIN_SET;
        BIN2 = GPIO_PIN_RESET;
    }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, my_abs(Moto2)); // 0-5999
    Contrl();
}

/**************************************************************************
函数功能：限制电机速度
入口参数：闭环控制最终输出值
**************************************************************************/
void Limit(int *motoA, int *motoB)
{
    if (*motoA > 4500)
        *motoA = 4500; // 最大4500
    if (*motoA < -4500)
        *motoA = -4500;

    if (*motoB > 4500)
        *motoB = 4500;
    if (*motoB < -4500)
        *motoB = -4500;
}

/**************************************************************************
函数功能：获取编码器的值
入口参数：2个指向整数的指针
**************************************************************************/
void get_encoder(int *left, int *right)
{
    *left = -1 * (short)__HAL_TIM_GET_COUNTER(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1, 0);

    *right = (short)__HAL_TIM_GET_COUNTER(&htim8);
    __HAL_TIM_SET_COUNTER(&htim8, 0);
}

// abs函数
int my_abs(int num)
{
    if (num < 0) num = -num;
    return num;
}

float float_abs(float num)
{
    if (num < 0) num = -num;
    return num;
}

void oled_show(void)
{

    OLED_ShowString(0, 0, "Gray_PID:", 16);
    sprintf(buf, "%d", Gray_KP);
    OLED_ShowString(0, 16, buf, 16);
    OLED_ShowString(0, 32, "distance", 16);
    sprintf(Data, "%.3f", distances);
    OLED_ShowString(0, 48, Data, 16);
    OLED_Refresh();
}

void led_show(void)
{
    uwTick = HAL_GetTick();
    if ((uwTick - uwTick_led) > 500) {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
        uwTick_led = uwTick;
    }
}

/*********************************************************************
速度环PI控制器：Kp*Ek+Ki*Ek_S(Ek_S：偏差的积分)
入口：左右编码器测到的数值
出口：
**********************************************************************/
int GetVelocity(int Encoder_left, int Encoder_right)
{

    // 1.计算速度偏差 	//
    Encoder_Err = (AimVelocity - (Encoder_Left + Encoder_Right)); // 速度目标值//Encoder_Left+Encoder_Right 最大是 160  7200 /（160 * 50%） = 90

    // 2.对速度偏差进行--低通滤波--
    // low_out = (1-a)*Ek+a*low_out_last
    Encoder      = Encoder_Err * 0.3f + Encoder_last * 0.7f; // 使得波形更加平滑
    Encoder_last = Encoder;                                  // 防止速度过大影响直立环的正常工作

    // 3.对速度偏差积分出位移,遥控的速度通过积分融入速度控制器，减缓速度突变对直立控制的影响
    Encoder_Integral += Encoder;

    // 4.积分限幅
    if (Encoder_Integral > 1000)
        Encoder_Integral = 1000;
    if (Encoder_Integral < -1000)
        Encoder_Integral = -1000;

    if (Moto_Flag == 1 || Start_Flag == 0)
        Encoder_Integral = 0; //===电机关闭后或者复位清除积分
                              // 5.速度环控制输出
    Velocity = Encoder * kp + Encoder_Integral * ki;

    return Velocity;
}

/***************************************************************************
PD控制器：Kp*Ek+Kd*Ek_D
入口：
出口：PMW数值
******************************************************************************/
float Gray_PID(int bias)
{

    float res = Gray_KP * bias + Gray_KD * (bias - last_bias);
    last_bias = bias;
    return res;
}

void ReadBlag(void)
{
    if (xun[0] == 0 && xun[1] == 0 && xun[2] == 0 && xun[3] == 0 && xun[4] == 0 && xun[5] == 0 && xun[6] == 0 && xun[7] == 0) {
        Start_Flag = 0;
        Moto1      = 0;
        Moto2      = 0;
        Set_Pwm(Moto1, Moto2);
        buzzer_flag = 1;
    }
}
