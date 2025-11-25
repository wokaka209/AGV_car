#ifndef __MY_MAIN__
#define __MY_MAIN__

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
// #include "oled.h"
#include "dma.h"

#include "stdio.h"
#include "string.h"
#include "math.h"
#include "oled.h"
#include "gray.h"


void setup(void);
void loop(void);
void key_process(void);
void key_func(void);
void Limit(int *motoA, int *motoB);
void Set_Pwm(int Moto1, int Moto2);
void Contrl(void);
int my_abs(int num);
float float_abs(float num);
void led_show(void);
void oled_show(void);
int GetVelocity(int Encoder_left, int Encoder_right);
float Gray_PID(int bias);
void get_encoder(int *left, int *right);
void ReadBlag(void);

#endif
