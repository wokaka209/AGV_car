#ifndef __OLED_H
#define __OLED_H

#include "main.h"
#include "stm32f4xx_hal.h"
//**********************宏定义
#define u8 unsigned char
#define u32 unsigned int
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

//**********************IO口的定义

#define OLED_SCL_low() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)//SCL
#define OLED_SCL_hig() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)

#define OLED_SDA_low() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)//SDA
#define OLED_SDA_hig() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)


void delay(unsigned int n);
void OLED_WR_Byte(unsigned char dat,unsigned char mode);
void OLED_ColorTurn(unsigned char i);
void I2C_start(void);
void I2C_stop(void);
void I2C_WaitAck(void);
void Send_Byte(u8 dat);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y);
void OLED_ClearPoint(u8 x,u8 y);
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2);
void OLED_DrawCircle(u8 x,u8 y,u8 r);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1);
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1);
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1);
void OLED_ScrollDisplay(u8 num,u8 space);
void OLED_WR_BP(u8 x,u8 y);
void OLED_ShowPicture(u8 x0,u8 y0,u8 x1,u8 y1,u8 BMP[]);
void OLED_Showdecimal(u8 x,u8 y,float num,u8 z_len,u8 f_len,u8 size2);
void OLED_ALL_NUM(u8 x,u8 y,float num,u8 z_len,u8 f_len,u8 size2);
void OLED_INT_NUM(u8 x, u8 y, int num, u8 size);
void OLED_Init(void);
void OLED_DisplayTurn(u8 i);                                                                                                                                                                                                                   


#endif




