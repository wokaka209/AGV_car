#include "gray.h"


uint8_t xun[8];
//int8_t arr_bit[8]={70,55,30,10,-10,-30,-55,-70};
int8_t arr_bit[8]={50,40,20,10,-10,-20,-40,-50};

int8_t Read_8PIN(void)
{
	xun[0] = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9);
	xun[1] = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15);	
	xun[2] = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11);
	xun[3] = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4);		
	xun[4] = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8);
	xun[5] = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14);		
	xun[6] = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
	xun[7] = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4);		
	return ERR();
}

int8_t ERR(void)
{
	int8_t Err = arr_bit[0]*xun[0]+arr_bit[1]*xun[1]+arr_bit[2]*xun[2]+arr_bit[3]*xun[3]+arr_bit[4]*xun[4]+arr_bit[5]*xun[5]+arr_bit[6]*xun[6]+arr_bit[7]*xun[7];
	return Err;
}