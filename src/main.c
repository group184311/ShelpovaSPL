/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/
#define IS_BUT1_PUSH (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14)) //кнопка 1 PC14
#define IS_BUT2_PUSH (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)) //кнопка 2 PB10
#define IS_BUT3_PUSH (!(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14))) //кнопка 3 PB14
#define IS_BUT4_PUSH (!(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7))) //кнопка 4 PB7

//установка светодиода в состояние "гореть" - логич.0 (reset)
//в состояние "не гореть" - логич.1 (set)
#define SET_LED_ON (GPIO_ResetBits(GPIOC, GPIO_Pin_13))
#define SET_LED_OFF (GPIO_SetBits(GPIOC, GPIO_Pin_13))

//проверка состояния диода
#define IS_LED_OFF (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13))

//минимальная задержка кнопки 1 = 50мс
#define MIN_TIME 100

//время коммутации каждой кнопки в мс
#define BUT2_TIME_MS 1500
#define BUT3_TIME_MS 2500
#define BUT4_TIME_MS 3500

//количество тактов в регистре ARR в зависимости от заданных мс
#define DEF_ARR (1000-1) //начальное значение = 500мс
#define BUT2_ARR (BUT2_TIME_MS/500*1000)
#define BUT3_ARR (BUT3_TIME_MS/500*1000)
#define BUT4_ARR (BUT4_TIME_MS/500*1000)

#include "stm32f10x.h"

int status = 0; //для отслеживания нажатия кнопки 1
unsigned int delay = DEF_ARR; //переменная, хранящая время паузы кнопки 1
int light = DEF_ARR; //переменная, хранящая время коммутации

int main(void)
{
		//включаем тактирование портов С и В
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

		GPIO_InitTypeDef gpio_init;
		gpio_init.GPIO_Pin = GPIO_Pin_13;
		gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
		gpio_init.GPIO_Speed = GPIO_Speed_2MHz;

		GPIO_Init(GPIOC, &gpio_init);

		gpio_init.GPIO_Pin = GPIO_Pin_14;
		gpio_init.GPIO_Mode = GPIO_Mode_IPD;

		GPIO_Init(GPIOC, &gpio_init);

		gpio_init.GPIO_Pin = GPIO_Pin_10;
		gpio_init.GPIO_Mode = GPIO_Mode_IPD;
		gpio_init.GPIO_Speed = GPIO_Speed_2MHz;

		GPIO_Init(GPIOB, &gpio_init);

		gpio_init.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_7;
		gpio_init.GPIO_Mode = GPIO_Mode_IPU;

		GPIO_Init(GPIOB, &gpio_init);

		//Включаем тактирование таймера
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

		//инициализируем таймер
		TIM_TimeBaseInitTypeDef tim;
		tim.TIM_ClockDivision = TIM_CKD_DIV1;
		tim.TIM_CounterMode = TIM_CounterMode_Up;
		tim.TIM_Period = DEF_ARR;
		tim.TIM_Prescaler = 36000 - 1;

		TIM_TimeBaseInit(TIM3, &tim);

		//запускаем таймер
		TIM_Cmd(TIM3, ENABLE);
		//разрешаем прерывание
		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

		//разрешаем обработку прерывания
		NVIC_InitTypeDef nvicInit;
		nvicInit.NVIC_IRQChannel = TIM3_IRQn;
		nvicInit.NVIC_IRQChannelCmd = ENABLE;
		nvicInit.NVIC_IRQChannelPreemptionPriority = 0;
		nvicInit.NVIC_IRQChannelSubPriority = 0;
		NVIC_Init(&nvicInit);

	for(;;)
	{
		if (IS_BUT1_PUSH)
		{
			TIM_Cmd(TIM3, DISABLE);
			SET_LED_OFF;
			TIM3->CNT = 0;
			tim.TIM_Period = UINT16_MAX;
			TIM_Cmd(TIM3, ENABLE);
			while(IS_BUT1_PUSH)
			{
				if ((TIM3->CNT) > MIN_TIME)
				delay = TIM3->CNT;
			}
			status = 1;
		}
		if ((!IS_BUT1_PUSH) & status)
		{
			TIM_Cmd(TIM3, DISABLE);
			TIM3->CNT = 0;
			tim.TIM_Period = 0;
			tim.TIM_Period = delay-1;
			TIM_Cmd(TIM3, ENABLE);
			status = 0;
		}
		if (IS_BUT2_PUSH)
		{
			light = BUT2_ARR;
		}
		else if (IS_BUT3_PUSH)
		{
			light = BUT3_ARR;
		}
		else if (IS_BUT4_PUSH)
		{
			light = BUT4_ARR;
		}
		else
		{
			light = DEF_ARR;
		}
	}
}
void TIM3_IRQHandler(void) {
	TIM_ClearFlag(TIM3, TIM_IT_Update); //сброс флага
	if (IS_LED_OFF)
	{
		SET_LED_ON;
		TIM3->ARR = light-1;

		return;
	}
	else
	{
		SET_LED_OFF;
		TIM3->ARR = delay-1;

		return;
	}
}
