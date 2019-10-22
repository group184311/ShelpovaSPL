/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/
#define IS_BUT1_PUSH (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14)) //������ 1 PC14
#define IS_BUT2_PUSH (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)) //������ 2 PB10
#define IS_BUT3_PUSH (!(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14))) //������ 3 PB14
#define IS_BUT4_PUSH (!(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7))) //������ 4 PB7

//��������� ���������� � ��������� "������" - �����.0 (reset)
//� ��������� "�� ������" - �����.1 (set)
#define SET_LED_ON (GPIO_ResetBits(GPIOC, GPIO_Pin_13))
#define SET_LED_OFF (GPIO_SetBits(GPIOC, GPIO_Pin_13))

//�������� ��������� �����
#define IS_LED_OFF (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13))

//����������� �������� ������ 1 = 50��
#define MIN_TIME 100

//����� ���������� ������ ������ � ��
#define BUT2_TIME_MS 1500
#define BUT3_TIME_MS 2500
#define BUT4_TIME_MS 3500

//���������� ������ � �������� ARR � ����������� �� �������� ��
#define DEF_ARR (1000-1) //��������� �������� = 500��
#define BUT2_ARR (BUT2_TIME_MS/500*1000)
#define BUT3_ARR (BUT3_TIME_MS/500*1000)
#define BUT4_ARR (BUT4_TIME_MS/500*1000)

#include "stm32f10x.h"

int status = 0; //��� ������������ ������� ������ 1
unsigned int delay = DEF_ARR; //����������, �������� ����� ����� ������ 1
int light = DEF_ARR; //����������, �������� ����� ����������

TIM_TimeBaseInitTypeDef tim;

int main(void)
{
		//�������� ������������ ������ � � �
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

		//�������� ������������ �������
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

		//�������������� ������

		tim.TIM_ClockDivision = TIM_CKD_DIV1;
		tim.TIM_CounterMode = TIM_CounterMode_Up;
		tim.TIM_Period = DEF_ARR;
		tim.TIM_Prescaler = 36000 - 1; //36000

		TIM_TimeBaseInit(TIM3, &tim);

		//��������� ������
		TIM_Cmd(TIM3, ENABLE);
		//��������� ����������
		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

		//��������� ��������� ����������
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
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM3, DISABLE);
			SET_LED_OFF;
			TIM_SetCounter(TIM3, 0);
			tim.TIM_Period = UINT16_MAX;
			TIM_TimeBaseInit(TIM3, &tim);
			TIM_Cmd(TIM3, ENABLE);
			while(IS_BUT1_PUSH)
			{
				if (TIM_GetCounter(TIM3) > MIN_TIME)
				delay = TIM_GetCounter(TIM3);
			}
			status = 1;
			TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
			SET_LED_ON;	
		}
		if ((!IS_BUT1_PUSH) & status)
		{
			TIM_Cmd(TIM3, DISABLE);
			TIM_SetCounter(TIM3, 0);
			tim.TIM_Period = delay-1;
			TIM_TimeBaseInit(TIM3, &tim);
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
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
		if (IS_LED_OFF) {
			SET_LED_ON;
			tim.TIM_Period = light - 1;
			TIM_TimeBaseInit(TIM3, &tim);
			TIM_ClearFlag(TIM3, TIM_IT_Update); //����� �����
			return;
		} else {
			SET_LED_OFF;
			tim.TIM_Period = delay - 1;
			TIM_TimeBaseInit(TIM3, &tim);
			TIM_ClearFlag(TIM3, TIM_IT_Update); //����� �����
			return;
		}
	}
}
