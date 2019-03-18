/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/
#define IS_BUT1_PUSH (GPIOC->IDR & GPIO_IDR_IDR14) //������ 1 PC14
#define IS_BUT2_PUSH (GPIOB->IDR & GPIO_IDR_IDR10) //������ 2 PB10
#define IS_BUT3_PUSH (!(GPIOB->IDR & GPIO_IDR_IDR14)) //������ 3 PB14
#define IS_BUT4_PUSH (!(GPIOB->IDR & GPIO_IDR_IDR7)) //������ 4 PB7

//��������� ���������� � ��������� "������" - �����.0 (reset)
//� � ��������� "�� ������" - �����.1 (set)
#define SET_LED_ON (GPIOC->BSRR=GPIO_BSRR_BR13)
#define SET_LED_OFF (GPIOC->BSRR=GPIO_BSRR_BS13)

//�������� ��������� �����
#define IS_LED_OFF (GPIOC->ODR & GPIO_ODR_ODR13)

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

int main(void)
{
		//�������� ������������ ������ � � �
		RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

		GPIOC->CRH &= ~(GPIO_CRH_CNF13_1 | GPIO_CRH_CNF13_0| GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0 |
						GPIO_CRH_MODE14_1 | GPIO_CRH_MODE14_0 | GPIO_CRH_CNF14_0 | GPIO_CRH_CNF14_1);
		GPIOB->CRH &= ~(GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1 | GPIO_CRH_CNF10_0 | GPIO_CRH_CNF10_1 |
						GPIO_CRH_MODE14_0 | GPIO_CRH_MODE14_1 | GPIO_CRH_CNF14_0 | GPIO_CRH_CNF14_1);
		GPIOB->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7);

		//����������� ��������� �� �����, ������ �� ����
		GPIOC->CRH |= (GPIO_CRH_MODE13_1 | GPIO_CRH_CNF14_1);
		GPIOB->CRH |= (GPIO_CRH_CNF10_1 | GPIO_CRH_CNF14_1);
		GPIOB->CRL |= GPIO_CRL_CNF7_1;

		//������ 1 - �����.�������� � �����
		GPIOC->ODR &= ~GPIO_ODR_ODR14;
		//������ 2 - �����.�������� � �����
		GPIOB->ODR &= ~GPIO_ODR_ODR10;
		//������ 3 � 4 - �����.�������� � �������
		GPIOB->ODR |= (GPIO_ODR_ODR14 | GPIO_ODR_ODR7);

		// �������� ������������ �������
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
		//�������������� ������
		TIM3->PSC = 36000 - 1;
		TIM3->ARR = DEF_ARR;
		//��������� ������
		TIM3->CR1 |= TIM_CR1_CEN;
		//��������� ����������
		TIM3->DIER |= TIM_DIER_UIE;

		//��������� ��������� ����������
		NVIC_EnableIRQ(TIM3_IRQn);
	for(;;)
	{
		if (IS_BUT1_PUSH)
		{
			TIM3->CR1 &= ~TIM_CR1_CEN;
			SET_LED_OFF;
			TIM3->CNT = 0;
			TIM3->ARR = UINT16_MAX;
			TIM3->CR1 |= TIM_CR1_CEN;
			while(IS_BUT1_PUSH)
			{
				if ((TIM3->CNT) > MIN_TIME)
				delay = TIM3->CNT;
			}
			status = 1;
		}
		if ((!IS_BUT1_PUSH) & status)
		{
			TIM3->CR1 &= ~TIM_CR1_CEN;
			TIM3->CNT = 0;
			TIM3->ARR = 0;
			TIM3->ARR = delay-1;
			TIM3->CR1 |= TIM_CR1_CEN;
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
	TIM3->SR &= ~TIM_SR_UIF; //����� �����
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
