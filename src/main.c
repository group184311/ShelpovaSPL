/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/
#define IS_BUT1_PUSH (GPIOReadInputDataBit(GPIOC, GPIO_Pin_14)) //кнопка 1 PC14
#define IS_BUT2_PUSH (GPIOReadInputDataBit(GPIOB, GPIO_Pin_10)) //кнопка 2 PB10
#define IS_BUT3_PUSH (!(GPIOReadInputDataBit(GPIOB, GPIO_Pin_14))) //кнопка 3 PB14
#define IS_BUT4_PUSH (!(GPIOReadInputDataBit(GPIOB, GPIO_Pin_7))) //кнопка 4 PB7

//óñòàíîâêà ñâåòîäèîäà â ñîñòîÿíèå "ãîðåòü" - ëîãè÷.0 (reset)
//è â ñîñòîÿíèå "íå ãîðåòü" - ëîãè÷.1 (set)
#define SET_LED_ON (GPIO_WriteBit(GPIOC, GPIO_Pin13_RESET))
#define SET_LED_OFF (GPIO_WriteBit(GPIOC, GPIO_Pin13_SET))

//ïðîâåðêà ñîñòîÿíèÿ äèîäà
#define IS_LED_OFF (GPIOReadOutputDataBit(GPIOC, GPIO_Pin13))

//ìèíèìàëüíàÿ çàäåðæêà êíîïêè 1 = 50ìñ
#define MIN_TIME 100

//âðåìÿ êîììóòàöèè êàæäîé êíîïêè â ìñ
#define BUT2_TIME_MS 1500
#define BUT3_TIME_MS 2500
#define BUT4_TIME_MS 3500

//êîëè÷åñòâî òàêòîâ â ðåãèñòðå ARR â çàâèñèìîñòè îò çàäàííûõ ìñ
#define DEF_ARR (1000-1) //íà÷àëüíîå çíà÷åíèå = 500ìñ
#define BUT2_ARR (BUT2_TIME_MS/500*1000)
#define BUT3_ARR (BUT3_TIME_MS/500*1000)
#define BUT4_ARR (BUT4_TIME_MS/500*1000)

#include "stm32f10x.h"

int status = 0; //äëÿ îòñëåæèâàíèÿ íàæàòèÿ êíîïêè 1
unsigned int delay = DEF_ARR; //ïåðåìåííàÿ, õðàíÿùàÿ âðåìÿ ïàóçû êíîïêè 1
int light = DEF_ARR; //ïåðåìåííàÿ, õðàíÿçàÿ âðåìÿ êîììóòàöèè

int main(void)
{
		//âêëþ÷àåì òàêòèðîâàíèå ïîðòîà Ñ è Â
		RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

		GPIOC->CRH &= ~(GPIO_CRH_CNF13_1 | GPIO_CRH_CNF13_0| GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0 |
						GPIO_CRH_MODE14_1 | GPIO_CRH_MODE14_0 | GPIO_CRH_CNF14_0 | GPIO_CRH_CNF14_1);
		GPIOB->CRH &= ~(GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1 | GPIO_CRH_CNF10_0 | GPIO_CRH_CNF10_1 |
						GPIO_CRH_MODE14_0 | GPIO_CRH_MODE14_1 | GPIO_CRH_CNF14_0 | GPIO_CRH_CNF14_1);
		GPIOB->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7);

		//íàñòðàèâàåì ñâåòîäèîä íà âûõîä, êíîïêè íà âõîä
		GPIOC->CRH |= (GPIO_CRH_MODE13_1 | GPIO_CRH_CNF14_1);
		GPIOB->CRH |= (GPIO_CRH_CNF10_1 | GPIO_CRH_CNF14_1);
		GPIOB->CRL |= GPIO_CRL_CNF7_1;

		//êíîïêà 1 - âíóòð.ïîäòÿæêà ê çåìëå
		GPIOC->ODR &= ~GPIO_ODR_ODR14;
		//êíîïêà 2 - âíóòð.ïîäòÿæêà ê çåìëå
		GPIOB->ODR &= ~GPIO_ODR_ODR10;
		//êíîïêè 3 è 4 - âíóòð.ïîäòÿæêà ê ïèòàíèþ
		GPIOB->ODR |= (GPIO_ODR_ODR14 | GPIO_ODR_ODR7);

		// Âêëþ÷àåì òàêòèðîâàíèå òàéìåðà
	//	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
		//èíèöèàëèçèðóåì òàéìåð
		TIM_TimeBaseInitTypeDef tim3;
		tim3.TIM_ClockDivision = TIM_CKD_DIV1;
		tim3.TIM_CounterMode = TIM_CounterMode_Up;
		tim3.TIM_Period = 1000 - 1;
		tim3.TIM_Prescaler = 36000 - 1;
		TIM_TimeBaseInit(TIM3, &tim3);
	//	TIM3->PSC = 36000 - 1;
	//	TIM3->ARR = DEF_ARR;
		//çàïóñêàåì òàéìåð
	//	TIM3->CR1 |= TIM_CR1_CEN;
		//ðàçðåøàåì ïðåðûâàíèå
	//	TIM3->DIER |= TIM_DIER_UIE;

		//ðàçðåøàåì îáðàáîòêó ïðåðûâàíèÿ
	//	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_InitTypeDef nvicInit;
	nvicInit.NVIC_IRQChannel = TIM3_IRQn;
	nvicInit.NVIC_IRQChannelCmd = ENABLE;
	nvicInit.NVIC_IRQChannelPremptionPriority = 0;
	nvicInit.NVIC_IRQChannelSubPriority = 0;
	NVICInit(&nvicInit);
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
	TIM3->SR &= ~TIM_SR_UIF; //ñáðîñ ôëàãà
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
