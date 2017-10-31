/**
 *	Keil project for PWM example
 *
 *	Before you start, select your target, on the right of the "Load" button
 *	
 *	STM32F429-Discovery doesn't have leds connected to PWM pins,
 *	so you can test this with STM32F4-Discovery or Nucleo-F4(0/1)1RE boards with leds.
 *	
 *	Below is code for both boards in main.c
 *	You have to set correct target above first, but code will be always compiled,
 *	if you select STM32F4-Discovery or Nucleo F401RE boards
 *
 *	@author		Tilen Majerle
 *	@email		tilen@majerle.eu
 *	@website	http://stm32f4-discovery.com
 *	@ide		Keil uVision 5
 *	@packs		STM32F4xx Keil packs version 2.2.0 or greater required
 *	@stdperiph	STM32F4xx Standard peripheral drivers version 1.4.0 or greater required
 */
 
#include "main.h"

void InitializeDelay(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_TimeBaseInitTypeDef timerInitStructure; 
	timerInitStructure.TIM_Prescaler = 83;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             ;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = DELAY*1000-1;
	timerInitStructure.TIM_ClockDivision = 0;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &timerInitStructure);

	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = TIM3_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
	
	TIM_ClearITPendingBit (TIM3, TIM_IT_Update);
  TIM_SetCounter (TIM3, 0);
  TIM_ITConfig (TIM3, TIM_IT_Update, ENABLE);
}

void InitializePost()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef gpioStructure;
	gpioStructure.GPIO_Pin = GPIO_Pin_15;
	gpioStructure.GPIO_Mode = GPIO_Mode_OUT;
	gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &gpioStructure);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseInitTypeDef timerInitStructure; 
	timerInitStructure.TIM_Prescaler = 83;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             ;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 19999;
	timerInitStructure.TIM_ClockDivision = 0;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &timerInitStructure);


	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
	
	TIM_ClearITPendingBit (TIM2, TIM_IT_Update);
  TIM_SetCounter (TIM2, 0);
  TIM_ITConfig (TIM2, TIM_IT_Update, ENABLE);
}

void InitializePre()
{
	GPIO_InitTypeDef gpio_C; 
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef gpioStructure;
	gpioStructure.GPIO_Pin = GPIO_Pin_12;
	gpioStructure.GPIO_Mode = GPIO_Mode_OUT;
	gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &gpioStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_TimeBaseInitTypeDef timerInitStructure; 
	timerInitStructure.TIM_Prescaler = 83;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 19999;
	timerInitStructure.TIM_ClockDivision = 0;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &timerInitStructure);

	
	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = TIM4_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
	
	TIM_ClearITPendingBit (TIM4, TIM_IT_Update);
  TIM_SetCounter (TIM4, 0);
  TIM_ITConfig (TIM4, TIM_IT_Update, ENABLE);
}


void TIM2_IRQHandler(){
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 
		GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_RESET);
		TM_DISCO_LedOff(LED_ORANGE);
		TIM_Cmd(TIM2, DISABLE);
	}
}

void TIM3_IRQHandler(){
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		TIM_Cmd(TIM3, DISABLE);
		TIM_Cmd(TIM2, ENABLE);
		GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_SET);
		TM_DISCO_LedOn(LED_ORANGE);
	}
}

void TIM4_IRQHandler(){
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_RESET);
		TM_DISCO_LedOff(LED_RED);
		TIM_Cmd(TIM4, DISABLE);
	}
}

void TM_EXTI_Handler(uint16_t GPIO_Pin){
	if (GPIO_Pin == GPIO_Pin_0){
		TIM_Cmd(TIM4, ENABLE);
		GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_SET);
		TM_DISCO_LedOn(LED_RED);
		TIM_Cmd(TIM3, ENABLE);
	}
}

int main(void) {
	//PD12 -- Pre, PD15 - Post

	
	/* Initialize system */
	SystemInit();
	TM_DELAY_Init();
	TM_DISCO_LedInit();
	TM_DISCO_ButtonInit();
	
	InitializeDelay();
	InitializePre();
	InitializePost();

	if (TM_EXTI_Attach(GPIOA, GPIO_Pin_0, TM_EXTI_Trigger_Rising) == TM_EXTI_Result_Ok){
		//TM_DISCO_LedOn(LED_RED);
	}
	
	while (1){
	
	}
}

