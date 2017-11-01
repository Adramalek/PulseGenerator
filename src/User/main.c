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

uint8_t DELAY = 12; // in milliseconds
uint8_t PULSE_DELAY = 40; // in milliseconds
uint8_t TRAINING_TIME = 30; // in seconds (32 sec is max value)

//PULSE_DELAY*1000-1 (PULSE_DELAY+DELAY)*1000-1
//static TIM_Data training_timer = {41999, TRAINING_TIME*1000-1, TIM_CounterMode_Down, TIM5, RCC_APB1Periph_TIM5, TIM5_IRQn};
static TIM_Data pulse_delay = {83, 0, TIM_CounterMode_Down, TIM5, RCC_APB1Periph_TIM5, TIM5_IRQn};
static TIM_Data delay_timer = {83, 0, TIM_CounterMode_Up, TIM3, RCC_APB1Periph_TIM3, TIM3_IRQn};
static TIM_Data pre_timer = {83, 19999, TIM_CounterMode_Up, TIM4, RCC_APB1Periph_TIM4, TIM4_IRQn};
static TIM_Data post_timer = {83, 19999, TIM_CounterMode_Up, TIM2, RCC_APB1Periph_TIM2, TIM2_IRQn};

void Init(){
	pulse_delay.Period = (PULSE_DELAY+DELAY)*1000-1;
	delay_timer.Period = DELAY*1000-1;
}

void Init_Timer(TIM_Data *timer){
	RCC_APB1PeriphClockCmd(timer->RCC_APB1Periph_TIM, ENABLE);
	TIM_TimeBaseInitTypeDef timerInitStructure; 
	timerInitStructure.TIM_Prescaler = timer->Prescaler;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          ;
	timerInitStructure.TIM_CounterMode = timer->TIM_CounterMode;
	timerInitStructure.TIM_Period = timer->Period;
	timerInitStructure.TIM_ClockDivision = 0;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(timer->TIM, &timerInitStructure);
	
	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = timer->NVIC_TIM_IRQChannel;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
	
	TIM_ClearITPendingBit (timer->TIM, TIM_IT_Update);
  TIM_SetCounter (timer->TIM, 0);
  TIM_ITConfig (timer->TIM, TIM_IT_Update, ENABLE);
}

void Init_pulse_delay(){
	Init_Timer(&pulse_delay);
}

void Initialize_Delay(){
	Init_Timer(&delay_timer);
}

void Initialize_Post()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef gpioStructure;
	gpioStructure.GPIO_Pin = GPIO_Pin_15;
	gpioStructure.GPIO_Mode = GPIO_Mode_OUT;
	gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &gpioStructure);
	
	Init_Timer(&post_timer);
}

void Initialize_Pre()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef gpioStructure;
	gpioStructure.GPIO_Pin = GPIO_Pin_12;
	gpioStructure.GPIO_Mode = GPIO_Mode_OUT;
	gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &gpioStructure);

	Init_Timer(&pre_timer);
}

void TIM2_IRQHandler(){
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 
		GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_RESET);
		TM_DISCO_LedToggle(LED_ORANGE);
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
		TM_DISCO_LedToggle(LED_RED);
		TIM_Cmd(TIM4, DISABLE);
	}
}

void TIM5_IRQHandler(){
		if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET){
			TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
			Start_Timers();
		}
}

inline void Stop_Timers(){
	TIM_Cmd(TIM5, DISABLE);
	TIM4_IRQHandler();
	TIM_Cmd(TIM3, DISABLE);
	TIM2_IRQHandler();
}

inline void Start_Timers(){
	TIM_Cmd(TIM4, ENABLE);
	GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_SET);
	TM_DISCO_LedOn(LED_RED);
	TIM_Cmd(TIM3, ENABLE);
}

void TM_EXTI_Handler(uint16_t GPIO_Pin){
	if (GPIO_Pin == GPIO_Pin_0){
		if (Mode == MULTI){
			static uint8_t stop_timer = 0;
			if (stop_timer){
				Stop_Timers();
			} else {
				TIM_Cmd(TIM5, ENABLE);
			}
			stop_timer = !stop_timer;
		} else {
			Start_Timers();
		}
	}
}



int main(void) {
	//PD12 -- Pre, PD15 - Post

	
	/* Initialize system */
	SystemInit();
	TM_DISCO_LedInit();
	TM_DISCO_ButtonInit();
	
	Init();
	Init_pulse_delay();
	Initialize_Delay();
	Initialize_Pre();
	Initialize_Post();

	if (TM_EXTI_Attach(GPIOA, GPIO_Pin_0, TM_EXTI_Trigger_Rising) == TM_EXTI_Result_Ok){
	}
	
	while (1){
	
	}
}

