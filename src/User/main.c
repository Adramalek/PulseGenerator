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

uint8_t auto_reload = 0;

uint8_t scale = 0;
uint16_t bounds[] = {65, 655, 6553, 32768};
uint16_t prescalers[] = {83, 839, 8339, 41999};

uint8_t delay = 12; // in milliseconds
uint8_t pulse_delay = 20; // in milliseconds
uint8_t pulse = 20; // in milliseconds

Impulse_Mode impulse_mode = MULTI;

/**
	Commands list.
-- chmod: switch current mode between SINGLE and MULTI. No arguments.
-- autorld: enables/disables auto reload of impulses after changes. Recieve 0 or 1 as argument.
-- setdl: set delay between channels. Agument: unsigned integer.
-- setpl: set pulse time. Argument: unsigned integer.
-- setpdl: set delay between pulses. Argument: unsigned integer.
*/
char *commands[] = {"chmod", "autorld", "setdl", "setpl", "setpdl"};
void (*handlers[])(uint8_t) = {&Set_Autoreload ,&Set_Delay, &Set_Pulse, &Set_Pulse_Delay};

static TIM_Data pulse_delay_timer = {0, 0, TIM_CounterMode_Down, TIM5, RCC_APB1Periph_TIM5, TIM5_IRQn};
static TIM_Data delay_timer = {0, 0, TIM_CounterMode_Up, TIM3, RCC_APB1Periph_TIM3, TIM3_IRQn};
static TIM_Data pre_timer = {0, 0, TIM_CounterMode_Up, TIM4, RCC_APB1Periph_TIM4, TIM4_IRQn};
static TIM_Data post_timer = {0, 0, TIM_CounterMode_Up, TIM2, RCC_APB1Periph_TIM2, TIM2_IRQn};
static uint32_t turn_off = 0;
static char buff[CMD_MAX_LEN];


void Init_Button_Interrupt(){
	// Clock for GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// Clock for SYSCFG
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	// GPIOA initialization as an input from user button (GPIOA0)
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Selects the GPIOA pin 0 used as external interrupt source
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	// External interrupt settings
	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStruct);

	// Nested vectored interrupt settings
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	// EXTI0_IRQn has Most important interrupt
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVIC_InitStruct);
}

void Init_LED(){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef gpioStructure;
	gpioStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_12 | GPIO_Pin_14;
	gpioStructure.GPIO_Mode = GPIO_Mode_OUT;
	gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &gpioStructure);
}

void Init_USART(){
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef USART_InitStructure;
	
	__enable_irq();

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//GPIO_StructInit(&gpio);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpio);

	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_BaudRate = BAUD_RATE;	
	USART_Init(USART2, &USART_InitStructure);	

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	NVIC_EnableIRQ(USART2_IRQn);
	USART_Cmd(USART2, ENABLE);
}

static void Init_Timer(TIM_Data *timer){
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

void Init_Pulse_Delay(){
	Init_Timer(&pulse_delay_timer);
}

void Init_Delay(){
	Init_Timer(&delay_timer);
}

void Init_Post(){
	Init_Timer(&post_timer);
}

void Init_Pre(){
	Init_Timer(&pre_timer);
}

void Set_Periods(){
	pulse_delay_timer.Period = (pulse_delay+delay)*1000-1;
	pulse_delay_timer.Prescaler = prescalers[scale];
	delay_timer.Period = delay*1000-1;
	delay_timer.Prescaler = prescalers[scale];
	pre_timer.Period = pulse*1000-1;
	pre_timer.Prescaler = prescalers[scale];
	post_timer.Period = pulse*1000-1;
	post_timer.Prescaler = prescalers[scale];
}

void Init(){
	Set_Periods();
	Init_LED();
	Init_Button_Interrupt();
	Init_USART();
	Init_Pulse_Delay();
	Init_Delay();
	Init_Pre();
	Init_Post();
}

void ReinitDelays(){
	Stop_Timers();
	turn_off = 0;
	Set_Periods();
	if ((impulse_mode == MULTI) && auto_reload){
		TIM_Cmd(TIM5, ENABLE);
		turn_off = 1;
	}
}

uint8_t Rescale(uint8_t time){
	uint8_t is_valid = 0;
	uint16_t *p = bounds;
	do{
		is_valid |= time < *p;
		if (is_valid){
			scale = p-bounds;
			break;
		}
	}while(++p < bounds+4);
	return is_valid;
}

void Set_Autoreload(uint8_t enable){
	auto_reload = enable;
}

void Set_Delay(uint8_t value){
	if (Rescale(value)){
		delay = value;
		ReinitDelays();
	}
}

void Set_Pulse(uint8_t value){
	if (Rescale(value)){
		pulse = value;
		ReinitDelays();
	}
}

void Set_Pulse_Delay(uint8_t value){
	if (Rescale(value)){
		pulse_delay = value;
		ReinitDelays();
	}
}

uint8_t Parse_Apply_Cmd(char *cmd_str){
	char *token = strtok(cmd_str, " ");
	if (token != NULL){
		for (int i = 1; i < 5; ++i){
			if (strcmp(commands[i], token)){
				handlers[i-1](strtol((token = strtok(NULL, " ")),NULL, 10));
				return 1;
			}
		}
	} else {
		if (strcmp(commands[0],cmd_str)){
			impulse_mode = !impulse_mode;
			return 1;
		}
	}
	return 0;
}

void USART2_IRQHandler(){
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		static char *p_buff = buff;
		if ((*p_buff = USART_ReceiveData(USART2)) == END_CHAR || p_buff < buff+CMD_MAX_LEN){
			*p_buff = '\0';
			if (Parse_Apply_Cmd(buff)){
				Send_Message("OK");
			} else {
				Send_Message(strcat("Unresolved command", buff));
			}
			memset(buff, 0, CMD_MAX_LEN);
			p_buff = buff;
		} else {
			p_buff++;
		}
	}
}

void Send_Message(char *msg){
	int len = strlen(msg);
	char *p_msg = msg;
	while(p_msg < msg+len){
		while(USART_GetITStatus(USART2, USART_IT_TXE) == SET);
		USART_SendData(USART2, (*p_msg)++);
	}
}

void TIM2_IRQHandler(){
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 
		GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_RESET);
		TIM_Cmd(TIM2, DISABLE);
	}
}

void TIM3_IRQHandler(){
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		TIM_Cmd(TIM3, DISABLE);
		TIM_Cmd(TIM2, ENABLE);
		GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_SET);
	}
}

void TIM4_IRQHandler(){
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_RESET);
		TIM_Cmd(TIM4, DISABLE);
	}
}

void TIM5_IRQHandler(){
		if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET){
			TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
			Start_Timers();
		}
}

void Start_Timers(){
	TIM_Cmd(TIM4, ENABLE);
	GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_SET);
	TIM_Cmd(TIM3, ENABLE);
}

void Stop_Timers(){
	TIM_Cmd(TIM5, DISABLE);
	TIM4_IRQHandler();
	TIM_Cmd(TIM3, DISABLE);
	TIM2_IRQHandler();
}

void EXTI0_IRQHandler(){
	if (EXTI_GetITStatus(EXTI_Line0) != RESET){
		EXTI_ClearITPendingBit(EXTI_Line0);
		if (impulse_mode == MULTI){
			if (turn_off){
				Stop_Timers();
			} else {
				TIM_Cmd(TIM5, ENABLE);
			}
			GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
			turn_off = !turn_off;
		} else {
			Start_Timers();
		}
	}
}

int main(void) {
	//PD12 -- Pre, PD15 - Post
	
	/* Initialize system */
	SystemInit();
	
	//Init();
	Set_Periods();
	Init_LED();
	Init_Button_Interrupt();
	//Init_USART();
	Init_Pulse_Delay();
	Init_Delay();
	Init_Pre();
	Init_Post();
	
	while (1){
	
	}
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


