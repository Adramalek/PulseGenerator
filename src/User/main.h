#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx.h"
#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include <string.h>
#include <stdlib.h>

#define BAUD_RATE 9600
#define CMD_MAX_LEN 14
#define END_CHAR 'x'

typedef enum {
	SINGLE = 0,
	MULTI = !SINGLE
} Impulse_Mode;

typedef struct {
	uint16_t Prescaler;
	uint16_t Period;
	uint16_t TIM_CounterMode;
	TIM_TypeDef* TIM;
	uint32_t RCC_APB1Periph_TIM;
	uint8_t NVIC_TIM_IRQChannel;
} TIM_Data;

void Start_Timers(void);
void Stop_Timers(void);
uint8_t Rescale(uint8_t);
void Set_Autoreload(uint8_t);
void Set_Delay(uint8_t);
void Set_Pulse(uint8_t);
void Set_Pulse_Delay(uint8_t); // this is not an union of two functions above
void Set_Time(void);
void Reset_Time(uint8_t);
uint8_t Parse_Apply_Cmd(char*);
void Send_Message(char *);
void ReinitDelays(void);
void Init(void);
void Init_Minute_Timer(void);
void Init_Pre(void);
void Init_Post(void);
void Init_Delay(void);
void Init_Pulse_Delay(void);
void Init_USART(void);
void Init_LED(void);
void Init_Button_Interrupt(void);

//#ifndef SETTINGS
//#define SETTINGS

const uint16_t bounds[] = {65, 655, 6553, 32768};
const uint16_t prescalers[] = {83, 839, 8339, 41999};
const uint16_t scalers[] = {1000, 100, 10, 2};
const uint16_t time = 30000;

uint8_t auto_reload = 0;
uint8_t scale = 0;
uint8_t delay = 40; // in milliseconds. delay beetween chanells
uint8_t pulse_delay = 60; // in milliseconds. pulse_delay + pulse = period
uint8_t pulse = 60; // in milliseconds. duty in ms
uint8_t time_factor = 2;
uint8_t tmd = 1;
uint32_t turn_off = 0;
uint8_t step = 0;


Impulse_Mode impulse_mode = MULTI;

/**
	Commands list.
-- chmod: switch current mode between SINGLE and MULTI. No arguments.
-- autorld: enables/disables auto reload of impulses after changes. Recieve 0 or 1 as argument.
-- setdl: set delay between channels. Agument: unsigned integer.
-- setpl: set pulse time. Argument: unsigned integer.
-- setpdl: set delay between pulses. Argument: unsigned integer.
*/
char *commands[] = {"tmd", "chmod", "autorld", "setdl", "setpl", "setpdl", "settm"};
void (*handlers[])(uint8_t) = {&Set_Autoreload ,&Set_Delay, &Set_Pulse, &Set_Pulse_Delay, &Reset_Time};
//void (*no_args_handlers[])(void) = {};

TIM_Data pulse_delay_timer = {0, 0, TIM_CounterMode_Down, TIM5, RCC_APB1Periph_TIM5, TIM5_IRQn};
TIM_Data delay_timer = {0, 0, TIM_CounterMode_Up, TIM3, RCC_APB1Periph_TIM3, TIM3_IRQn};
TIM_Data pre_timer = {0, 0, TIM_CounterMode_Up, TIM4, RCC_APB1Periph_TIM4, TIM4_IRQn};
TIM_Data post_timer = {0, 0, TIM_CounterMode_Up, TIM2, RCC_APB1Periph_TIM2, TIM2_IRQn};
TIM_Data minute_timer = {0, 0, TIM_CounterMode_Up, TIM7, RCC_APB1Periph_TIM7, TIM7_IRQn};

char buff[CMD_MAX_LEN];
//#endif

#endif
