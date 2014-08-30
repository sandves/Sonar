#include <stdio.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_tim.h>
#include <stm32f10x.h>
#include <misc.h>
#include <stm32f10x_usart.h>
#include "queue.h"

/**
 * Application that interacts with the HC-SR04 ultrasonic
 * sensor and transmits the measured distance over USART1
 * once per second.
 */

#define CARRIAGE_RETURN		13
#define ASCII_DIGIT_BASE	48

// Members
uint32_t delay_time = 100;
int led_state = 0;
static __IO uint32_t TimingDelay;
Queue UART1_TXq;
static int TxPrimed = 0;
volatile char Message[] = "Pulse width of channel 1 = ";
uint16_t distance = 0;

// Function prototypes
static void init_HC_SR04();
static void init_gpio();
static void init_trigger_timer();
static void init_echo_timer();
static void init_read_timer();
static void init_usart();
static void sendchar(uint16_t);
static void toggleInternalLEDs();
static void delay(uint32_t time);

int main(void)
{
	// Tick every ms
	SysTick_Config(SystemCoreClock / 1000);

	init_HC_SR04();

	toggleInternalLEDs();

	while(1)
	{
	}
}

static void init_HC_SR04()
{
	init_gpio();
	init_trigger_timer();
	init_echo_timer();
	init_read_timer();
	init_usart();
}

/**
 * Configure GPIO pins
 */
static void init_gpio()
{
	// PB6 -----> TIM4_CH1
	// PA8 -----> TIM1_CH1

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA
			| RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef GPIO_InitTypeStructure;

	// Trigger pin
	GPIO_InitTypeStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitTypeStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitTypeStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitTypeStructure);

	// Echo pin
	GPIO_InitTypeStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitTypeStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitTypeStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitTypeStructure);

	// USART1_Tx
	GPIO_InitTypeStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitTypeStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitTypeStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitTypeStructure);

	// USART1_Rx
	GPIO_InitTypeStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitTypeStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitTypeStructure);

	//Internal LEDs
	GPIO_InitTypeStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitTypeStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitTypeStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitTypeStructure);

}

/**
 * Configure TIM4 to generate a 10us pulse at a 10Hz rate
 * to trigger the ultrasonic sensor.
 */
static void init_trigger_timer()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = (uint16_t) 24000 - 1;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 10;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);

	TIM_Cmd(TIM4, ENABLE);
}

/**
 * Configuration of TIM1.
 * - Configure channel 1 to latch timer on a rising input on t1
 * - Configure channel 2 to latch timer on a falling input on t2
 * - Configure TIM1 in slave mode to reset on the capture 1 event
 */
static void init_echo_timer()
{
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 24000 - 1;
	// Slightly longer period than the trigger signal
	TIM_TimeBaseStructure.TIM_Period = 150 - 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = 0;
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = 0;
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	TIM_SelectInputTrigger(TIM1, TIM_TS_TI1FP1);
	TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);

	TIM_Cmd(TIM1, ENABLE);
}

/**
 * Configure TIM2 to generate an interrupt every second
 * so that we can read the distance measured by the ultrasonic
 * sensor.
 */
static void init_read_timer()
{
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	TIM_OCInitTypeDef         TIM_OCInitStructure;
	NVIC_InitTypeDef          NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_OCStructInit(&TIM_OCInitStructure);

	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 1000) - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

}

static void init_usart()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	USART_InitTypeDef USART_InitStructure;
	// Initialize USART structure
	USART_StructInit(&USART_InitStructure);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	USART_Init(USART1, &USART_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void)
{
	toggleInternalLEDs();

	static int tx_index = 0;
	static int rx_index = 0;
	static int done = 0;

	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
		if(!done)
		{
			USART_SendData(USART1, Message[tx_index++]);

			if (tx_index >= (sizeof(Message)))
			{
				tx_index = 0;

				USART_SendData(USART1, (distance + ASCII_DIGIT_BASE));
				done = 1;
			}
		}
		else
		{
			done = !done;
			USART_SendData(USART1, CARRIAGE_RETURN);
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		}
	}
}

/**
 * Print the values captured by channel 1 and 2 on TIM1.
 * Toggle the chips internal LEDs for debugging purposes.
 */
void TIM2_IRQHandler(void) {

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		uint16_t ch1 = TIM_GetCapture1(TIM1);
		uint16_t ch2 = TIM_GetCapture2(TIM1);
		sendchar((ch1));

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

static void sendchar(uint16_t c)
{
	distance = c;
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

static void toggleInternalLEDs()
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_8, led_state ? Bit_RESET : Bit_SET);
	GPIO_WriteBit(GPIOC, GPIO_Pin_9, led_state ? Bit_RESET : Bit_SET);
	led_state = !led_state;
}

/**
 * Sleep for a given amount of milliseconds
 */
static void delay(uint32_t time)
{
	TimingDelay = time;
	while(TimingDelay != 0);
}

void SysTick_Handler()
{
	if (TimingDelay != 0x00)
		TimingDelay--;
}
