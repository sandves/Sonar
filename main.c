#include <stdio.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_tim.h>
#include <stm32f10x.h>
#include <misc.h>

static void init_trigger_timer();
static void init_echo_timer();
static void init_read_timer();
static void init_HC_SR04();
static void init_gpio();
static void delay(uint32_t time);

uint32_t delay_time = 100;
int led_state = 0;
static __IO uint32_t TimingDelay;

int main(void)
{
	SysTick_Config(SystemCoreClock / 1000);
	init_HC_SR04();

	while(1)
	{
	}
}

static void init_HC_SR04()
{
	init_trigger_timer();
	init_echo_timer();
	init_read_timer();
	init_gpio();
}

static void init_trigger_timer()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseInitTypeDef timTbItd;

	timTbItd.TIM_Period = 100 - 1;
	timTbItd.TIM_Prescaler = (uint16_t) 24000 - 1;
	timTbItd.TIM_ClockDivision = TIM_CKD_DIV1;
	timTbItd.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &timTbItd);

	TIM_OCInitTypeDef timOcItd;

	timOcItd.TIM_OCMode = TIM_OCMode_PWM1;
	timOcItd.TIM_OutputState = TIM_OutputState_Enable;
	timOcItd.TIM_Pulse = 10;
	timOcItd.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM4, &timOcItd);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);

	TIM_Cmd(TIM4, ENABLE);
}

static void init_echo_timer()
{
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	// Configure TIM1 prescaler and period
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 24000 - 1; // 0..479
	TIM_TimeBaseStructure.TIM_Period = 150 - 1; // 0..999
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	// Configure channel 1 to latch the timer on a rising input on t1.
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = 0;
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	// Cofigure channel 2 to latch the timer on a falling input on t2.
	//TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = 0;
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	// Cofigure TIM1 in slave mode to reset on the capture 1 event.
	TIM_SelectInputTrigger(TIM1, TIM_TS_TI1FP1);
	TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);

	/* enable counter TIM1 */
	TIM_Cmd(TIM1, ENABLE);
}

static void init_read_timer()
{

	uint16_t PrescalerValue = 0;

	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	TIM_OCInitTypeDef         TIM_OCInitStructure;
	NVIC_InitTypeDef          NVIC_InitStructure;

	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_OCStructInit(&TIM_OCInitStructure);

	/*Compute the prescaler value */
	PrescalerValue = (uint16_t) (SystemCoreClock / 1000) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 1000;      //in mSecs
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* Enable the TIM2 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

}

static void init_gpio()
{

	// PB6 -----> TIM4_CH1
	// PA8 -----> TIM1_CH1

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef itd;

	itd.GPIO_Pin = GPIO_Pin_6;
	itd.GPIO_Mode = GPIO_Mode_AF_PP;
	itd.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &itd);

	itd.GPIO_Pin = GPIO_Pin_8;
	itd.GPIO_Mode = GPIO_Mode_AF_PP;
	itd.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &itd);

	//Internal LEDs
	itd.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	itd.GPIO_Mode = GPIO_Mode_Out_PP;
	itd.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &itd);

}

void TIM2_IRQHandler(void) {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		printf("TIM2 interrupt\r\n");
		printf("SR04: %u %u\r\n", TIM_GetCapture1(TIM1), TIM_GetCapture2(TIM1));

		GPIO_WriteBit(GPIOC, GPIO_Pin_8, led_state ? Bit_RESET : Bit_SET);
		GPIO_WriteBit(GPIOC, GPIO_Pin_9, led_state ? Bit_RESET : Bit_SET);
		led_state = !led_state;

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

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