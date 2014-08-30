/**
 * @file		servo_control.c
 * @author		Stian Sandve
 */

#include "servo_control.h"
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

static void init_servo_gpio();
static void init_servo_timer();
static int duty_cycle_to_degrees(int degrees);

static uint32_t servo = CENTER;
static Direction servo_dir = DIR_CLOCKWISE;

void servo_control_init()
{
	init_servo_gpio();
	init_servo_timer();
}

static void init_servo_gpio()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |
							   RCC_APB2Periph_AFIO, ENABLE);

		GPIO_InitTypeDef itd;

		itd.GPIO_Pin = GPIO_Pin_6;
		itd.GPIO_Mode = GPIO_Mode_AF_PP;
		itd.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &itd);
}

static void init_servo_timer()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseInitTypeDef timTbItd;

	/**
	 * Servos need a period of 20ms and a duty cycle between 1ms and 2ms.
	 * What is 20ms in Hertz? --> 1000ms / 20ms = 50Hz.
	 * 20ms is equal to 20000us, 1ms is 1000us and 2ms is 2000us.
	 * We prescale the 24MHz clock frequency by 24 (24 - 1 since it is zero based)
	 * 24 000 000 Hz / 24 = 1 000 000 Hz (1MHz).
	 * CNT register becomes 1 000 000 each second. To calculate the correct
	 * value for the ARR register (which defines the period), we divide the
	 * result of the prescaled frequency by the intended frequency for our period.
	 * 1 000 000 Hz / 50 Hz = 20 000.
	 * Every time the CNT register is increased by one, this is the equivalent to
	 * 1us. The duty cycle is now easily set by writing to the CCRx register.
	 * For a 1.5ms duty cycle, we set the CCRx register to 1500 and so on.
	 */

	timTbItd.TIM_Period = 20000 - 1;
	timTbItd.TIM_Prescaler = (uint16_t) 24 - 1;
	timTbItd.TIM_ClockDivision = TIM_CKD_DIV1;
	timTbItd.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &timTbItd);

	TIM_OCInitTypeDef timOcItd;

	timOcItd.TIM_OCMode = TIM_OCMode_PWM1;
	timOcItd.TIM_OutputState = TIM_OutputState_Enable;
	timOcItd.TIM_Pulse = 1500;
	timOcItd.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM4, &timOcItd);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);

	TIM_Cmd(TIM4, ENABLE);
}

void move_servo(Direction dir)
{
	switch(dir)
	{
	case DIR_CLOCKWISE:
		if(servo <= MAX_DEFLECTION + MOVE)
		{
			servo = servo - MOVE;
		}
		break;
	case DIR_COUNTER_CLOCKWISE:
		if(servo >= MIN_DEFLECTION - MOVE)
		{
			servo = servo + MOVE;
		}
		break;
	case DIR_CENTER:
		servo = CENTER;
		break;
	}
	set_servo_duty_cycle(servo);
}

void set_servo_duty_cycle(int pos)
{
	if(pos <= MAX_DEFLECTION && pos >= MIN_DEFLECTION)
	{
		TIM4->CCR1 = pos;
	}
}

bool set_servo_pos(int degrees)
{
	if(degrees >= 0 && degrees <= 180)
	{
		set_servo_duty_cycle(degrees_to_duty_cycle(degrees));
		return true;
	}
	return false;
}

int duty_cycle_to_degrees(int duty_cycle)
{
	return (duty_cycle - MIN_DEFLECTION) / ((MAX_DEFLECTION - MIN_DEFLECTION) / 180);
}

int degrees_to_duty_cycle(int degrees)
{
	return (((MAX_DEFLECTION - MIN_DEFLECTION) / 180) * degrees) + MIN_DEFLECTION;
}

void center_servo()
{
	set_servo_duty_cycle(CENTER);
}

int get_servo_pos()
{
	return duty_cycle_to_degrees(servo);
}

void toggle_servo()
{
	if(get_servo_pos() == MIN_DEFLECTION && servo_dir == DIR_CLOCKWISE)
		servo_dir = DIR_COUNTER_CLOCKWISE;
	else if (get_servo_pos() == MAX_DEFLECTION && servo_dir == DIR_COUNTER_CLOCKWISE)
		servo_dir = DIR_CLOCKWISE;

	if(servo_dir == DIR_CLOCKWISE)
		move_servo(DIR_CLOCKWISE);
	else if(servo_dir == DIR_COUNTER_CLOCKWISE)
		move_servo(DIR_COUNTER_CLOCKWISE);
}
