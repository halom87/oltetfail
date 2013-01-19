#include "PWM.h"
#include "stm32f10x.h"

void PWM_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

//	Update_event = TIM_CLK/((PSC + 1)*(ARR + 1)*(RCR + 1))
//	10e3=64e6/(2*3200*1)
	//10kHz
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_Prescaler= 1; //32Mz
	TIM_TimeBaseStructure.TIM_Period=3200-1; //10kHz
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM4,&TIM_OCInitStructure);
	TIM_OC2Init(TIM4,&TIM_OCInitStructure);
	TIM_OC3Init(TIM4,&TIM_OCInitStructure);
	TIM_OC4Init(TIM4,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);

	TIM_Cmd(TIM4, ENABLE);





}

void PWM_SetDutyCycle(uint8_t percent)
{
	uint16_t compare;
	compare=(uint16_t)(percent*32);
	TIM_SetCompare1(TIM4,compare);
	TIM_SetCompare2(TIM4,compare);
	TIM_SetCompare3(TIM4,compare);
	TIM_SetCompare4(TIM4,compare);
}
void DebugTimerInit (void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_TimeBaseStructure.TIM_Prescaler=64;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);

	TIM_Cmd(TIM3, ENABLE);

}
