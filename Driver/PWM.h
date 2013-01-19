/*
 * PWM.h
 *
 *  Created on: 2012.10.27.
 *      Author: Adam
 */
#include "stm32f10x.h"

#ifndef PWM_H_
#define PWM_H_
void PWM_Config(void);
void PWM_SetDutyCycle(uint8_t percent);
void DebugTimerInit (void);


#endif /* PWM_H_ */
