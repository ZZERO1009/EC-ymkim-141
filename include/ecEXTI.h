/*----------------------------------------------------------------\
@ Embedded Controller by Young-Min Kim - Handong Global University
Author           : 21900141 Young-Min Kim

Language/ver     : C++ in Keil uVision

Description      : Lib and Function define for make EXTI lab
/----------------------------------------------------------------*/

#ifndef __EC_EXTI_H
#define __EC_EXTI_H

#include "stm32f411xe.h"

#define FALL 0
#define RISE 1
#define BOTH 2

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

void EXTI_init(GPIO_TypeDef *Port, int pin, int trig, int priority);
void EXTI_enable(uint32_t pin);
void EXTI_disable(uint32_t pin);
uint32_t is_pending_EXTI(uint32_t pin);
void clear_pending_EXTI(uint32_t pin);
void EXTI15_10_IRQHandler(void);


#ifdef __cplusplus
}
#endif /* __cplusplus */
	 
#endif