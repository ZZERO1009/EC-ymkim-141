/*----------------------------------------------------------------\
@ Embedded Controller by Youngmin - Handong Global University


Language/ver     : C++ in Keil uVision

Description      : for RCC
/----------------------------------------------------------------*/


#include "stm32f411xe.h"
#include "ecRCC.h"

#ifndef __ECGPIO_H
#define __ECGPIO_H

#define INPUT  0x00
#define OUTPUT 0x01
#define AF     0x02
#define ANALOG 0x03

#define HIGH 1
#define LOW  0

#define LED_PIN 	5
#define BUTTON_PIN 13
#define SEG_A_PIN 7
#define SEG_B_PIN 6
#define SEG_C_PIN 7
#define SEG_D_PIN 9


#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output);
int  GPIO_read(GPIO_TypeDef *Port, int pin);
void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode);
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
void GPIO_pupd(GPIO_TypeDef* Port, int pin, int pupd);
void sevensegment_init(GPIO_TypeDef *Port, int pin, int Output); 
void sevensegment_decoder(unsigned int num);
void updateDisplay(void);
void LED_toggle(void);
extern volatile unsigned int cnt;


 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
