 /*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : 21900141 Young-Min Kim

Language/ver     : C++ in Keil uVision

Description      : MAIN code of LAB: PWM
/----------------------------------------------------------------*/

#include "stm32f411xe.h"
#include "math.h"

#include "..\..\lib\ecPinNames.h"
#include "..\..\lib\ecGPIO.h"
#include "..\..\lib\ecSysTick.h"
#include "..\..\lib\ecRCC.h"
#include "..\..\lib\ecTIM.h"
#include "..\..\lib\ecPWM.h" 
#include "..\..\lib\ecEXTI.h"
// Definition Button Pin & PWM Port, Pin
#define BUTTON_PIN 13
#define PWM_PIN PA_5
void setup(void);


int main(void) {
	// Initialization --------------------------------------------------
	setup();	
	

	// Infinite Loop ---------------------------------------------------
	while(1){
		LED_toggle();		
		for (int i=0; i<10; i++) {						
			TIM2->CCR1 = i*1000;			
			delay_ms(100);
		}
			for (int i=0; i<10; i++) {						
			TIM2->CCR1 = (10-i)*1000;			
			delay_ms(100);
		
	}

}
	}


// Initialiization 
void setup(void) {	
	RCC_PLL_init();
	SysTick_init();
		

	// PWM of 20 msec:  TIM2_CH1 (PA_5 AFmode)
	GPIO_init(GPIOA, 5, AF);
	PWM_init(PWM_PIN);	
	PWM_period(PWM_PIN, 20);   // 20 msec PWM period

}