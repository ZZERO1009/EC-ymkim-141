 /*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University

Author           : 21900141 Young-Min Kim

Language/ver     : C++ in Keil uVision

Description      : code for embeded controller test.

FAIL


0-> MID(CW)->MAX(CW)->0->MID(CCW)->MAX(CCW)->0->...

PWM, DIR -> Motor control	
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

#define PWM_PIN PA_0
#define DIR_PIN PC_2



uint32_t _count = 0;




void setup(void);


int main(void) {
	// Initialization --------------------------------------------------
	setup();
	
	
	// Infinite Loop ---------------------------------------------------
	while(1){
updateDisplay();		// 7seg update by exti 13
	
		
	}
}
	
	


// Initialization
void setup(void){
	RCC_PLL_init();				// System Clock = 84MHz
	GPIO_init(GPIOA, LED_PIN, OUTPUT);	// calls RCC_GPIOA_enable()
	
	PWM_init(PWM_PIN);
	GPIO_init(GPIOA, 0, AF);
	
	

	
	
	
	TIM_UI_init(TIM3, 1);			// TIM3 Update-Event Interrupt every 1 msec 
	TIM_UI_enable(TIM3);
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	  sevensegment_init(GPIOA, SEG_A_PIN, 1UL);
    sevensegment_init(GPIOB, SEG_B_PIN, 1UL);
    sevensegment_init(GPIOC, SEG_C_PIN, 1UL);
    sevensegment_init(GPIOA, SEG_D_PIN, 1UL);
	  EXTI_enable(13);
		EXTI_init(GPIOC, BUTTON_PIN, FALL,13);
		GPIO_pupd(GPIOC, BUTTON_PIN, 2);	// Pull up
		PWM_pulsewidth(PWM_PIN, 2000);
	}


void TIM3_IRQHandler(void){			//for motor
	if(is_UIF(TIM3)){			
		_count++;
		if (_count > 5000) {
			LED_toggle();		// LED toggle
			_count = 0;
		}
		clear_UIF(TIM3); 		// Clear UI flag by writing 0
	}
}

void TIM2_IRQHandler(void){
	if(is_UIF(TIM2)){			// Check UIF(update interrupt flag)
		_count++;
		for (int i=0; i<5; i++) {						
			PWM_duty(PWM_PIN, (float)0.2*i);			//PWN DUTY
			delay_ms(1000);
		}	
		clear_UIF(TIM2); 		// Clear UI flag by writing 0
	}
}