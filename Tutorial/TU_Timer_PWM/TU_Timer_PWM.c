#include "stm32f411xe.h"
#include "..\..\lib\ecRCC.h"
#include "..\..\lib\ecGPIO.h"
#include "..\..\lib\ecSysTick.h"
#include "..\..\lib\ecTIM.h"
#define LED_PIN 	5

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	RCC_PLL_init();                         // System Clock = 84MHz
	SysTick_init();                         // for delay_ms()
	GPIO_init(GPIOA, LED_PIN, 2);     // GPIOA 5 ALTERNATE function
	GPIO_ospeed(GPIOA, LED_PIN, 3);   // GPIOA 5 HIGH SPEED
	
	// TEMP: TIMER Register Initialiization --------------------------------------------------------		
	TIM_TypeDef *TIMx;
	TIMx = TIM2;
	
	// GPIO: ALTERNATIVE function setting
	GPIOA->AFR[0]	 = (1 << (LED_PIN * 4));      						// AF1 at PA5 = TIM2_CH1 (p.150)
	
	// TIMER: PWM setting
	RCC->APB1ENR |= 0x01<<(4*LED_PIN);              				// Enable TIMER clock
	
	TIMx->CR1 &= ~ (1<<0)			;	              		// Direction Up-count
	
	TIMx->PSC = 	1000000;        // Set Timer CLK = 100kHz : (PSC + 1) = 84MHz/100kHz --> PSC = ?
	
	TIMx->ARR = 		1000-1;			        // Auto-reload: Upcounting (0...ARR). 
																				// Set Counter CLK = 1kHz : (ARR + 1) = 100kHz/1kHz --> ARR = ?
	
	TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;  			// Clear ouput compare mode bits for channel 1
	TIMx->CCMR1 |= (6 << 4)   	;		// OC1M = 110 for PWM Mode 1 output on ch1
	TIMx->CCMR1	|= TIM_CCMR1_OC1PE;    		// Output 1 preload enable (make CCR1 value changable)
	
	TIMx->CCR1 =  500;	// Output Compare Register for channel 1 	
	
	TIMx->CCER &= ~TIM_CCER_CC1P;    			// select output polarity: active high	
	TIMx->CCER |= 			TIM_CCER_CC1E;  									// Enable output for ch1
	
	TIMx->CR1  |= TIM_CR1_CEN;      			// Enable counter
	
	// Inifinite Loop ----------------------------------------------------------
	while(1) {
    // Increase brightness from 0% to 100%
    for (int i = 0; i <= 1000; i++) {
        TIM2->CCR1 = i;      // Set PWM duty cycle
        delay_ms(1);            // Delay for 1ms
    }

    // Decrease brightness from 100% to 0%
    for (int i = 1000; i >= 0; i--) {
        TIM2->CCR1 = i;      // Set PWM duty cycle
        delay_ms(1);            // Delay for 1ms
    }
}

}
// Initialiization 
void setup(void) {	
	TIM_init(TIM2, 1);
	 GPIO_pupd(GPIOA, LED_PIN, 0);
    GPIO_ospeed(GPIOA, LED_PIN, 1UL);
   GPIO_otype(GPIOA, LED_PIN, 0);
	RCC_PLL_init();       // System Clock = 84MHz
	SysTick_init();       // for delay_ms()
}