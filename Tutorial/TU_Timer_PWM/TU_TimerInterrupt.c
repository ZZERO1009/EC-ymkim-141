/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2023-10-19 Young-Min Kim	
* @brief   Embedded Controller:  Tutorial for interuupt
* 
******************************************************************************
*/
#include "stm32f411xe.h"
#include "..\..\lib\ecRCC.h"
#include "..\..\lib\ecGPIO.h"


uint32_t count = 0;


void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	TIM_TypeDef* timerx;
	timerx = TIM2;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	timerx->PSC = 		1000000;		// Set Timer Clock Pre-scaler value 84MHz To 100kHz

	timerx->ARR = 	1000-1		;	// Set auto reload register to maximum (count up to 65535)
	timerx->DIER |=   1<<0;                  	// Enable Interrupt
	timerx->CR1 |=    1<<0;           	// Enable counter
	
	NVIC_SetPriority(TIM2_IRQn, 2);               	// TIM2_IRQHandler Set priority as 2
	NVIC_EnableIRQ(TIM2_IRQn);			// TIM2_IRQHandler Enable
	
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){}
}

// Initialiization 
void setup(void)
{	
	RCC_PLL_init();                       // System Clock = 84MHz

	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()	
}

void TIM2_IRQHandler(void){
		static uint8_t blinkToggle = 0; 
	if((TIM2->SR & TIM_SR_UIF) == TIM_SR_UIF     ){ // update interrupt flag
		//Create the code to toggle LED by 1000ms
		GPIOA->ODR ^= (1 << 5);
		TIM2->SR &=   TIM2->SR &= ~TIM_SR_UIF;                  // clear by writing 0
	}
}