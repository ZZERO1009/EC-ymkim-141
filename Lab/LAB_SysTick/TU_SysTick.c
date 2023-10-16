#include "stm32f411xe.h"
#include "..\..\lib\ecRCC.h"
#include "..\..\lib\ecGPIO.h"
#include "..\..\lib\ecSysTick.h"

#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 84000000

volatile uint32_t msTick = 0;
volatile uint32_t curTicks;
volatile uint32_t msDelay=1000;

void setup(void);

int main(void) {
	
// System CLOCK, GPIO Initialiization ----------------------------------------
	setup();

// SysTick Initialiization ------------------------------------------------------				
	//  SysTick Control and Status Register
	SysTick->CTRL = 0;				// Disable SysTick IRQ and SysTick Counter

	// Select processor clock
	// 1 = processor clock;  0 = external clock
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;


	// uint32_t MCU_CLK=EC_SYSTEM_CLK
	// SysTick Reload Value Register
	SysTick->LOAD =MCU_CLK_PLL / 1000 - 1; // 1ms

	// Clear SysTick Current Value 
	SysTick->VAL = 0;

	// Enables SysTick exception request
	// 1 = counting down to zero asserts the SysTick exception request
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

		
	// Enable SysTick IRQ and SysTick Timer
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

		
	NVIC_SetPriority(SysTick_IRQn, 16);		// Set Priority to 1
	NVIC_EnableIRQ(SysTick_IRQn);			// Enable interrupt in NVIC

		
// While loop ------------------------------------------------------				
	msTick = 0;
	while(1){		
	// Method 1		
		curTicks = msTick;
		while ((msTick - curTicks) < 1000);
		GPIOA->ODR ^= (1 << 5);
		msTick = 0;
}
}
void SysTick_Handler(void){
	msTick++;
	msDelay--;
}

void setup(void)
{
	RCC_PLL_init();                         // System Clock = 84MHz	
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()	
}


