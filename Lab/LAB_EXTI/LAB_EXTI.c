#include "..\..\lib\ecRCC.h"
#include "..\..\lib\ecGPIO.h"
#include "..\..\lib\ecEXTI.h"



void setup(void);



#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 84000000


void setup(void);

int main(void) {
    setup();
    
    while (1) {
       
        updateDisplay();
			
    }
}






// Initialiization 
void setup(void)
{
	
	RCC_PLL_init();                         // System Clock = 84MHz
	// Initialize GPIOA_5 for Output
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_ospeed(GPIOA, LED_PIN, 1UL);
	// Initialize GPIOC_13 for Input Button
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	  sevensegment_init(GPIOA, SEG_A_PIN, 1UL);
    sevensegment_init(GPIOB, SEG_B_PIN, 1UL);
    sevensegment_init(GPIOC, SEG_C_PIN, 1UL);
    sevensegment_init(GPIOA, SEG_D_PIN, 1UL);
	  EXTI_enable(13);
		EXTI_init(GPIOC, BUTTON_PIN, FALL,13);
		
}

  