/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : 21900141 Young-Min Kim

Language/ver     : C++ in Keil uVision

Description      : MAIN code of LAB: SysTick
/----------------------------------------------------------------*/

#include "stm32f411xe.h"
#include "..\..\lib\ecRCC.h"
#include "..\..\lib\ecGPIO.h"

#include "..\..\lib\ecSysTick.h"

#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 84000000
 
volatile uint32_t msDelay = 1000;

void setup(void);

int main(void) {
	
    // System CLOCK, GPIO Initialization ----------------------------------------
    setup();

    // SysTick Initialization is now in ecSysTick.h
    SysTick_init(); // Initialize SysTick timer as defined in ecSysTick.h
	

    NVIC_SetPriority(SysTick_IRQn, 16);       // Set SysTick interrupt priority to 16
    NVIC_EnableIRQ(SysTick_IRQn);            // Enable SysTick interrupt

    // While loop ------------------------------------------------------
  int buttonPressed = 0; // button defalt is low 

    while (1) {
        int buttonState = GPIO_read(GPIOC, BUTTON_PIN); //read button input

        if (buttonState == LOW || buttonPressed == 1) { 
            cnt = 0; // when button is pressed and previous is low, reset cnt
					 updateDisplay();
            buttonPressed = 0;
        } else {
            buttonPressed = 0; //if other, defalt
        }

        if (msTicks >= msDelay) { //time goes on, cnt is updated
            cnt++;
            if (cnt > 9) {
                cnt = 0;
            }
            updateDisplay();
            msTicks = 0;
        }
    }
}

void setup(void) {
    RCC_PLL_init(); // Initialize the PLL for system clock

    
    GPIO_init(GPIOA, LED_PIN, OUTPUT);
    GPIO_ospeed(GPIOA, LED_PIN, 1UL);
    GPIO_init(GPIOC, BUTTON_PIN, INPUT);
    sevensegment_init(GPIOA, SEG_A_PIN, 1UL);
    sevensegment_init(GPIOB, SEG_B_PIN, 1UL);
    sevensegment_init(GPIOC, SEG_C_PIN, 1UL);
    sevensegment_init(GPIOA, SEG_D_PIN, 1UL);
		
}

