/**
******************************************************************************
* @author  Kim Young-Min 	
* @brief   Embedded Controller:  LAB_GPIO_ 7-segment Display
* 
******************************************************************************
*/

#include "..\..\include\ecGPIO.h"
#include "stm32f4xx.h"
#include "..\..\include\ecRCC.h"

unsigned int cnt = 0;
int buttonState = 0;
int lastButtonState = 0;

void setup(void);
void delay(uint32_t ms);
void updateDisplay(void);

int main(void) {
    setup();
    while (1) {
        buttonState = GPIO_read(GPIOC, BUTTON_PIN);
        if (buttonState == 0 && lastButtonState == 1) {
             cnt++;
            if (cnt > 9) {
                cnt = 0;
            }
        }
        lastButtonState = buttonState;
        updateDisplay();
    }
}

void setup(void) {
    RCC_HSI_init();
    
    GPIO_init(GPIOC, BUTTON_PIN, 0UL);
    GPIO_pupd(GPIOC, BUTTON_PIN, 1UL);

    sevensegment_init(GPIOA, SEG_A_PIN, 1UL);
    sevensegment_init(GPIOB, SEG_B_PIN, 1UL);
    sevensegment_init(GPIOC, SEG_C_PIN, 1UL);
    sevensegment_init(GPIOA, SEG_D_PIN, 1UL);
    
}

void updateDisplay(void) {
    sevensegment_decoder(cnt);
}
