/**
******************************************************************************
* @author  Young-Min Kim
* @brief   Embedded Controller:  LAB_GPIO_ 7-segment Display_without_decoder
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
void updateDisplay(void);

int main(void) {
    setup();
    
    while (1) {
        buttonState = GPIO_read(GPIOC, BUTTON_PIN);
        if (buttonState != lastButtonState) {
            if (buttonState == LOW) {
                cnt++;
                if (cnt > 9) {
                    cnt = 0;
                }
            }
            lastButtonState = buttonState;
        }
        updateDisplay();
    }
}

void setup(void) {
    RCC_HSI_init();
		GPIO_init(GPIOC, BUTTON_PIN, 0UL);
    GPIO_pupd(GPIOC, BUTTON_PIN, 2UL);
    sevensegment_init(GPIOA, SEG_A_PIN, 1UL);
    sevensegment_init(GPIOA, SEG_B_PIN, 1UL);
    sevensegment_init(GPIOA, SEG_C_PIN, 1UL);
    sevensegment_init(GPIOB, SEG_D_PIN, 1UL);
    sevensegment_init(GPIOC, SEG_E_PIN, 1UL);
    sevensegment_init(GPIOA, SEG_F_PIN, 1UL);
    sevensegment_init(GPIOA, SEG_G_PIN, 1UL);
    sevensegment_init(GPIOB, SEG_H_PIN, 1UL);
}

void updateDisplay(void) {
    sevensegment_decoder(cnt);
}

