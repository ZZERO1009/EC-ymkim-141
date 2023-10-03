#include "..\..\include\ecGPIO.h"

#define BUTTON_PIN 13
#define LED_PIN 5
#define LED_PIN1 6


int ledState = 0;

void setup(void);

int main(void) {
    setup();

    while (1) {
        if (GPIO_read(GPIOC, BUTTON_PIN) == 0) {
            if (ledState == 0) {
                GPIO_write(GPIOA, LED_PIN, 1);
                GPIO_write(GPIOB, LED_PIN1, 0);
           
                ledState = 1;
            } else if (ledState == 1) {
                GPIO_write(GPIOA, LED_PIN, 0);
                GPIO_write(GPIOB, LED_PIN1, 1);
                ledState = 0;
            } 

            while (GPIO_read(GPIOC, BUTTON_PIN) == 0);
        }
    }
}

void setup(void) {
    RCC_HSI_init();
    GPIO_init(GPIOC, BUTTON_PIN, INPUT);
    GPIO_init(GPIOA, LED_PIN, OUTPUT);
    GPIO_init(GPIOB, LED_PIN1, OUTPUT);
    

    GPIO_otype(GPIOA, LED_PIN, 0);
    GPIO_otype(GPIOB, LED_PIN1, 0);

    GPIO_ospeed(GPIOA, LED_PIN, 2);
    GPIO_ospeed(GPIOB, LED_PIN1, 2);
    GPIO_pupd(GPIOC, BUTTON_PIN, 1);
}
