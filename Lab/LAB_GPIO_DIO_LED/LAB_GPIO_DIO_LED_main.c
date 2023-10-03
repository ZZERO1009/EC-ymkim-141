#include "..\..\include\ecGPIO.h"

#define BUTTON_PIN 13
#define LED_PIN 5

void setup(void);

int main(void) {
    setup();

    while (1) {
        if (GPIO_read(GPIOC, BUTTON_PIN) == 0) {
            if (GPIO_read(GPIOA, LED_PIN) == 1UL) {
                GPIO_write(GPIOA, LED_PIN, 0UL);
            } else {
                GPIO_write(GPIOA, LED_PIN, 1UL);
            }

            while (GPIO_read(GPIOC, BUTTON_PIN) == 0);
        }
    }
}

void setup(void) {
    RCC_HSI_init();
    GPIO_init(GPIOC, BUTTON_PIN, INPUT);
    GPIO_init(GPIOA, LED_PIN, OUTPUT);
    GPIO_otype(GPIOA, LED_PIN, 1UL);
    GPIO_ospeed(GPIOA, LED_PIN, 2UL);
    GPIO_pupd(GPIOC, BUTTON_PIN, 1UL);
}
