#include "..\..\lib\ecGPIO.h"

#define BUTTON_PIN 13
#define LED_PIN1 12
#define LED_PIN2 11
#define LED_PIN3 10
#define LED_PIN4 9 // D9 ? ?? (LD4)

int ledState = 0; // ?? ??: LED_PIN1?? ??

void setup(void);

int main(void) {
    setup();

    while (1) {
        if (GPIO_read(GPIOC, BUTTON_PIN) == 0) {
            if (ledState == 0) {
                GPIO_write(GPIOA, LED_PIN1, 1);
                GPIO_write(GPIOA, LED_PIN2, 0);
                GPIO_write(GPIOA, LED_PIN3, 0);
                GPIO_write(GPIOB, LED_PIN4, 0); // LD4 (D9) ??
                ledState = 1;
            } else if (ledState == 1) {
                GPIO_write(GPIOA, LED_PIN1, 0);
                GPIO_write(GPIOA, LED_PIN2, 1);
                GPIO_write(GPIOA, LED_PIN3, 0);
                GPIO_write(GPIOB, LED_PIN4, 0); // LD4 (D9) ??
                ledState = 2;
            } else if (ledState == 2) {
                GPIO_write(GPIOA, LED_PIN1, 0);
                GPIO_write(GPIOA, LED_PIN2, 0);
                GPIO_write(GPIOA, LED_PIN3, 1);
                GPIO_write(GPIOB, LED_PIN4, 0); // LD4 (D9) ??
                ledState = 3;
            } else if (ledState == 3) {
                GPIO_write(GPIOA, LED_PIN1, 0);
                GPIO_write(GPIOA, LED_PIN2, 0);
                GPIO_write(GPIOA, LED_PIN3, 0);
                GPIO_write(GPIOB, LED_PIN4, 1); // LD4 (D9) ??
                ledState = 0;
            }

            while (GPIO_read(GPIOC, BUTTON_PIN) == 0); // ?? ??? ??
        }
    }
}

void setup(void) {
    RCC_HSI_init();
    GPIO_init(GPIOC, BUTTON_PIN, INPUT);
    GPIO_init(GPIOA, LED_PIN1, OUTPUT);
    GPIO_init(GPIOA, LED_PIN2, OUTPUT);
    GPIO_init(GPIOA, LED_PIN3, OUTPUT);
    GPIO_init(GPIOB, LED_PIN4, OUTPUT);

    GPIO_otype(GPIOA, LED_PIN1, 1); // Push-Pull ?? ??
    GPIO_otype(GPIOA, LED_PIN2, 1);
    GPIO_otype(GPIOA, LED_PIN3, 1);
    GPIO_otype(GPIOB, LED_PIN4, 1);

    GPIO_ospeed(GPIOA, LED_PIN1, 2); // Medium Speed ??
    GPIO_ospeed(GPIOA, LED_PIN2, 2);
    GPIO_ospeed(GPIOA, LED_PIN3, 2);
    GPIO_ospeed(GPIOB, LED_PIN4, 2);

    GPIO_pupd(GPIOC, BUTTON_PIN, 1); // Pull-up ??
}
