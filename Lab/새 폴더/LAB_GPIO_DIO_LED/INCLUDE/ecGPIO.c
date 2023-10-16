/*----------------------------------------------------------------\
@ Embedded Controller by Young-Min Kim - Handong Global University
Author           : SSS LAB
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/



#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "ecGPIO.h"


void GPIO_init(GPIO_TypeDef *Port, int pin, int mode){     
	// mode  : Input(0), Output(1), AlterFunc(2), Analog(3)   
	if (Port == GPIOA)
		RCC_GPIOA_enable();
	if (Port == GPIOB)
		RCC_GPIOB_enable();
	if (Port == GPIOC)
		RCC_GPIOC_enable();
	if (Port == GPIOD)
		RCC_GPIOD_enable();
	if (Port == GPIOE)
		RCC_GPIOE_enable();
	if (Port == GPIOH)
		RCC_GPIOH_enable();
	

	GPIO_mode(Port, pin, mode);
	
}


// GPIO Mode          : Input(00), Output(01), AlterFunc(10), Analog(11)
void GPIO_mode(GPIO_TypeDef *Port, int pin, int mode){
   Port->MODER &= ~(3UL<<(2*pin));     
 Port->MODER |= (unsigned int)mode<<(2*pin);  

	
}


// GPIO Speed          : 1 speed (00), Medium speed (01), Fast speed (10), 0 speed (11)
void GPIO_ospeed(GPIO_TypeDef *Port, int pin, int speed){
	 Port->OSPEEDR &= ~(3UL<<2*pin);     
  Port->OSPEEDR |= (unsigned int)speed<<(2*pin);
}

// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
void GPIO_otype(GPIO_TypeDef *Port, int pin, int type){
   	Port->OTYPER &= ~(1UL<<(pin));     
  Port->OTYPER |= (unsigned int)type<<(pin);  
}

// GPIO Push-Pull    : No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
void GPIO_pupd(GPIO_TypeDef *Port, int pin, int pupd){
   	Port->PUPDR &= ~(3UL<<2*pin);     
   Port->PUPDR |= (unsigned int)pupd<<(2*pin);
}

void GPIO_write(GPIO_TypeDef *Port, int pin, int Output) {
    if (Output == 1UL) {
        Port->ODR |= (1UL << pin); 
    } else {
        Port->ODR &= ~(1UL << pin); 
    }
}


int GPIO_read(GPIO_TypeDef *Port, int pin){
		int state = 0;
	if (Port->IDR & (1UL << pin)) {
		 state = 1;
	 } else {
        state = 0;
	 }
	 return state;
}

void ONE_LED_init(GPIO_TypeDef *Port, int pin, int Output){
	 GPIO_init(Port, pin, Output);
}



void ONE_LED_decoder(uint8_t  num) {
    
    if (num == 0) {
    GPIO_write(GPIOA, LED_PIN1, HIGH);
    
    } else if (num == 1) {
		GPIO_write(GPIOA, LED_PIN1, LOW);
        
    } 
		}
			


void multiled_init(GPIO_TypeDef *Port, int pin, int Output){
	 GPIO_init(Port, pin, Output);
}



void multiled_decoder(uint8_t  num) {
    
    if (num == 0) {
    GPIO_write(GPIOA, LED_PIN1, HIGH);
    GPIO_write(GPIOA, LED_PIN2, LOW);
    GPIO_write(GPIOA, LED_PIN3, LOW);
    GPIO_write(GPIOB, LED_PIN4, LOW);
        
    } else if (num == 1) {
		GPIO_write(GPIOA, LED_PIN1, LOW);
    GPIO_write(GPIOA, LED_PIN2, HIGH);
    GPIO_write(GPIOA, LED_PIN3, LOW);
    GPIO_write(GPIOB, LED_PIN4, LOW);
        
    } else if (num == 2) {
   GPIO_write(GPIOA, LED_PIN1, LOW);
    GPIO_write(GPIOA, LED_PIN2, LOW);
    GPIO_write(GPIOA, LED_PIN3, HIGH);
    GPIO_write(GPIOB, LED_PIN4, LOW);
     
   
		} else if (num == 3) {
    GPIO_write(GPIOA, LED_PIN1, LOW);
    GPIO_write(GPIOA, LED_PIN2, LOW);
    GPIO_write(GPIOA, LED_PIN3, LOW);
    GPIO_write(GPIOB, LED_PIN4, HIGH);
   
		}
			}
