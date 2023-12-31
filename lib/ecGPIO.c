/*----------------------------------------------------------------\
@ Embedded Controller by Youngmin - Handong Global University


Language/ver     : C++ in Keil uVision

Description      : For RCC
/----------------------------------------------------------------*/



#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecSysTick.h"

volatile unsigned int cnt = 0;

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
	
	//[TO-DO] YOUR CODE GOES HERE
	// Make it for GPIOB, GPIOD..GPIOH

	// You can also make a more general function of
	// void RCC_GPIO_enable(GPIO_TypeDef *Port); 

	GPIO_mode(Port, pin, mode);
	
}


// GPIO Mode          : Input(00), Output(01), AlterFunc(10), Analog(11)
void GPIO_mode(GPIO_TypeDef *Port, int pin, int mode){
   Port->MODER &= ~(3UL<<(2*pin));     
 Port->MODER |= (unsigned int)mode<<(2*pin);  

	
}


// GPIO Speed          : Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
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
void sevensegment_init(GPIO_TypeDef *Port, int pin, int Output){
	 GPIO_init(Port, pin, Output);
}

void updateDisplay(void) {
	
    sevensegment_decoder(cnt);
}

void sevensegment_decoder(unsigned int num) {
    
    if (num == 0) {
    GPIO_write(GPIOA, SEG_A_PIN, LOW);
    GPIO_write(GPIOB, SEG_B_PIN, LOW);
    GPIO_write(GPIOC, SEG_C_PIN, LOW);
    GPIO_write(GPIOA, SEG_D_PIN, LOW);
        
    } else if (num == 1) {
    GPIO_write(GPIOA, SEG_A_PIN, HIGH);
    GPIO_write(GPIOB, SEG_B_PIN, LOW);
    GPIO_write(GPIOC, SEG_C_PIN, LOW);
    GPIO_write(GPIOA, SEG_D_PIN, LOW);
        
    } else if (num == 2) {
    GPIO_write(GPIOA, SEG_A_PIN, LOW);
    GPIO_write(GPIOB, SEG_B_PIN, HIGH);
    GPIO_write(GPIOC, SEG_C_PIN, LOW);
    GPIO_write(GPIOA, SEG_D_PIN, LOW);
       
    } else if (num == 3) {
    GPIO_write(GPIOA, SEG_A_PIN, HIGH);
    GPIO_write(GPIOB, SEG_B_PIN, HIGH);
    GPIO_write(GPIOC, SEG_C_PIN, LOW);
    GPIO_write(GPIOA, SEG_D_PIN, LOW);
   
		} else if (num == 4) {
    GPIO_write(GPIOA, SEG_A_PIN, LOW);
    GPIO_write(GPIOB, SEG_B_PIN, LOW);
    GPIO_write(GPIOC, SEG_C_PIN, HIGH);
    GPIO_write(GPIOA, SEG_D_PIN, LOW);
    
		} else if (num == 5) {
    GPIO_write(GPIOA, SEG_A_PIN, HIGH);
    GPIO_write(GPIOB, SEG_B_PIN, LOW);
    GPIO_write(GPIOC, SEG_C_PIN, HIGH);
    GPIO_write(GPIOA, SEG_D_PIN, LOW);
    
		} else if (num == 6) {
    GPIO_write(GPIOA, SEG_A_PIN, LOW);
    GPIO_write(GPIOB, SEG_B_PIN, HIGH);
    GPIO_write(GPIOC, SEG_C_PIN, HIGH);
    GPIO_write(GPIOA, SEG_D_PIN, LOW);
    } else if (num == 7) {
     GPIO_write(GPIOA, SEG_A_PIN, HIGH);
    GPIO_write(GPIOB, SEG_B_PIN, HIGH);
    GPIO_write(GPIOC, SEG_C_PIN, HIGH);
    GPIO_write(GPIOA, SEG_D_PIN, LOW);
        
    } else if (num == 8) {
    GPIO_write(GPIOA, SEG_A_PIN, LOW);
    GPIO_write(GPIOB, SEG_B_PIN, LOW);
    GPIO_write(GPIOC, SEG_C_PIN, LOW);
    GPIO_write(GPIOA, SEG_D_PIN, HIGH);
        
    } else if (num == 9) {
    GPIO_write(GPIOA, SEG_A_PIN, HIGH);
    GPIO_write(GPIOB, SEG_B_PIN, LOW);
    GPIO_write(GPIOC, SEG_C_PIN, LOW);
    GPIO_write(GPIOA, SEG_D_PIN, HIGH);
        
    }
}

void LED_toggle(void) {
    GPIOA->ODR ^= (1 << 5);
}


