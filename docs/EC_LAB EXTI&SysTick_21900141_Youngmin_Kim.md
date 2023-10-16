# LAB: EXTI & SysTick

**Date: 2023.10.15.

**Author/Partner:** 21900141 Youngmin Kim

**Github**: <https://github.com/ZZERO1009/EC-ymkim-141/blob/main/docs/EC_LAB_led_21900141_Youngmin_Kim.md>

**Demo video link**:


## Introduction

In this lab, required to create two simple programs using interrupt:

(1) displaying the number counting from 0 to 9 with Button Press

(2) counting at a rate of 1 second

### Requirement

#### Hardware 

* MCU
  * NUCLEO-F401RE


* Actuator/Sensor/Others:
  * 7-segment display(5101ASR)
  * Array resistor (330 ohm)
  * breadboard

#### Software
 * Keil uVision, CMSIS, EC_HAL library



## Problem 1 : Counting numbers on 7-Segment using EXTI Button

#### 1-1. Create HAL library
For LAB, have to create the library directory and own library, and declare and define functions in library : **ecEXTI.h**

##### ecEXTI.h

```c++
void EXTI_init(GPIO_TypeDef *Port, int pin, int trig, int priority);
void EXTI_enable(uint32_t pin);
void EXTI_disable(uint32_t pin);
uint32_t is_pending_EXTI(uint32_t pin);
void clear_pending_EXTI(uint32_t pin);
void EXTI15_10_IRQHandler(void); //This code is for handle dispay function
```

Define This functions on EXTI.c


##### ecEXTI.c

```c++
void EXTI_init(GPIO_TypeDef *Port, int Pin, int trig_type,int priority){

	// SYSCFG peripheral clock enable	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;		
	
	// Connect External Line to the GPIO
	int EXTICR_port;
	if			(Port == GPIOA) EXTICR_port = 0;
	else if	(Port == GPIOB) EXTICR_port = 1;
	else if	(Port == GPIOC) EXTICR_port = 2;
	else if	(Port == GPIOD) EXTICR_port = 3;
	else 										EXTICR_port = 4;
	
	 SYSCFG->EXTICR[Pin >> 2] &= ~(0xF << ((Pin % 4) * 4)); // Clear 4 bits
   SYSCFG->EXTICR[Pin >> 2] |= (EXTICR_port << ((Pin % 4) * 4)); // Set 4 bits

	// Configure Trigger edge
	if (trig_type == FALL) EXTI->FTSR |= (1 << Pin);   // Falling trigger enable 
	else if	(trig_type == RISE) EXTI->RTSR |= (1 << Pin);   // Rising trigger enable 
	else if	(trig_type == BOTH) {			// Both falling/rising trigger enable
		EXTI->RTSR |= (1 << Pin); 
		EXTI->FTSR |= (1 << Pin);
	} 
	
	// Configure Interrupt Mask (Interrupt enabled)
	EXTI->IMR  |=  (1 << Pin);     // not masked
	
	
	// NVIC(IRQ) Setting
	int EXTI_IRQn = 0;
	
	if (Pin < 5) 	EXTI_IRQn = EXTI0_IRQn + Pin;
	else if	(Pin < 10) 	EXTI_IRQn = EXTI9_5_IRQn;
	else 			EXTI_IRQn = EXTI15_10_IRQn;
								
	NVIC_SetPriority(EXTI_IRQn, priority);	// EXTI priority
	NVIC_EnableIRQ(EXTI_IRQn); 	// EXTI IRQ enable
}


void EXTI_enable(uint32_t pin) {
	 EXTI->IMR |= (1 << pin);     // not masked (i.e., Interrupt enabled)
}
void EXTI_disable(uint32_t pin) {
	EXTI->IMR &= ~(1 << pin);     // masked (i.e., Interrupt disabled)
}

uint32_t is_pending_EXTI(uint32_t pin){
	uint32_t EXTI_PRx = 1 << pin;     	// check  EXTI pending 	
	 return ((EXTI->PR & EXTI_PRx) == EXTI_PRx);
}

void clear_pending_EXTI(uint32_t pin){
	EXTI->PR = (1 << pin); // clear EXTI pending 
}

void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR13) {
        // Check EXTI line 13 (interrupt source) has pending interrupt flag set.
        cnt++; 
        // Increment the 'cnt' variable. 

        if (cnt > 9) {
            cnt = 0;
        }
        // If 'cnt' exceeds 9, reset it to 0.
        EXTI->PR |= EXTI_PR_PR13;
        // Clear the EXTI line 13 pending interrupt flag. This line is processed.
    }
}

```



#### 1-2. Procedure

Create the LAB_EXTI project and Include library above, and  create code on project  that satisfies the configuration table.

#### Configuration 

| Digital In for Button (B1) |       Digital Out for 7-Segment decoder       |
| :------------------------: | :-------------------------------------------: |
|         Digital In         |                  Digital Out                  |
|            PC13            |              PA7, PB6, PC7, PA9               |
|          PULL-UP           | Push-Pull, No Pull-up-Pull-down, Medium Speed |

#### Circuit Diagram

![](C:\Users\Zzero1009\사진\스크린샷\스크린샷 2023-10-08 000335.png)



#### Discussion

1. **We can use two different methods to detect an external signal: polling and interrupt. What are the advantages and disadvantages of each approach?**

   Polling:

   ​	advantage:

   ​		Requires fewer resources compared to interrupt handling, making it suitable for simple applications.

   ​	disadvantage:

   ​		Slower response time as external signals need to be periodically checked, making it unsuitable for real-time 		applications where responsiveness is crucial.

​	Polling:

​		advantage:

​			Requires fewer resources compared to interrupt handling, making it suitable for simple applications.

​		disadvantage:

​			Slower response time as external signals need to be periodically checked, making it unsuitable for real-time 			applications where responsiveness is crucial.





#### Main code

```c++
#include "..\..\lib\ecRCC.h"
#include "..\..\lib\ecGPIO.h"
#include "..\..\lib\ecEXTI.h"

void setup(void);
	//Define pll and hsi to 84MHz
#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 84000000


void setup(void);

int main(void) {
    setup();
    
    while (1) {
       			// EXTI15_10_IRQHandler will go on
        updateDisplay();
			
    }
}

// Initialiization 
void setup(void)
{
	
	RCC_PLL_init();                       
	// Initialize GPIOA_5 for Output
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_ospeed(GPIOA, LED_PIN, 1UL);
	// Initialize GPIOC_13 for Input Button
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
    	// Part for 7seg
	  sevensegment_init(GPIOA, SEG_A_PIN, 1UL);
    sevensegment_init(GPIOB, SEG_B_PIN, 1UL);
    sevensegment_init(GPIOC, SEG_C_PIN, 1UL);
    sevensegment_init(GPIOA, SEG_D_PIN, 1UL);
	  EXTI_enable(13);				// Enable EXTI for interrupt handling on line 13 (corresponding to GPIOC_13)	
		EXTI_init(GPIOC, BUTTON_PIN, FALL,13); // Initialize EXTI to detect falling edge on GPIOC_13
		
}

   
```



### Result

https://youtu.be/oPWBX6mxVtI?si=91R1qfn8IzEs2_rF

## Problem 2: Counting numbers on 7-Segment using SysTick

Display the number 0 to 9 on the 7-segment LED at the rate of 1 sec. After displaying up to 9, then it should display ‘0’ and continue counting.

When the button is pressed, the number should be reset ‘0’ and start counting again.

#### 2-1. Create HAL library

For LAB, have to create the library directory and own library, and declare and define functions in library : **ecSysTick.h**

##### ecSysTick.h

```c++
extern volatile uint32_t msTicks;
void SysTick_init(void);
void SysTick_Handler(void);
void SysTick_counter(void);
void delay_ms(uint32_t msec);
void SysTick_reset(void);
uint32_t SysTick_val(void);
void SysTick_enable(void);
void SysTick_disable (void);
```

Define This functions on EXTI.c

```c++
#include "ecSysTick.h"

volatile uint32_t msTicks = 0;

// Initialize and configure the SysTick timer
void SysTick_init(void) {
    // Disable SysTick IRQ and SysTick Counter
    SysTick->CTRL = 0;
    
    // Select the processor clock as the clock source
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

    // Set the reload value for 1ms with a 84MHz processor clock (HSI PLL)
    SysTick->LOAD = 84000 - 1;

    // Initialize the current value
    SysTick->VAL = 0;

    // Enable SysTick exception request
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

    // Enable SysTick IRQ and SysTick Timer
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    // Set the priority to 1 and enable the SysTick interrupt in NVIC
    NVIC_SetPriority(SysTick_IRQn, 1);
    NVIC_EnableIRQ(SysTick_IRQn);
}

// SysTick interrupt handler
void SysTick_Handler(void) {
    SysTick_counter();
}

// Increment the millisecond counter
void SysTick_counter() {
    msTicks++;
}

// Delay for a specified number of milliseconds
void delay_ms(uint32_t mesc) {
    uint32_t curTicks;

    curTicks = msTicks;
    while ((msTicks - curTicks) < mesc);

    msTicks = 0;  // Reset the millisecond counter
}

// Reset the SysTick timer
void SysTick_reset(void) {
    SysTick->VAL = 0;
}

// Get the current value of the SysTick timer
uint32_t SysTick_val(void) {
    return SysTick->VAL;
}

// Enable the SysTick timer
void SysTick_enable(void) {
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

// Disable the SysTick timer
void SysTick_disable(void) {
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}
```



#### 2-2. Procedure

Create the LAB_SysTick project and Include library above, and  create code on project  that satisfies the configuration table.

#### Configuration 

| Digital In for Button (B1) |       Digital Out for 7-Segment decoder       |
| :------------------------: | :-------------------------------------------: |
|         Digital In         |                  Digital Out                  |
|            PC13            |              PA7, PB6, PC7, PA9               |
|          PULL-UP           | Push-Pull, No Pull-up-Pull-down, Medium Speed |



#### Circuit Diagram

![](C:\Users\Zzero1009\사진\스크린샷\스크린샷 2023-10-08 000335.png)

 #### Main code

 ```c++
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


 ```



### Result

https://youtu.be/1-ufuONGUbk?si=F_WSb04-OdtuS5uF
