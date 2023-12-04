---
description: EC_HAL API
---

# Embedded Controller HAL

Written by:   김영민

Course:  임베디드컨트롤러



Program: 		C/C++

IDE/Compiler: Keil uVision 5

OS: 					WIn11

MCU:  				STM32F411RE, Nucleo-64





## GPIO Digital In/Out 

### Header File

 `#include "ecGPIO.h"`


```c++
#include "stm32f411xe.h"
#include "ecRCC.h"

#ifndef __ECGPIO_H
#define __ECGPIO_H

#define INPUT  0x00
#define OUTPUT 0x01
#define AF     0x02
#define ANALOG 0x03

#define HIGH 1
#define LOW  0

#define LED_PIN 	5
#define BUTTON_PIN 13
#define SEG_A_PIN 7
#define SEG_B_PIN 6
#define SEG_C_PIN 7
#define SEG_D_PIN 9


#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output);
int  GPIO_read(GPIO_TypeDef *Port, int pin);
void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode);
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
void GPIO_pupd(GPIO_TypeDef* Port, int pin, int pupd);
void sevensegment_init(GPIO_TypeDef *Port, int pin, int Output); 
void sevensegment_decoder(unsigned int num);
void updateDisplay(void);
void LED_toggle(void);
extern volatile unsigned int cnt;


 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```




### GPIO_init\(\)

Initializes GPIO pins with default setting and Enables GPIO Clock. Mode: In/Out/AF/Analog

```c++
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH

* **pin**:  pin number (int) 0~15

* **mode**:   INPUT (0), OUTPUT (1),  AF(02), ANALOG (03)

  

**Example code**

```c++
GPIO_init(GPIOA, 5, OUTPUT);
GPIO_init(GPIOC, 13, INPUT); //GPIO_init(GPIOC, 13, 0);
```



### GPIO_mode\(\)

Configures  GPIO pin modes: In/Out/AF/Analog

```c++
void GPIO_mode(GPIO_TypeDef *Port, int pin, int mode);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH

* **pin**:  pin number (int) 0~15

* **mode**:   INPUT (0), OUTPUT (1),  AF(02), ANALOG (03)

  

**Example code**

```c++
GPIO_mode(GPIOA, 5, OUTPUT); //set pin5 output mode
```



### GPIO_pupdr\(\)

Configure the I/O pull-up or pull-down: No PullupPulldown/ Pull-Up/Pull-Down/Analog/Reserved

```c++
void GPIO_pupdr(GPIO_TypeDef* Port, int pin, int pupd);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15
* **pupd**:   No PullupPulldown (0),  Pull-Up(1), Pull-Down(2) , Reserved(3)



**Example code**

```c++
GPIO_pupdr(GPIOA, 5, 0);  // 0: No PUPD pin5
```



### GPIO_ospeed\(\)

Configure the I/O output speed: Low speed/Medium speed/Fast speed/High speed

```c++
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15
* **speed**:  Low speed(0), Medium speed(1), Fast speed(2), High speed(3)



**Example code**

```c++
GPIO_ospeed(GPIOA, 5, 3);  // 3: Fast speed pin5
```



### GPIO_otype\(\)

Configure the output type of the I/O port:  Output push-pull/Output open-drain

```c++
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15
* **type**:   Output push-pull(0)/Output open-drain(1)



**Example code**

```c++
GPIO_otype(GPIOA, 5, 0);  // 0: push-pull
```



### GPIO_read\(\)

Receive the input signal 

```c++
int GPIO_read(GPIO_TypeDef* Port, int pin);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15



**Example code**

```c++
GPIO_read(GPIOA, 13);  // read signal of GPIOA pin13
```



### GPIO_write(\)

Configures output of on/off: LOW/HIGH

```c++
void GPIO_write(GPIO_TypeDef* Port, int pin, int output);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15
* **output**:   LOW(0), HIGH(1)



**Example code**

```c++
GPIO_output(GPIOA, 5, 0);  // 0: LOW 
```



### sevensegment_init(\)

Initializes 7-segment  8pins

```c++
void sevensegment_init();
```

This function includes others functions 

ex) GPIO_init, GPIO_pudr, GPIO_otype, GPIO_ospeed

so, it define register's state

**Example code**

```c++
void sevensegment_init(); // setting registers
```



### sevensegment_decode(\)

According to Input signal, change the 7-segment display

```c++
void sevensegment_decode(int number);
```

**Parameters**

* number: the number shown on the display (0~9)

**Example code**

```c++
void sevensegment_init(5); // Appear to number 5 on the 7-segment
```



### LED_toggle (\)

Function to set LED toggle

```c++
void LED_toggle(void)
```

**Example code**

```c++
LED_toggle()
```



### updateDisplay()

Update display of sevenseg

```c++
void updateDisplay(void)
```

**Example code**

```c++
updateDisplay()
```



------

## Interrupt EXTI 

### Header File

 `#include "ecEXTI.h"`


```c++
#ifndef __EC_EXTI_H
#define __EC_EXTI_H

#include "stm32f411xe.h"

#define FALL 0
#define RISE 1
#define BOTH 2

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

void EXTI_init(GPIO_TypeDef *Port, int pin, int trig, int priority);
void EXTI_enable(uint32_t pin);
void EXTI_disable(uint32_t pin);
uint32_t is_pending_EXTI(uint32_t pin);
void clear_pending_EXTI(uint32_t pin);
void EXTI15_10_IRQHandler(void);


#ifdef __cplusplus
}
#endif /* __cplusplus */
	 
```



### EXTI_init\(\)

Initializes EXTI pins with default setting. 

```c++
void EXTI_init (GPIO_TypeDef *Port, uint32_t pin, int edge , int prior);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15
* **edge**: FALL(0), RISE (1), BOTH(2)
* **prior**: prior number. The smaller the number, the more priority

**Example code**

```c++
EXTI_init(GPIOC, 13, FALL, 0); //When falling edge, Pin 13 of GPIOC port does interrupt   
```



### EXTI_enable\(\)

Enable the EXTI that fits the pin number.

```c++
void EXTI_enable(uint32_t pin);
```

**Parameters**

* **Pin:**  pin number (int) 0~15

**Example code**

```c++
EXTI_enable(13); //EXTI13 is enable
```



### EXTI_disable\(\)

Disable the EXTI that fits the pin number.

```c++
void EXTI_disable(uint32_t pin);
```

**Parameters**

* **Pin:**  pin number (int) 0~15

**Example code**

```c++
EXTI_disable(13); //EXTI13 is disable
```



### is_pending_EXTI\(\)

Check the pending is ON

```c++
uint32_t is_pending_EXTI(uint32_t pin);
```

**Parameters**

* **Pin:**  pin number (int) 0~15

**Example code**

```c++
is_pending_EXTI(13); // If Pending of pin 13 is ON, retrun 1. 
```



### clear_pending_EXTI\(\)

Clear the pending's value

```c++
void clear_pending_EXTI(uint32_t pin);
```

**Parameters**

* **Pin:**  pin number (int) 0~15

**Example code**

```c++
clear_pending_EXTI(13); //clear pending of pin 13
```



### EXTIx_IRQHandler()

If EXTIx interrupt is ON,  Implement this function

```c++
void EXTIx_IRQHandler(void); x = 0,1,2 ....
```

***caution**: The function name varies depending on the pin number. and use in main.c (Refence: Spec Sheet)

**Example code**

```c++
void EXTIx_IRQHandler(void); // called by interrupt
```



## Interrupt SysTick

### Header File

 `#include "ecSystick.h"`


```c++
#ifndef __EC_SYSTICK_H
#define __EC_SYSTICK_H

#include "stm32f4xx.h"
#include "ecRCC.h"
#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

extern volatile uint32_t msTicks;
void SysTick_init(void);
void SysTick_Handler(void);
void SysTick_counter(void);
void delay_ms(uint32_t msec);
void SysTick_reset(void);
uint32_t SysTick_val(void);
void SysTick_enable(void);
void SysTick_disable (void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

```
![image](https://user-images.githubusercontent.com/84221531/139367509-558fa483-d7ab-4edb-a8aa-9fb9b82de6bc.png)


### SysTick_init(void);

Initialize Systick Clock.

```c++
void SysTick_init(void);
```

**Parameters**

* **msec**: reload value + 1

  Calculate Method: Reload = ( Systick inverval(Goal)/ Clock Period ) -1

**Example code**

```c++
SysTick_Init(84000); // Systick clock is 1ms , assume PLL Clock 84MHz
```



### SysTick_Handler()

If Systick interrupt is ON,  Implement this function

```c++
void SysTick_Handler(void);
```

***caution**: use in main.c 

**Example code**

```c++
SysTick_Handler(); // Called by interrupt
```



### SysTick_reset()

reset the systick val

```c++
void SysTick_reset(void);
```

**Example code**

```c++
SysTick_reset(); //reset the val value
```



### delay_ms()

Delay by the input time.

```c++
void delay_ms(uint32_t msec);
```

**Parameters**

* **msec**: Time of ms Unit

**Example code**

```c++
delay_ms(1000); // Give 1s delay
```



### SysTick_val()

Return the current Systick reload

```c++
uint32_t SysTick_val(void);
```

**Example code**

```c++
SysTick_val(void); // return current reload value
```



### SysTick_enable()

Enable Systick 

```c++
void SysTick_enable(void);
```

**Example code**

```c++
SysTick_enable(); // enable Systick interrupt.
```



### SysTick_disable()

Disable Systick 

```c++
void SysTick_disable(void);
```

**Example code**

```c++
SysTick_disable(); // disable Systick interrupt.
```







## Timer

### Header File

 `#include "ecTIM.h"`


```c++
#ifndef __EC_TIM_H 
#define __EC_TIM_H
#include "stm32f411xe.h"
#include "ecPinNames.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Input Capture*/
// ICn selection according to CHn
#define FIRST 1
#define SECOND 2

// Edge Type
#define IC_RISE 0
#define IC_FALL 1
#define IC_BOTH 2

// IC Number
#define IC_1 1
#define IC_2 2
#define IC_3 3
#define IC_4 4

/* Timer Configuration */
void TIM_init(TIM_TypeDef *TIMx, uint32_t msec);  
void TIM_period_us(TIM_TypeDef* TIMx, uint32_t usec);  
void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec);

void TIM_UI_init(TIM_TypeDef* TIMx, uint32_t msec); 
void TIM_UI_enable(TIM_TypeDef* TIMx);
void TIM_UI_disable(TIM_TypeDef* TIMx);

uint32_t is_UIF(TIM_TypeDef *TIMx);
void clear_UIF(TIM_TypeDef *TIMx);

void ICAP_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN);
void ICAP_init(PinName_t pinName);
void ICAP_setup(PinName_t pinName, int ICn, int edge_type);
void ICAP_counter_us(PinName_t pinName, int usec);
uint32_t ICAP_capture(TIM_TypeDef* TIMx, uint32_t ICn);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```



### TIM_init()

Turn on Timer counter : default upcounter

```c++
void TIM_init(TIM_TypeDef *timerx, uint32_t msec);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11
* **msec**: Count period // Unit msec 

**Example code**

```c++
TIM_init(TIM2, 1) //1ms Timer ON.
```



### TIM_period_us()

Select TImer period: Unit(us)

```c++
void TIM_period_us(TIM_TypeDef* timx, uint32_t usec);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11
* **usec**: Timer period.

**Example code**

```c++
TIM_period_us(TIM2, 10); //Setting timer 2  (10usec)  
```



### TIM_period_ms()

Select TImer period: Unit(ms)

```c++
void TIM_period_ms(TIM_TypeDef* timx, uint32_t msec);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11
* **msec**: Timer period.

**Example code**

```c++
TIM_period_ms(TIM2, 10); //Setting timer 2  (10msec) 
```



### TIM_UI_init()

Interrupt by using Timer 

```c++
void TIM_INT_init(TIM_TypeDef* timerx, uint32_t msec);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11
* **msec**: Timer period. Specify IRQ according to the timer.

**Example code**

```c++
TIM_INT_init(TIM2,10) // Setting interrupt every 10ms
```



### TIM_UI_enable()

TImer interrupt ON

```c++
void TIM_INT_enable(TIM_TypeDef* timx);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11

**Example code**

```c++
TIM_INT_enable(TIM2) // Timer 2 interrput ON
```



### TIM_UI_disable()

TImer interrupt OFF

```c++
void TIM_INT_disable(TIM_TypeDef* timx);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11

**Example code**

```c++
TIM_INT_disable(TIM2) // Timer 2 interrput OFF
```



### is_UIF()

Check UIFlag : ON , OFF

```c++
uint32_t is_UIF(TIM_TypeDef *TIMx);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11

**Example code**

```c++
is_UIF(TIM2); //Drive (TIMx->SR & TIM_SR_UIF)!= 0 // delay 1000ms
```



### clear_UIF()

Clear UIFlage

```c++
void clear_UIF(TIM_TypeDef *TIMx);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11

**Example code**

```c++
clear_UIF(TIM2) // clear Timer 2 UI Flag 
```



### ICAP_init()

Initialize Input capture mode, and select input port and pin

```c++
void ICAP_init(PinName_t pinName)
```

**Parameters**

* **PinName_t pinNam**: Defined pin

**Example code**

```c++
ICAP_init(ECHO);   	
```



### ICAP_setup()

Select input capture channel and edge detech

```c++
void ICAP_setup(PinName_t pinName, int ICn, int edge_typ);
```

**Parameters**

* **ICn**: IC channel number ex) Timer2 channel 3, 4
* **edge_type**: RISE(0), FALL(1), BOTH(2) // edge detech

**Example code**

```c++
ICAP_setup(ECHO, 3, RISE); //when rising edge channel 3 does input capture  
```



### ICAP_counter_us()

Select input capture timer's  counter period

```c++
void ICAP_counter_us(PinName_t pinName, int usec);
```

**Parameters**

* **usec**: timer period (Unit: usec)

**Example code**

```c++
ICAP_counter_us(ECHO, 10); //counter period is 10usec
```



### ICAP_capture()

Select input capture timer's  counter period

```c++
uint32_t ICAP_capture(TIM_TypeDef* TIMx, uint32_t ICn); // Capture value
```

**Parameters**

* **ICn**: IC channel number ex) Timer2 channel 3, 4

**Example code**

```c++
ICAP_capture(2, 4);
```







## PWM

### Header File

 `#include "ecPWM.h"`


```c++
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecTIM.h"  			// change to ecTIM.h
#include "ecPinNames.h"

#ifndef __EC_PWM_H
#define __EC_PWM_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* PWM Configuration using PinName_t Structure */

/* PWM initialization */
// Default: 84MHz PLL, 1MHz CK_CNT, 50% duty ratio, 1msec period
void PWM_init(PinName_t pinName);
void PWM_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN);


/* PWM PERIOD SETUP */
// allowable range for msec:  1~2,000
void PWM_period(PinName_t pinName,  uint32_t msec);	
void PWM_period_ms(PinName_t pinName,  uint32_t msec);	// same as PWM_period()
// allowable range for usec:  1~1,000
void PWM_period_us(PinName_t pinName, uint32_t usec);


/* DUTY RATIO SETUP */
// High Pulse width in msec
void PWM_pulsewidth(PinName_t pinName, uint32_t pulse_width_ms);
void PWM_pulsewidth_ms(PinName_t pinName, uint32_t pulse_width_ms);  // same as void PWM_pulsewidth
void PWM_pulsewidth_us(PinName_t pinName, uint32_t pulse_width_us);
// Duty ratio 0~1.0
void PWM_duty(PinName_t pinName, float duty);



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

```



### PWM_init()

Select OUTPUT(PWM) Port, Pin 

```c++
void PWM_init(PinName_t pinName);
```

**Parameters**

* **PinName_t pinName**:  Deifined pin

**Example code**

```c++
PWM_init(PWM1);
```



### PWM_pinmap()

Mapping pin

```c++
void PWM_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN);
```

**Parameters**

* **PinName_t pinName**:  Deifined pin
* **TIM_TypeDef **TIMx**: timer number
* **chn**: channel

**Example code**

```c++
PWM_pinmap(PWM1, 1, 1);
```



### PWM_period()

Set pwm period using msec

```c++
void PWM_period(PinName_t pinName,  uint32_t msec);	
```

**Parameters**

* **uint32_t msec**:  allowable range for msec:  1~2,000

  

**Example code**

```c++
PWM_period(PWM1,20); // PWM Period 20ms   
```



### PWM_period_ms()

Same as same as PWM_period()

```c++
void PWM_period_ms(PWM_t *pwm,  uint32_t msec);	
```



### PWM_period_us()

Select PWM period: Unit(us)

```c++
void PWM_period_us(PinName_t pinName, uint32_t usec);  
```

**Parameters**

* **usec**: PWM period (us)

**Example code**

```c++
PWM_period_us(PWM1,20); // PWM Period 20us   
```



### PWM_pulsewidth_ms()( =PWM_pulsewidth())

Make the pulsewidth Unit(ms)

```c++
void PWM_pulsewidth_ms(PinName_t pinName, uint32_t pulse_width_ms);
```

**Parameters**

* **pulse_width_ms**: pulse width Unit(ms)  // "Pulse width" within the declared period of "PWM".

**Example code**

```c++
PWM_pulsewidth_ms(PWM1, 0.5) 
```



### PWM_duty()

Select the duty ratio

```c++
void PWM_duty(PinName_t pinName, float duty);
```

**Parameters**

* **duty**: "duty ratio" that I want to use. 

**Example code**

```c++
PWM_duty(PWM1, 0.5/20) //  pulsewidth 0.5ms / total 20ms = duty // 2.5% 
```



![image](https://user-images.githubusercontent.com/84221531/138924327-a954a858-dbab-4b97-ab68-6dff2936022b.png)

![image](https://user-images.githubusercontent.com/84221531/139368612-297ebd6d-6cf9-4eb6-9282-9df1f9ef59c4.png)

## Stepper Motor

### Header File

 `#include "ecStepper.h"`


```c++

#include "ecSysTick.h"
#include "ecPinNames.h"
#ifndef __EC_STEPPER_H
#define __EC_STEPPER_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

//State mode
#define HALF 0
#define FULL 1	 
	 
/* Stepper Motor */
//stepper motor function

typedef struct{
	GPIO_TypeDef *port1;
	int pin1;
	GPIO_TypeDef *port2;
	int pin2;
	GPIO_TypeDef *port3;
	int pin3;
	GPIO_TypeDef *port4;
	int pin4;
	uint32_t _step_num;
} Stepper_t;

	 
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);
void Stepper_setSpeed(long whatSpeed);
void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode); 
void Stepper_stop(void);



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

```



### Stepper_init()

Select stepper motor output pin and initialization

```c++
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);
```

**Parameters**

* **GPIO_TypeDef* port1~4**:  Select GPIO port GPIOA~GPIOH
* **pin1~4**: Select the pin number 0~15 

**Example code**

```c++
Stepper_init(GPIOB,10,GPIOB,4,GPIOB,5,GPIOB,3); // PB10, PB4, PB5, PB3 initialization 
```



### Stepper_setSpeed()

Select Rotation speed of the motor

```c++
void Stepper_setSpeed (long whatSpeed);
```

**Parameters**

* **whatSpeed**: Motor Speed (Unit: RPM)  

**Example code**

```c++
Stepper_setSpeed (3); //3RPM
```



### Stepper_step()

The number of next step, 5steps => S0-> S1-> S2-> S3-> S0

```c++
void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode); 
```

**Parameters**

* **steps**: total steps numbers
* **direction**: Right cycle(0), Left cycle(1)
* **mode**: FULL(0) , HALF(1) 

**Example code**

```c++
Stepper_step(1000, Right, FULL); // FULL MODE, Right cycle, 1000steps 
```



### Stepper_stop ()

Stop motor

```c++
void Stepper_stop (); 
```

**Example code**

```c++
 Stepper_stop (); 
```





## ADC(Analog to Digital Converter)

### Header File

 `#include "ecADC.h"`


```c++
#ifndef __MY_ADC_H
#define __MY_ADC_H
#include "stm32f411xe.h"
#include "ecPinNames.h"


// ADC trigmode
#define SW 0
#define TRGO 1

// ADC contmode
#define CONT 0
#define SINGLE 1

// Edge Type
#define RISE 1
#define FALL 2
#define BOTH 3

#define _DEFAULT 0

/////////////////////////////////////////////////////
// ADC default setting
/////////////////////////////////////////////////////

// ADC init
// Default:  one-channel mode, continuous conversion
// Default: HW trigger - TIM3 counter, 1msec
void ADC_init(PinName_t pinName);
void JADC_init(PinName_t pinName);


// Multi-Channel Scan Sequence 
void ADC_sequence(PinName_t *seqCHn, int seqCHnums); 
void JADC_sequence(PinName_t *seqCHn, int seqCHnums); 

void ADC_start(void);
void JADC_start(void);

// flag for ADC interrupt
uint32_t is_ADC_EOC(void);
uint32_t is_ADC_OVR(void);
void clear_ADC_OVR(void);

// read ADC value
uint32_t ADC_read(void);



// Private Function
void ADC_pinmap(PinName_t pinName, uint32_t *chN);

#endif

```



### ADC_init()	

Initialize Setted pin to ADC

```c++
void ADC_init(PinName_t pinName);	
```

**Example code**

```c++
ADC_init(TRGO); 
```



### ADC_sequence()

Set the Channel sequence

```c++
void ADC_sequence(PinName_t *seqCHn, int seqCHnums); 
```

**Parameters**

* **PinName_t *seqCHn**: Defined pin with channel
* **seqCHnums**:  Chage to Multi-Channel mode(scan mode) 

**Example code**

```c++
ADC_sequence(IR1,1); 
```



### ADC_start()

ADC is started 

```c++
void ADC_start(void);
```

**Example code**

```c++
ADC_start(); //ADC start 
```



### uint32_is_ADC_EOC()

If conversion has been finished, this bit is setted as 1. Then, it is cleared by software. 

```c++
uint32_t is_ADC_EOC(void);
```

**Example code**

```c++
is_ADC_EOC(); //Check if Conversion is finished
```



### uint32_is_ADC_OVR()

this bit is set by hardware when data are lost.

```c++
uint32_t is_ADC_OVR(void);
```

**Example code**

```c++
is_ADC_OVR(); // check the overrap.
```



### uint32_clear_ADC_OVR()

Clear overrap bit

```c++
void clear_ADC_OVR(void);
```

**Example code**

```c++
clear_ADC_OVR(); // clear the overrap.
```



### ADC_read()

Read current data.

```c++
uint32_t ADC_read(void);
```

**Example code**

```c++
ADC_read(); //current analog data.
```





### ADC_pinmap()

Match the Channel each port and pin

```c++
uint32_t ADC_pinmap(PinName_t pinName, uint32_t *chN);
```

**Parameters**

* **chN**: Channel neumber

**Example code**

```c++
ADC_pinmap(ADC1, 2); 
```

## UART

 `#include "ecUART.h"`


```c++
#ifndef __EC_USART_H
#define __EC_USART_H

#include <stdio.h>
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"

#define POL 0
#define INT 1

// You can modify this
#define BAUD_9600	9600
#define BAUD_19200	19200
#define BAUD_38400  	38400
#define BAUD_57600	57600
#define BAUD_115200 	115200
#define BAUD_921600 	921600


// ********************** USART 2 (USB) ***************************
// PA_2 = USART2_TX
// PA_3 = USART2_RX
// Alternate function(AF7), High Speed, Push pull, Pull up
// APB1
// **********************************************************

// ********************** USART 1 ***************************
// PA_9 = USART1_TX (default)	// PB_6  (option)
// PA_10 = USART1_RX (default)	// PB_3 (option)
// APB2
// **********************************************************

// ********************** USART 6 ***************************
// PA_11 = USART6_TX (default)	// PC_6  (option)
// PA_12 = USART6_RX (default)	// PC_7 (option)
// APB2
// **********************************************************

// Configuration UART 1, 2 using default pins 
void UART1_init(void);
void UART2_init(void);	
void UART1_baud(uint32_t baud);
void UART2_baud(uint32_t baud);

// USART write & read
void USART1_write(uint8_t* buffer, uint32_t nBytes);
void USART2_write(uint8_t* buffer, uint32_t nBytes);
uint8_t USART1_read(void);										
uint8_t USART2_read(void);	

// RX Inturrupt Flag USART1,2
uint32_t is_USART1_RXNE(void);
uint32_t is_USART2_RXNE(void);

// private functions
void USART_write(USART_TypeDef* USARTx, uint8_t* buffer, uint32_t nBytes);
void USART_init(USART_TypeDef* USARTx, uint32_t baud);  		
void UART_baud(USART_TypeDef* USARTx, uint32_t baud);											
uint32_t is_USART_RXNE(USART_TypeDef * USARTx);
uint8_t USART_read(USART_TypeDef * USARTx);										
void USART_setting(USART_TypeDef* USARTx, GPIO_TypeDef* GPIO_TX, int pinTX, GPIO_TypeDef* GPIO_RX, int pinRX, uint32_t baud); 
void USART_delay(uint32_t us);  

#endif
```



### UART1_init(), UART1_init(), 

Initialize only UART2. when we communicate with Tera-Term, this function is necessary.

```c++
void UART2_init();
```

**Example code**

```c++
UART2_init(); // Start UART2
```



### UART2_baud()

Set uart2 baud

```c++
UART_baud2(baud);
```

**Example code**

```c++
UART_baud2(BAUD_9600);
```





### USART_write()

this function for writing in tera-term.

```c++
void USART_write(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes);
```

**Parameters**

* **USART_TypeDef * USARTx**: USART1, USART2 ...
* **buffer**: saved data
  * **nBytes**: the number of Character

  **Example code**

```c++
USART_write(USART1, &muc2Data, 1); //write down the muc2Data's value in tera-term
```



### USART_delay()

We need waiting time so Use this function. This function works similar to delay_ms()

```c++
void USART_delay(uint32_t us);
```

**Parameters**

* **us**: delay time (Unit: usec)

**Example code**

```c++
USART_delay(300); delay 300 usec
```



### USART_init()

in this funcition, Specify the pin according to Uart's number.

```c++
void USART_init(USART_TypeDef* USARTx, uint32_t baud);
```

**Parameters**

* **USART_TypeDef* USARTx**: USART1, USART2 ...
* **baud**: communication rate //9600 .. 

**Example code**

```c++
  USART_init(USART2, 38400); //ON USART2 and baud-rate 38400
```



### USART_setting()

Turn on the USART communication. this function usually use in the USART_init()

```c++
void USART_begin(USART_TypeDef* USARTx, GPIO_TypeDef* GPIO_TX, int pinTX,int GPIO_TypeDef* GPIO_RX, int pinRX, int baud);
```

**Parameters**

* **USART_TypeDef* USARTx**: USART1, USART2 ...

* **GPIO_TypeDef* GPIO_TX**: GPIOA~GPIOH 

* **pinTX**: 0~15

* **pinTX,GPIO_TypeDef* GPIO_RX**: GPIOA~GPIOH

* **pinRX**: 0~15

  *Check the UASRAT Pin map
  
  ![image](https://user-images.githubusercontent.com/84221531/143729060-f5b9680a-3e6d-4790-8499-b547cd364926.png)

**Example code**

```c++
  USART_setting(USART1, GPIOA,9,GPIOA,10, 9600); //USART1 baud: 9600 
```



### uint8_t USART_read()

Get the USART DATA

```c++
uint8_t USART_read(USART_TypeDef * USARTx);		
```

**Parameters**

* **USART_TypeDef* USARTx**: USART1, USART2 ...

**Example code**

```c++
uint8_t USART_readc(USART2);
```



### is_USART_RXNE()

Make sure you are ready to read the value.

```c++
uint32_t is_USART_RXNE(USART_TypeDef * USARTx);
```

**Parameters**

* **USART_TypeDef* USARTx**: USART1, USART2 ...
![image](https://user-images.githubusercontent.com/84221531/143729012-2bba2191-e665-4c2b-9f68-d8144016719c.png)


**Example code**

```
is_USART_RXNE(USART1); 
```

![image](https://user-images.githubusercontent.com/84221531/143729012-2bba2191-e665-4c2b-9f68-d8144016719c.png)

