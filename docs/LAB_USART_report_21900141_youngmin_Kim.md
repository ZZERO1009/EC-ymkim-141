# LAB: USART - LED, Bluetooth  

**Date: 2023.11.16.

**Author/Partner:** 21900141 Youngmin Kim

**Github**:  




## Introduction

In this lab, we will learn how to configure and use ‘USART(Universal synchronous asynchronous receiver transmitter)’ of MCU. Then, we will learn how to communicate between your PC and MCU and MCU to another MCU with wired serial communication.

- **Mission 1**: Control LED(LD2) of each other MCU.
- **Mission 2**: Run DC motors with Bluetooth



### Requirement

#### Hardware 

* MCU
  * NUCLEO-F401RE


* Actuator/Sensor/Others:
  * 3Stepper Motor 28BYJ-48
  * Motor Driver ULN2003
  * breadboard

#### Software
 * Keil uVision, CMSIS, EC_HAL library



## Problem 1 : Stepper Motor

#### Hardware Connection

![](C:\Users\Zzero1009\사진\스크린샷\스크린샷 2023-11-09 155401.png)

#### Stepper Motor Sequence

Using unipolar stepper motor for this lab

Making each output data depending on the below sequence.

##### Full-stepping sequence

![image-20231109155923463](C:\Users\Zzero1009\AppData\Roaming\Typora\typora-user-images\image-20231109155923463.png)



![image-20231109202430144](C:\Users\Zzero1009\AppData\Roaming\Typora\typora-user-images\image-20231109202430144.png)

##### Half-stepping sequence

![](C:\Users\Zzero1009\사진\스크린샷\스크린샷 2023-11-09 185321.png)

![](C:\Users\Zzero1009\사진\스크린샷\스크린샷 2023-11-09 185134.png)



### Finite State Machine

Drawing a State Table for Step Sequence, Use Moore FSM for this case.

##### Full-Stepping Sequence

![](C:\Users\Zzero1009\사진\스크린샷\스크린샷 2023-11-09 191134.png)

##### Half-Stepping Sequence

![스크린샷 2023-11-09 191155](C:\Users\Zzero1009\사진\스크린샷\스크린샷 2023-11-09 191155.png)



# Problem 2: Firmware Programming

#### Create HAL library

##### ecStepper.h

```c++
// Initialize with 4 pins
// ( A, B,  AN,  BN)
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);

//or   using ecPinNames.h 
void Stepper_init(PinName_t A, PinName_t B,  PinName_t AN, PinName_t BN);


// whatSpeed [rev/min]
void Stepper_setSpeed(long whatSpeed);

// Run for n Steps
void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode); 

// Immediate Stop.
void Stepper_stop(void);
```


##### ecStepper.c

```c++

//State number 
#define S0 0 // to s7->7



// Stepper Motor function
uint32_t direction = 1; 
uint32_t step_delay = 100; 
uint32_t step_per_rev = 64*32;
	 

// Stepper Motor variable
volatile Stepper_t myStepper; 


//FULL stepping sequence  - FSM
typedef struct {
	uint8_t out;
  	uint32_t next[2];
} State_full_t;

State_full_t FSM_full[4] = {  	// 1010 , 0110 , 0101 , 1001
 	{0b1100,{S1,S3}},
 	{0b0110,{S2,S0}},
 	{0b0011,{S3,S1}},
 	{0b1001,{S0,S2}},
};

//HALF stepping sequence
typedef struct {
	uint8_t out;
  	uint32_t next[2];
} State_half_t;

State_half_t FSM_half[8] = { // 1000 , 1010 , 0010 , 0110 , 0100 , 0101, 0001, 1001
 	{0b1000,{S1,S7}},
	{0b1100,{S2,S0}},
	{0b0100,{S3,S1}},
	{0b0110,{S4,S2}},
	{0b0010,{S5,S3}},
	{0b0011,{S6,S4}},
	{0b0001,{S7,S5}},
	{0b1000,{S0,S6}},
};



void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4){
	 
	//  GPIO Digital Out Initiation
	myStepper.port1 = port1;
	myStepper.pin1  = pin1;
	// Repeat for port2,pin3,pin4 

	
	//  GPIO Digital Out Initiation
	// No pull-up Pull-down , Push-Pull, Fast	

	//pin1-port1
	GPIO_init(port1,pin1,1);
	GPIO_pupd(port1, pin1, 00);
	GPIO_otype(port1, pin1, 0);
	GPIO_ospeed(port1, pin1, 2);
	// same sequence to other pins
}


void Stepper_pinOut (uint32_t state, uint32_t mode){	
   	if (mode == FULL){         // FULL mode
		GPIO_write(myStepper.port1, myStepper.pin1, (FSM_full[state].out & 0b1000) >> 3);
  		// Repeat for pin2~port4 
		GPIO_write(myStepper.port2, myStepper.pin2, (FSM_full[state].out & 0b0100) >> 2);
		GPIO_write(myStepper.port3, myStepper.pin3, (FSM_full[state].out & 0b0010) >>	1);
		GPIO_write(myStepper.port4, myStepper.pin4, (FSM_full[state].out & 0b1001) >>	0		);
	}	else if (mode == HALF){    // HALF mode
		GPIO_write(myStepper.port1, myStepper.pin1, (FSM_half[state].out & 0b1000) >> 3);
  		// Repeat for pin2~port4 
		GPIO_write(myStepper.port2, myStepper.pin2, (FSM_half[state].out & 0b0100) >> 2);
		GPIO_write(myStepper.port3, myStepper.pin3, (FSM_half[state].out & 0b0010) >>	1);
		GPIO_write(myStepper.port4, myStepper.pin4, (FSM_half[state].out & 0b1001) >>	0	);
	}
}


void Stepper_setSpeed (long whatSpeed){      // rpm [rev/min]
		step_delay = 60000/(whatSpeed*64*32);//YOUR CODE   // Convert rpm to  [msec] delay
}


void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode){
	 uint32_t state = 0;
	 myStepper._step_num = steps;

	 for(; myStepper._step_num > 0; myStepper._step_num--){ // run for step size
		// YOUR CODE                        // delay (step_delay); 			
			delay_ms(step_delay);
	    	if (mode == FULL) 		 												
			state = FSM_full[state].next[direction];// if full, use full stm and direction
		else if (mode == HALF) 
			state = FSM_half[state].next[direction];// if half, use half stm and direction		
		delay_ms(step_delay);
		Stepper_pinOut(state, mode);
   	}
}


void Stepper_stop (void){ 
    	myStepper._step_num = 0;    
	// All pins(A,AN,B,BN) set as DigitalOut '0'
	// by using &, all goes to zero, stop motor
FSM_full[4].out &= 0b0000;
FSM_half[8].out &= 0b0000;

}

```



####  Procedure

Connect the MCU to the motor driver and the stepper motor, run for full and half, in both direction.
Test maximum and minimum speed.



#### Configuration 

| Digital Out                                                  | SysTick |
| :----------------------------------------------------------- | :-----: |
| PB10, PB4, PB5, PB3 <br />NO Pull-up Pull-down <br />Push-Pull <br />Fast | delay() |





#### Discussion

1. **Find out the trapezoid-shape velocity profile for a stepper motor. When is this profile necessary?**
   The  Trapezoid-shape velocity profile expands the range of operation that  motor can start on.
    Linear state, power of the motor for the input pulse may be stably  transmitted or reduced.

2. **How would you change the code more efficiently for micro-stepping control? You don’t have to code this but need to explain your strategy.**

   Reduce vibration by increasing rotor pole or stator  phase. 
   



#### Main code

```c++

	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	
	Stepper_step(10000, 0,FULL);  // (Step : 1024, Direction : 0 or 1, Mode : FULL or HALF)
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){;}
}

// Initialiization 
void setup(void)
{	
	
	RCC_PLL_init();                                 // System Clock = 84MHz
	SysTick_init();                                 // Systick init
	
	EXTI_init(GPIOC,BUTTON_PIN,FALL,0);             // External Interrupt Setting
	GPIO_init(GPIOC, BUTTON_PIN, 0);           // GPIOC pin13 initialization

	Stepper_init(GPIOB,10,GPIOB,4,GPIOB,5,GPIOB,3); // Stepper GPIO pin initialization
	Stepper_setSpeed(29);                          //  set stepper motor speed
}
void EXTI15_10_IRQHandler(void) {  // This code is use for stop motor by using button.
	if (is_pending_EXTI(BUTTON_PIN)) {
		Stepper_stop();
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	} 
}

   
```



### Result

Full-Half and both direction result video: https://youtu.be/dnJqaDnRjqg
Max and minimum speed result video: https://youtu.be/cLtEtaLqBnk


