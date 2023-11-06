#include "stm32f4xx.h"
#include "ecStepper.h"
#include "ecPinNames.h"
#include "ecSysTick.h"



//State number 
#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7


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

State_full_t FSM_full[4] = {  
 	{0b1100,{S1,S3}},
 	{0b0110,{S2,S0}},
 	{0b1100,{S3,S1}},
 	{0b1100,{S0,S2}},
};

//HALF stepping sequence
typedef struct {
	uint8_t out;
  	uint32_t next[2];
} State_half_t;

State_half_t FSM_half[8] = { 
 	{0b1001,{S1,S2}},
	 // YOUR CODE
	 // YOUR CODE
	 // YOUR CODE
	 // YOUR CODE
	 // YOUR CODE
	 // YOUR CODE
	 // YOUR CODE
};



void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4){
	 
	//  GPIO Digital Out Initiation
	myStepper.port1 = port1;
	myStepper.pin1  = pin1;
	// Repeat for port2,pin3,pin4 
	myStepper.port2 = port2;
	myStepper.pin2  = pin2;
	myStepper.port3 = port3;
	myStepper.pin3 = pin3;
	myStepper.port4 = port4;
	myStepper.pin4  = pin4;
	
	//  GPIO Digital Out Initiation
	// No pull-up Pull-down , Push-Pull, Fast	

	//pin1-port1
	GPIO_pupd(port1, pin1, 00);
	GPIO_otype(port1, pin1, 0);
	GPIO_ospeed(port1, pin1, 2);
	
	GPIO_pupd(port2, pin2, 00);
	GPIO_otype(port2, pin2, 0);
	GPIO_ospeed(port2, pin2, 2);
	
	GPIO_pupd(port2, pin3, 00);
	GPIO_otype(port2, pin3, 0);
	GPIO_ospeed(port2, pin3, 2);
	
	GPIO_pupd(port2, pin4, 00);
	GPIO_otype(port2, pin4, 0);
	GPIO_ospeed(port2, pin4, 2);
}


void Stepper_pinOut (uint32_t state, uint32_t mode){	
   	if (mode == FULL){         // FULL mode
		GPIO_write(myStepper.port1, myStepper.pin1, (FSM_full[state].out & 0b1000) >> 3);
  		// Repeat for pin2~port4 
		GPIO_write(myStepper.port2, myStepper.pin2, (FSM_full[state].out & 0b0100) >> 2);
		GPIO_write(myStepper.port3, myStepper.pin3, (FSM_full[state].out & 0b0010) >>	1);
		GPIO_write(myStepper.port4, myStepper.pin4, (FSM_full[state].out & 0b1001) >>	1		);
	}	else if (mode == HALF){    // HALF mode
		// YOUR CODE 
		// YOUR CODE 
		// YOUR CODE 
		// YOUR CODE 
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
	    	if (mode == FULL) 		 												
			state = FSM_full[state].next[direction];// YOUR CODE       // state = next state
		else if (mode == HALF) 
			state = FSM_full[state].next[1];// YOUR CODE       // state = next state		
		Stepper_pinOut(state, mode);
   	}
}


void Stepper_stop (void){ 
    	myStepper._step_num = 0;    
	// All pins(A,AN,B,BN) set as DigitalOut '0'
	// YOUR CODE 
	GPIO_write(GPIOB,10,0);
	GPIO_write(GPIOB,4,0);
	GPIO_write(GPIOB,5,0);
	GPIO_write(GPIOB,8,0);

}

