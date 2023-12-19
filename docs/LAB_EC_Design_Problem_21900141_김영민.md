# LAB: EC Design Problem

**Date: 2023.12.17.**

**Author/Partner:** 21900141 Youngmin Kim

**Github**:  




## Introduction

In this project design lab, I aimed to create a machine that allows introverted individuals, who find it difficult to seek help from others, to safely exercise for bulking up through working out. The project is titled "Dream Of Meruchi," encapsulating the meaning that a shy and introverted anchovy wishes to become a mackerel through exercise.

### Requirement

#### Hardware 

* MCU
  * NUCLEO-F401RE


* Actuator/Sensor/Others:
  * DC motor, DC motor driver(L9110s),
  * Bluetooth Module(HC-06),
  * IR sensor() X2,
  * Ultra sonic sensor.
  * Motor driver

#### Software
 * Keil uVision, CMSIS, EC_HAL library



## Problem 1 : EC HAL library
#### Create HAL library

##### ecUART_student.h

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

##### ecUART_student.c

```c++
#include "ecUART_student.h"
#include <math.h>

// ********************** DO NOT MODIFY HERE ***************************
// 
// Implement a dummy __FILE struct, which is called with the FILE structure.
//#ifndef __stdio_h
struct __FILE {
    //int dummy;
		int handle;

};

FILE __stdout;
FILE __stdin;
//#endif

// Retarget printf() to USART2
int fputc(int ch, FILE *f) { 
  uint8_t c;
  c = ch & 0x00FF;
  USART_write(USART2, (uint8_t *)&c, 1);
  return(ch);
}

// Retarget getchar()/scanf() to USART2  
int fgetc(FILE *f) {  
  uint8_t rxByte;
  rxByte = USART_read(USART2);
  return rxByte;
}


/*================ private functions ================*/
void USART_write(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes) {
	// TXE is set by hardware when the content of the TDR 
	// register has been transferred into the shift register.
	int i;
	for (i = 0; i < nBytes; i++) {
		// wait until TXE (TX empty) bit is set
		while (!(USARTx->SR & USART_SR_TXE));  
		// Writing USART_DR automatically clears the TXE flag 	
		USARTx->DR = buffer[i] & 0xFF;
		USART_delay(300);
	}
	// wait until TC bit is set
	while (!(USARTx->SR & USART_SR_TC));		
	// TC bit clear
	USARTx->SR &= ~USART_SR_TC;	

}  
 
uint32_t is_USART_RXNE(USART_TypeDef * USARTx){
	return (USARTx->SR & USART_SR_RXNE);
}


uint8_t USART_read(USART_TypeDef * USARTx){
	// Wait until RXNE (RX not empty) bit is set by HW -->Read to read
	while ((USARTx->SR & USART_SR_RXNE) != USART_SR_RXNE);  
	// Reading USART_DR automatically clears the RXNE flag 
	return ((uint8_t)(USARTx->DR & 0xFF)); 
}

void USART_setting(USART_TypeDef* USARTx, GPIO_TypeDef* GPIO_TX, int pinTX, GPIO_TypeDef* GPIO_RX, int pinRX, uint32_t baud){
//1. GPIO Pin for TX and RX	
	// Enable GPIO peripheral clock 	 
	// Alternative Function mode selection for Pin_y in GPIOx
	// AF, Push-Pull, No PUPD, High Speed
	GPIO_init(GPIO_TX, pinTX, AF);
	GPIO_otype(GPIO_TX, pinTX, 0);
	GPIO_pupd(GPIO_TX, pinTX, 00);		
	GPIO_ospeed(GPIO_TX, pinTX, HIGH);						
	
	GPIO_init(GPIO_RX, pinRX, AF);
	GPIO_otype(GPIO_RX, pinRX, 0);
	GPIO_pupd(GPIO_RX, pinRX, 00);
	GPIO_ospeed(GPIO_RX, pinRX, HIGH);
	
	// Set Alternative Function Register for USARTx.	
	// AF7 - USART1,2 
	// AF8 - USART6 	 
	if (USARTx == USART6){ 
		// USART_TX GPIO AFR
		if (pinTX < 8) GPIO_TX->AFR[0] |= 8 << (4*pinTX);
		else 					 GPIO_TX->AFR[1] |= 8 << (4*(pinTX-8));
		// USART_RX GPIO AFR
		if (pinRX < 8) GPIO_RX->AFR[0] |= 8 << (4*pinRX); 
		else 					 GPIO_RX->AFR[1] |= 8 << (4*(pinRX-8));
	}
	else{	//USART1,USART2
		// USART_TX GPIO AFR
		if (pinTX < 8) GPIO_TX->AFR[0] |= 7 << (4*pinTX);
		else 					 GPIO_TX->AFR[1] |= 7 << (4*(pinTX-8));
		// USART_RX GPIO AFR
		if( pinRX < 8) GPIO_RX->AFR[0] |= 7 << (4*pinRX);
		else 					 GPIO_RX->AFR[1] |= 7 << (4*(pinRX-8));
	}

	
//2. USARTx (x=2,1,6) configuration	
	// Enable USART peripheral clock 
	if (USARTx == USART1)
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN; 	// Enable USART 1 clock (APB2 clock: AHB clock = 84MHz)	
	else if(USARTx == USART2)
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  	// Enable USART 2 clock (APB1 clock: AHB clock/2 = 42MHz)
	else
		RCC->APB2ENR |= RCC_APB2ENR_USART6EN;  	// Enable USART 6 clock (APB2 clock: AHB clock = 84MHz)
	
	// Disable USARTx. 
	USARTx->CR1  &= ~USART_CR1_UE; 						// USART disable
	 
	// No Parity / 8-bit word length / Oversampling x16 
	USARTx->CR1&= ~USART_CR1_M;   		// No parrity bit
	USARTx->CR1&= ~USART_CR1_PCE;       	// M: 0 = 8 data bits, 1 start bit    
	USARTx->CR1 &= ~USART_CR1_OVER8;  	// 0 = oversampling by 16 (to reduce RF noise)	 
	// Configure Stop bit
	USARTx->CR2 &= ~USART_CR2_STOP;  		// 1 stop bit																 

	// CSet Baudrate to 9600 using APB frequency (42MHz)
	// If oversampling by 16, Tx/Rx baud = f_CK / (16*USARTDIV),  
	// If oversampling by 8,  Tx/Rx baud = f_CK / (8*USARTDIV)
	// USARTDIV = 42MHz/(16*9600) = 237.4375

	UART_baud(USARTx, baud);

	// Enable TX, RX, and USARTx 
	USARTx->CR1 |= (USART_CR1_RE | USART_CR1_TE);   	// Transmitter and Receiver enable
	USARTx->CR1 |= USART_CR1_UE; 										// USART enable
	
	
// 3. Read USARTx Data (Interrupt)	
	// Set the priority and enable interrupt
	USARTx->CR1 |= USART_CR1_RXNEIE;       			// Received Data Ready to be Read Interrupt
	if (USARTx == USART1){
		NVIC_SetPriority(USART1_IRQn, 1);      		// Set Priority to 1
		NVIC_EnableIRQ(USART1_IRQn);             	// Enable interrupt of USART2 peripheral
	}
	else if (USARTx == USART2){
		NVIC_SetPriority(USART2_IRQn, 1);      		// Set Priority to 1
		NVIC_EnableIRQ(USART2_IRQn);             	// Enable interrupt of USART2 peripheral
	}
	else {																			// if(USARTx==USART6)
		NVIC_SetPriority(USART6_IRQn, 1);      		// Set Priority to 1
		NVIC_EnableIRQ(USART6_IRQn);            	// Enable interrupt of USART2 peripheral
	}
	USARTx->CR1 |= USART_CR1_UE; 							// USART enable
} 


void UART_baud(USART_TypeDef* USARTx, uint32_t baud){
	// Disable USARTx. 
	USARTx->CR1  &= ~USART_CR1_UE; 						// USART disable
	USARTx->BRR = 0;
	
// Configure Baud-rate 
	float fCK = 84000000;                                    // if(USARTx==USART1 || USARTx==USART6), APB2
	if(USARTx == USART2) fCK =fCK/2;      // APB1

// Method 1
	float USARTDIV = (float) fCK/16/baud;
	uint32_t mantissa = (uint32_t)USARTDIV;
	uint32_t fraction = round((USARTDIV-mantissa)*16);
	USARTx->BRR |= (mantissa<<4|fraction);
	
	// Enable TX, RX, and USARTx 
	USARTx->CR1 |= USART_CR1_UE;

}

void USART_delay(uint32_t us) {
   uint32_t time = 100*us/7;    
   while(--time);   
}
	

/*================ Use functions ================*/
void UART1_init(void){
	// ********************** USART 1 ***************************
	// PA_9 = USART1_TX (default)	// PB_6  (option)
	// PA_10 = USART1_RX (default)	// PB_3 (option)
	// APB2
	// **********************************************************
	USART_setting(USART1, GPIOA, 9, GPIOA, 10, 9600);
}
void UART2_init(void){
	// ********************** USART 2 ***************************
	// PA2 = USART2_TX
	// PA3 = USART2_RX
	// Alternate function(AF7), High Speed, Push pull, Pull up
	// **********************************************************
	USART_setting(USART2, GPIOA, 2, GPIOA, 3, 9600);
}

void UART1_baud(uint32_t baud){
	UART_baud(USART1, baud);
}
void UART2_baud(uint32_t baud){
	UART_baud(USART2, baud);
}

void USART1_write(uint8_t* buffer, uint32_t nBytes){
	USART_write(USART1, buffer, nBytes);
}

void USART2_write(uint8_t* buffer, uint32_t nBytes){
	USART_write(USART2, buffer, nBytes);
}
uint8_t USART1_read(void){
	return USART_read(USART1);
}	

uint8_t USART2_read(void){
	return USART_read(USART2);
}

uint32_t is_USART1_RXNE(void){
	return is_USART_RXNE(USART1);
}
uint32_t is_USART2_RXNE(void){
	return is_USART_RXNE(USART2);
}

```
# Problem 2:  Communicate MCU1-MCU2 using RS-232

![image-20231117042853115](C:\Users\Zzero1009\AppData\Roaming\Typora\typora-user-images\image-20231117042853115.png)

##### code

```c++
/**
******************************************************************************
* @author ECLAB
* @Mod       2023-11-16 by YMKIM
* @brief   Embedded Controller:  USART LED CONNECT
*
******************************************************************************
*/
#include "..\..\lib\ecHAL.h"

#define LED_PA_5

static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;
uint8_t PC_string[]="Loop:\r\n";


void setup(void){
	RCC_PLL_init();
	SysTick_init();
	
GPIO_init(GPIOA,5,1);
	
	// USART2: USB serial init
	UART2_init();
	UART2_baud(BAUD_38400);

	// USART1: BT serial init 
	UART1_init();
	UART1_baud(BAUD_38400);
}

int main(void){	
	setup();
	printf("MCU Initialized\r\n");	
	while(1){
		switch(BT_Data){
			
		case 'H':

case 'h':

GPIO_write(GPIOA,5,1);

break;

case 'L':

case 'l':

GPIO_write(GPIOA,5,0);

break;

      
	}
}
	}

void USART2_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	if(is_USART2_RXNE()){
		PC_Data = USART2_read();		// RX from UART2 (PC)
		USART1_write(&PC_Data,1);		// TX to USART2	 (PC)	 Echo of keyboard typing		
		USART2_write(&PC_Data,1);
	}
}
void USART1_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	if(is_USART1_RXNE()){
		BT_Data = USART1_read();		// RX from UART1 (BT)		
		if (BT_Data != '\0'){
		printf("MCU_1 received : %c \r\n", BT_Data); // TX to USART2(PC)
		}

	}
}
```

### Result
https://youtu.be/N30-rabblsc

this link is for led

# Problem 3: Control DC Motor via Bluetooth

#### Control DC Motor via Bluetooth
![image-20231117042920041](C:\Users\Zzero1009\AppData\Roaming\Typora\typora-user-images\image-20231117042920041.png)

![image-20231117043051646](C:\Users\Zzero1009\AppData\Roaming\Typora\typora-user-images\image-20231117043051646.png)

![image-20231117043117553](C:\Users\Zzero1009\AppData\Roaming\Typora\typora-user-images\image-20231117043117553.png)

##### code

```c++

/**
******************************************************************************
* @author  SSSLAB
* @Mod       2023-11-17 by YKKIM
* @brief   Embedded Controller:  USART RC CAR
*
******************************************************************************
*/


// USART2 : MCU to PC via usb 
// USART1 : MCU to MCU2
#include "..\..\lib\ecHAL.h"
#define PWM1 PA_0
#define PWM2 PA_6

uint8_t mcu2Data = 0;
uint8_t pcData = 0;
int indx =0;
int maxBuf=10;
uint8_t buffer[100]={0,};
uint8_t buffer2 = '\r\n';
int endChar = 13;

int dir =0;
int Right_DC_velocity =0;
int Left_DC_velocity =0;
int MT_angle = 3;
float duty = 0.f;

void RC_control(char cmd);
void setup(void);

int main(void) {
   // Initialiization --------------------------------------------------------
   setup();
   printf("Hello Nucleo\r\n");
	 delay_ms(500);

 
   // Inifinite Loop ----------------------------------------------------------
   while (1){
       
     if(dir ==0){
     printf("RC car: DIR:%d[deg] VEL:%d[%%] FWD\r\n\r\n",(MT_angle-1)*45,Right_DC_velocity*25);
		 printf("RC car: DIR:%d[deg] VEL:%d[%%] FWD\r\n\r\n",(MT_angle-1)*45,Left_DC_velocity*25);
     }else if(dir ==1)
     printf("RC car: DIR:%d[deg] VEL:%d[%%] BWD\r\n\r\n",(MT_angle-1)*45,Right_DC_velocity*25);
		 printf("RC car: DIR:%d[deg] VEL:%d[%%] BWD\r\n\r\n",(MT_angle-1)*45,Left_DC_velocity*25);
		 delay_ms(1000);
	}
}

// Initialiization 
void setup(void)
{
  RCC_PLL_init();
  SysTick_init();
   
  // USART congfiguration
  
	UART2_init();
	UART2_baud(BAUD_9600);
	USART_setting(USART1, GPIOA,9,GPIOA,10, BAUD_9600);
	//PWM setting
	PWM_init(PWM1);  //RC servo angle //TIM2 - ch2
	PWM_init(PWM2); //DC Motor Speed //TIM2 - ch3
	
	PWM_period_ms(PWM1, 20);	 //TIMER 2 period
	PWM_duty(PWM1,0);					//DC default
	PWM_duty(PWM2,0);			//servo default

	
	//GPIO output setting
	GPIO_init(GPIOA,0,OUTPUT);
	GPIO_init(GPIOA,6,OUTPUT);	
	

}

void RC_control(char cmd)
{
	switch(cmd){
		case 'I' : 
			Right_DC_velocity	++; 
			Left_DC_velocity	++; 
				break;
		case 'M' : 
			Right_DC_velocity--; 
			Left_DC_velocity--; 
				break;
		case 'R' : MT_angle++;    break;
		case 'L' : MT_angle--;    break;
		case 'W' : dir =0;				break;
		case 'S' : dir =1;				break;
		case 'X' : 
				Right_DC_velocity = 0;	
				Left_DC_velocity = 0;
		MT_angle = 3; break;
		}
	
	if((Right_DC_velocity < 0)&&(Left_DC_velocity < 0))
		{
		Right_DC_velocity = 0;
		Left_DC_velocity = 0;
	
	}else if((Right_DC_velocity > 4)&&(Left_DC_velocity > 4)){
		Right_DC_velocity  = 4;
		Left_DC_velocity  = 4;			
	}
	if		 (MT_angle < 1) MT_angle = 1;
  else if(MT_angle > 5) MT_angle = 5;
	
	GPIO_write(GPIOA,11,dir);
	GPIO_write(GPIOA,12,dir);
		
	if(dir == 0){
		
		duty = 0.25 * (float)Right_DC_velocity;
	}
  else if(dir == 1){
		
		duty = 1.0 - 0.25 * (float)Right_DC_velocity;
	}
	
	PWM_duty(PWM2,duty);			
	PWM_duty(PWM1,0.025*MT_angle);	
	
}


//FIX this breciecve 
void USART1_IRQHandler(){      //USART1 INT 
   if(is_USART_RXNE(USART1)){
      mcu2Data = USART_read(USART1);
			USART_write(USART1,&mcu2Data,1);
      if(mcu2Data==endChar) {
				 RC_control(buffer[0]);
				 USART_write(USART1,&buffer2,1);
				 buffer[0] = NULL;
         indx = 0;
				
      }
      else{
         if(indx>maxBuf){
            indx =0;
            memset(buffer, 0, sizeof(char) * maxBuf);
            printf("ERROR : Too long string\r\n");
         }
         buffer[indx] = mcu2Data;
         indx++;
				 
      }
   }
}

void USART2_IRQHandler(){      //USART2 INT 
   if(is_USART_RXNE(USART2)){
      pcData = USART_read(USART2);
      USART_write(USART1,&pcData,1);   // transmit char to USART1
      printf("%c",pcData);             // echo to sender(pc)
      
      if(pcData==endChar){
         printf("\r\n");               // to change line on PC display
      }
   }
}

```

### Result
https://youtu.be/N30-rabblsc

for rc car
