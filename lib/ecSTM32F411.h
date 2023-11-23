/*----------------------------------------------------------------\
@ Embedded Controller by Young-Min Kim - Handong Global University
Author           : 21900141 Young-Min Kim

Language/ver     : C++ in Keil uVision

Description      : To include all headers, it is new version of ec hall, for defalut code, could be changed
/----------------------------------------------------------------*/

#include "ecGPIO.h"
#include "ecEXTI.h"
#include "ecRCC.h"
#include "ecTIM.h"

#include "ecStepper.h"
#include "ecPWM.h"
#include "ecUART.h"
#include "ecSysTick.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "stm32f411xe.h"
#include "ecPinNames.h"
#include "ecICAP.h"
#include "ecADC.h"

#define TRIG PB_6
#define ECHO PA_0
#define LED PA_5
#define IR1 PC_1
#define IR2 PC_0
void MCU_init(void){
    // CLOCK PLL 84MHz
    RCC_PLL_init();
    SysTick_init();
    // SysTick 1msec
    SysTick_init();    
    
    // Button PC13
    GPIO_init(GPIOC, 13, 0);
    GPIO_pupd(GPIOC, 13, 1);    
    
    // LED INIT
    GPIO_init(GPIOA, LED,1);
    GPIO_write(GPIOA,LED, LOW);    
	
				
    // USART Default Initialization
      UART1_init();
      UART1_baud(BAUD_9600);
			
			// Ultrasonic ECHO setting
        ICAP_init(ECHO);
        ICAP_counter_us(ECHO, 10);
        ICAP_setup(ECHO, IC_1, IC_RISE);
        ICAP_setup(ECHO, IC_2, IC_FALL);

    // Others
		//IR init
        ADC_init(IR1);
        ADC_init(IR2);
				static PinName_t seqCHn[2] = {IR1, IR2};
        ADC_sequence(seqCHn, 2);
}