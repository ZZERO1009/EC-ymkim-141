# LAB: EC Design Problem

**Date: 2023.12.17.**

**Author/Partner:** 21900141 Youngmin Kim

**Github**: https://github.com/ZZERO1009/EC-ymkim-141/blob/03833c5bc1b0d9ac569ac70eb4a3f6e86259d95a/docs/LAB_EC_Design_Problem_21900141_%EA%B9%80%EC%98%81%EB%AF%BC.md




## 1. Introduction

In this project design lab, I aimed to create a machine that allows introverted individuals, who find it difficult to seek help from others, to safely exercise for bulking up through working out. The project is titled "Dream Of Meruchi," encapsulating the meaning that a shy and introverted anchovy wishes to become a mackerel through exercise.

### Requirement

#### Hardware 

* MCU
  * NUCLEO-F401RE


* Actuator/Sensor/Others:
  * DC motor X2,
  * Bluetooth Module(HC-06),
  * IR sensor(TCRT 5000) X2,
  * Ultra sonic sensor(HC-SR04),
  * Motor driver(L9110s)

#### Software
 * Keil uVision, Tera term



## 2. Problem 
### Problem description
This exercise machine has three modes. 

easily recognizable by names inspired by superhero comics: Captain America Mode, Alfred Mode, Jarvis Mode, and 

an "Emergency Mode" that can be activated from any mode.



##### 1. System mode(workout mode)

If the machine is powered on or reset, it prompts the user on display to select a mode (input via the numpad) in sequential order (0 for Captain America Mode, 1 for Alfred Mode, 2 for Jarvis Mode).

|      MODE       |                         Description                          |
| :-------------: | :----------------------------------------------------------: |
| Captain America | This mode (selected by default as 0) is designed for individuals seeking solitude and focus. When this mode is chosen, the display simply prompts to finish up and advises to press reset. |
|     Alfred      | This mode, similar to Batman's butler Alfred briefing Batman, updates the workout situation via the "workout()" function. Within the workout() function, it prompts for the number of reps per set and the number of sets. Using a Finite State Machine (FSM), it progresses the barbell sequentially from the highest to the lowest height and back to the highest, incrementing the count by one each time the barbell moves, until reaching the designated number of reps per set. Upon completing all reps for a set, it resets to zero and increases the set count by one. Once the set count reaches the targeted value, the function stops and indicates that the workout is finished. |
|     Jarvis      | This mode, akin to Iron Man's assistant Jarvis, actively intervenes in emergency situations. Similar to Jarvis, it utilizes IR sensors to monitor balance. Initially, if the sensor detects an imbalance on either the left or right side, it warns the user about the disrupted balance. However, if both sides deviate from the expected parameters, indicating a critical situation, it recognizes the danger and initiates an active emergency escape procedure immediately. |

##### 2. Emergency mode

This mode utilizes the "exit" function to operate from any other mode. It serves as an emergency escape mode, where upon activation, along with verbal cues, it rotates the motor to lift the barbell upward.

| **Function**    | **Sensor**                 | **Configuration**                                            | **Comments**                               |
| --------------- | -------------------------- | ------------------------------------------------------------ | ------------------------------------------ |
| Distance check  | Ultrasonic distance sensor | Using ICAP, check distance and flag and maximum distance(9cm) and minimum distance(3cm), using code "time_interval * 340.f / 2.f / 10.f;" | result will be used in workout()           |
| Balance check   | IR sensor                  | Using ADC, check value to see balance. limitation is under 2000. | using in jarvis mode                       |
| Emergency check | Button                     | On GIPOC 13pin, using exti to set on all process.            | Must could activate whenever in situation. |



## MCU Configuration

##### ![image](https://github.com/ZZERO1009/EC-ymkim-141/assets/144536736/ba7104f0-1dee-4170-8fa4-12a2dffa5814)

## CIRCUIT DIAGRAM

![image](https://github.com/ZZERO1009/EC-ymkim-141/assets/144536736/b07613bd-8f40-4086-9a13-4584a9af33fa)

![image](https://github.com/ZZERO1009/EC-ymkim-141/assets/144536736/98279d5e-3c76-408a-9de8-e6a7b7a66e29)

secod picture is modeling part for workout machine, sensor and acturator is drew half of real.



## 3. Algorithm

1. Emergency active

when press button, print that"you pressed emergency button, escaping process start!", motor start to move and barbell go up.

2. workout function.

This function is used on alfred and jarvis, survey set number and number in set. set is consist by FSM which consist of D0, D1, D2. when barbell distance is go to same or over maximum->same or less minimum->same or over maximum, +1. when number of set is completed, reset to zero and set number is plus one. when set number is complited, all number is go to zero and print workout is done.



### Code

```c#
**	
 * @author Youngmin Kim 
 * @Created 2023-12
 * @Modified 2023-12
 for final code, meruch
*/

#include "..\..\lib\ecHAL.h"



#define UP 1

#define CAPTINE 0
#define ALFRED 1
#define JARVIS 2


#define EMERGENCY 1

#define LED PA_5

// Ultrasonic
#define TRIG PB_6
#define ECHO PA_0

// MOTOR1 is left motor and MOTOR2 is right motor
#define MOTOR1_PWM PA_6
#define MOTOR1_DIR PA_11
#define MOTOR2_PWM PB_5
#define MOTOR2_DIR PA_12

#define IR1 PC_1
#define IR2 PC_0

#define emergency_button PC_13
int s=0;
int n=0;

	

typedef struct
{
        float speed;
        uint8_t next_state[2];
} SpeedFSM;



static PinName_t seqCHn[2] = {IR1, IR2};





static volatile char BT_Data;

static volatile uint32_t value1 = 0;
static volatile uint32_t value2 = 0;

static uint32_t ovf_cnt = 0;
static float time1 = 0.f;
static float time2 = 0.f;
static float time_interval = 0.f;
static float distance = 0.f;



void setup(void);

void captine(void);
void alfred(void);
void jarvis(void);
void workout(void);


void emergency(void);

void LED_action(void);

int main(void)
{
        setup();
		msTicks = 0;
		uint8_t dir = UP;
		
		delay_ms(500);
	msTicks = 0;
	while(1)
	{
		printf("Welcome to the gym!\n");
    printf("Select the workout mode!\n");
    printf("(0=Captain mode, 1=Alfred mode, 2=Jarvis mode)\n");	
		if(BT_Data == '0')
		{
			printf("Enjoy your workout!\n");
			printf("When you leave, press reset button and clean the machine!\n");
			
		}
		else if(BT_Data == '1')
		{	
			alfred();		
			}
		else if(BT_Data == '2')
		{
		jarvis();	
		}
		distance = time_interval * 340.f / 2.f / 10.f;
	}
	
}

void setup(void)
{			
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
        // LED init
				GPIO_init(GPIOA, LED,1);
        GPIO_write(GPIOA,LED, LOW);
				
      

        // Motor init
        PWM_init(MOTOR1_PWM);
        PWM_period_us(MOTOR1_PWM, 500);
        PWM_duty(MOTOR1_PWM, 0.f);
				GPIO_init(GPIOA,MOTOR1_DIR,1);
        GPIO_write(GPIOA, MOTOR1_DIR, LOW);

        PWM_init(MOTOR2_PWM);
        PWM_period_us(MOTOR2_PWM, 500);
        PWM_duty(MOTOR2_PWM, 0.f);
        GPIO_init(GPIOA,MOTOR2_DIR,1);
        GPIO_write(GPIOA, MOTOR2_DIR, LOW);
				
				//EXTI SETTING FOR BUTTON	
				GPIO_init(GPIOC, emergency_button, INPUT);
				GPIO_pupd(GPIOC, emergency_button, 1);
				EXTI_enable(13);
				EXTI_init(GPIOC, emergency_button, FALL,13);
}

// workout mode fuctions
#define SET_number s
#define getsu_number n

void alfred()
{
	printf("you choose alfred mode!\n");
	delay_ms(1000);
	printf("In this mode, machine will notice your jase and count using monitor!\n");
	delay_ms(1000);
	printf("Please setting the 1set!\n");
	scanf("%d", &n);
	delay_ms(1000);
	printf("Your 1set is gusung with %d\n",n);
	delay_ms(1000);
	printf("Please setting set number!\n");
	scanf("%d", &s);
	delay_ms(1000);
	printf("Your set number is %d\n",s);
	delay_ms(1000);
	workout();
}

#define maximumheight max
#define minimumheight min
float max = 9;
float min = 3;

	


void jarvis()	
{
	printf("you choose jarvis!\n");
	delay_ms(1000);
	printf("In this mode, same like alfred, but alert when jase is bad!\n");
	delay_ms(1000);
	printf("Please setting the 1set!\n");
	scanf("%d", &n);
	delay_ms(1000);
	printf("Your 1set is gusung with %d\n",n);
	delay_ms(1000);
	printf("Please setting set number!\n");
	scanf("%d", &s);
	delay_ms(1000);
	printf("Your set number is %d\n",s);
	delay_ms(1000);
	workout();
				uint8_t right_color = value1 > 2000;
        uint8_t left_color = value2 > 2000;
	 if (right_color == left_color)		// for both sensor is not on line
        {
                printf("Good jase");
					delay_ms(1000);
        }
        else if (right_color)							// only  left sensor is on line, turn left
        {
                printf("BAD JASE!, danger on left!");
					delay_ms(1000);
        }
        else	if (left_color)														// only right  sensor is on line, turn right
        {
                printf("BAD JASE!, danger on right!");
					delay_ms(1000);
        }
				else
				{printf("if in danger, press btn!");
					delay_ms(1000);
	}
}

//emergency mode fuctions


void emergency()	
{
	printf("you pressed emergency button, escaping process start!\n");
	
								GPIO_write(GPIOA,11, UP);
                GPIO_write(GPIOA,12, UP);
                PWM_duty(MOTOR1_PWM, 0.4f);
                PWM_duty(MOTOR2_PWM, 0.4f);
}
#define D0 (uint8_t)0 //stm mumbers
#define D1 (uint8_t)1
#define D2 (uint8_t)2
typedef struct
{
        
        uint8_t next_state[1];
} GetsuFSM;
static uint8_t Getsu_state = D0;
static GetsuFSM Getsu_fsm[3] = { // speed stm
                {0,  {D0}},
                {0,  {D1}},
								{0,  {D2}},
};
void workout(){

	printf("jase jabgo, babell un choi go nob 2 e seo, start!");
	int Jigum_getsu = 0;
	int Jigum_set = 0;
	switch (Getsu_state) {
        case D0:
            if (distance >= max) {
                Getsu_state = D1;
            }
            break;
        case D1:
            if (distance <= min) {
                Getsu_state = D2;
            }
            break;
        case D2:
				if (distance >= max) {
                Getsu_state = D0;
					Jigum_getsu++;
            }
            break;
					}
    
						
	if(Jigum_getsu==n)
	{
		Jigum_set++;
		Jigum_getsu=0;
	}
	
	if(Jigum_set==s){
	printf("undong wahn ryo!!, jungri and press reset!");}
}



void USART1_IRQHandler(){
	 static volatile char new_line[2] = "\r\n";

        if (is_USART1_RXNE())
        {
                BT_Data = USART1_read();        // RX from UART1 (BT)

                USART1_write((uint8_t*) &BT_Data, 1);

                USART1_write((uint8_t*) new_line, 2);
        }
}

void ADC_IRQHandler(void)	
{
       static uint8_t flag = 0;

        if (is_ADC_OVR())
                clear_ADC_OVR();
        if (is_ADC_EOC())
        {        // after finishing sequence
                if (flag)
                        value2 = ADC_read();
                else
                        value1 = ADC_read();

                flag = !flag;
        }  
}

void TIM2_IRQHandler(void)
{
        if (is_UIF(TIM2))
        {
                ovf_cnt++;                                           // overflow count
                clear_UIF(TIM2);                               // clear update interrupt flag
        }
        if (is_CCIF(TIM2, IC_1))
        {
                time1 = (float) ICAP_capture(TIM2, IC_1); // Capture TimeStart
                clear_CCIF(TIM2, IC_1);                 // clear capture/compare interrupt flag
        }
        else if (is_CCIF(TIM2, IC_2))
        {
                time2 = (float) ICAP_capture(TIM2, IC_2); // Capture TimeEnd
                // (10 us * counter pulse -> [msec] unit) Total time of echo pulse
                time_interval = (time2 - time1 + (float) ovf_cnt) / 100.f;
                ovf_cnt = 0;                                         // overflow reset
                clear_CCIF(TIM2, IC_2);      
				}
}
void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(emergency_button)) {
		emergency();
		clear_pending_EXTI(emergency_button); 
	}
}
```

## 5. Result

https://youtu.be/AUOLD_tF8lM
![KakaoTalk_20231219_192004524_01](https://github.com/ZZERO1009/EC-ymkim-141/assets/144536736/e830283a-041b-4dd2-8dd3-a8cbe7137fae)
![KakaoTalk_20231219_192004524_02](https://github.com/ZZERO1009/EC-ymkim-141/assets/144536736/7cea9ef0-3b25-486b-b44e-b4fe8fe4f962)
![KakaoTalk_20231219_192004524_03](https://github.com/ZZERO1009/EC-ymkim-141/assets/144536736/24ced7e2-f3c4-401f-b6e3-fc0e83365ef4)
![KakaoTalk_20231219_192004524](https://github.com/ZZERO1009/EC-ymkim-141/assets/144536736/24c50d8f-bc67-4193-8379-c798b44646b2)

So sadly, project is not worked.

**Printf was not working on tera term**

check that it was same code, and was worked on other lab code.  Because starting code was that, entire was not worked

