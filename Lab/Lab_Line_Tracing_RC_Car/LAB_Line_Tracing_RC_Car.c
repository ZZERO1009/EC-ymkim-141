/**
 * @author Youngmin Kim & Jarry Goon
 * @Created 2023-11-20
 * @Modified 2023-11-20
 for RC car Working code
*/

#include "..\..\lib\ecSTM32F411.h"

#define S0 (uint8_t)0 //stm mumbers
#define S1 (uint8_t)1
#define S2 (uint8_t)2
#define S3 (uint8_t)3
#define S4 (uint8_t)4
#define S5 (uint8_t)5
#define S6 (uint8_t)6

#define FORWORD 1
#define BACKWORD 0

#define MANUAL 0
#define AUTO 1

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

#define MOTOR_SPEED(DIR, SPEED) fabsf((float)(DIR) - (float)(SPEED))

typedef struct
{
        float speed;
        uint8_t next_state[2];
} SpeedFSM;

typedef struct
{
        float right_speed_ratio;
        float left_speed_ratio;
        uint8_t next_state[2];
} AngleFSM;

static PinName_t seqCHn[2] = {IR1, IR2};

static float speed;
static uint8_t speed_state = S3;
static SpeedFSM speed_fsm[7] = { // speed stm
                {-0.9f,  {S1, S0}},
                {-0.65f, {S2, S0}},
                {-0.4f,  {S3, S1}},
                {0.f,    {S4, S2}},
                {0.4f,   {S5, S3}},
                {0.65f,  {S6, S4}},
                {0.9f,   {S6, S5}}
};

static float right_angle;
static float left_angle;
static uint8_t angle_state = S3;
static AngleFSM angle_fsm[7] = { // angle stm
                {1.f,   0.5f,  {S1, S0}},
                {1.f,   0.65f, {S2, S0}},
                {1.f,   0.8f,  {S3, S1}},
                {1.f,   1.f,   {S4, S2}},
                {0.8f,  1.f,   {S5, S3}},
                {0.65f, 1.f,   {S6, S4}},
                {0.5f,  1.f,   {S6, S5}}
};

static volatile char BT_Data;

static volatile uint32_t value1 = 0;
static volatile uint32_t value2 = 0;

static uint8_t control_mode = MANUAL;

static uint32_t ovf_cnt = 0;
static float time1 = 0.f;
static float time2 = 0.f;
static float time_interval = 0.f;
static float distance = 0.f;

void setup(void);

void manual(void);

void automatic(void);

void LED_action(void);

int main()
{
        setup();

        msTicks = 0;

        uint8_t dir = FORWORD;

        while (1)
        {					//For mode chage
                if (BT_Data == '7')
                {
                        if (control_mode)
                        {
                                speed = 0.f;
                                control_mode = MANUAL;
                        }
                        else
                        {
                                speed = 0.9f;
                                dir = FORWORD;
                                speed_state = S3;
                                angle_state = S3;
                                control_mode = AUTO;
                        }

                        BT_Data = '\0';
                }
								//for check distance to emergency stop on auto mode
                distance = time_interval * 340.f / 2.f / 10.f;

                switch (control_mode)
                {
                        case MANUAL:
                                manual();
                                break;

                        case AUTO:
                                automatic();
                                break;

                        default:
                                break;
                }

                if (speed >= 0) dir = FORWORD;
                else dir = BACKWORD;
								GPIO_write(GPIOA,11, dir);
                GPIO_write(GPIOA,12, dir);
                PWM_duty(MOTOR1_PWM, MOTOR_SPEED(dir, speed * left_angle));
                PWM_duty(MOTOR2_PWM, MOTOR_SPEED(dir, speed * right_angle));
								
								
								
								
                
                

                LED_action();
        }
}

void setup(void)
{				// other settings are on ecSTM411re, in can be changed later
				MCU_init();
       

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
}

void manual()
{												// on manual mode, numpad keyboard is control tower, 7 is mode key, 5 is emergency key
        switch (BT_Data)
        {
                case '8':
                        speed_state = speed_fsm[speed_state].next_state[0];
                        break;

                case '4':
                        angle_state = angle_fsm[angle_state].next_state[1];
                        break;

                case '6':
                        angle_state = angle_fsm[angle_state].next_state[0];
                        break;

                case '2':
                        speed_state = speed_fsm[speed_state].next_state[1];
                        break;

                case '5':
                        speed_state = S3;
                        angle_state = S3;

                default:
                        break;
        }

        speed = speed_fsm[speed_state].speed;
        right_angle = angle_fsm[angle_state].right_speed_ratio;
        left_angle = angle_fsm[angle_state].left_speed_ratio;

        BT_Data = '\0';
}

void automatic()	//auto mode code
{
        speed = 0.9f; //auto defalut speed

					// color code is for turn, value 2000 is come from sensor checked
        uint8_t right_color = value1 > 2000;
        uint8_t left_color = value2 > 2000;

        if (distance < 5.f)
        {
                speed = 0.f;
                return;
        }

        if (right_color == left_color)		// for both sensor is not on line
        {
                left_angle = 1.f;
                right_angle = 1.f;
        }
        else if (right_color)							// only  left sensor is on line, turn left
        {
                left_angle = 0.0f;
                right_angle = 1.f;
        }
        else															// only right  sensor is on line, turn right
        {
                left_angle = 1.f;
                right_angle = 0.0f;
        }
}

void LED_action()																							// LED have to be differrent when mode is changed.
{
        switch (control_mode)
        {
                case MANUAL:
                        GPIO_write(GPIOA,LED, HIGH);
                        break;

                case AUTO:
                        if (msTicks >= 1000)
                        {
                                msTicks = 0;
                               LED_toggle();
                        }
                        break;

                default:
                        break;
        }
}

void USART1_IRQHandler()
{
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
                clear_CCIF(TIM2, IC_2);                 // clear capture/compare interrupt flag
        }
}