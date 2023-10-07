# LAB: GPIO Digital InOut 7-segment

**Date: 2023.10.4.**

**Author/Partner:** 21900141 Youngmin Kim

**Github**: <https://github.com/ZZERO1009/EC-ymkim-141/blob/5132852617779f43b92299d6aeb5b2cbc81def34/docs/EC_GPIO_Digital_InOut_7-segment_21900141_Youngmin_Kim.md>


## Introduction

In this lab, I created a simple program to control a 7-segment display to show a decimal number (0~9) that increases by pressing a push-button.
## Requirement

### Hardware 

* MCU
  * NUCLEO-F401RE


* Actuator/Sensor/Others:
  * 7-segment display(5101ASR)
  * Array resistor (330 ohm)
  * decoder chip(74LS47)
  * breadboard

### Software
 * Keil uVision, CMSIS, EC_HAL library

## Problem 1 :

### Procedure
Connecting 7 seg to resistors(becausse my ckt draw prodram doesn't have array resistor, i used usual resistor).

### Connection Diagram
![스크린샷 2023-10-06 002149](https://github.com/ZZERO1009/EC-ymkim-141/assets/144536736/8f1775c0-d2a1-45d5-96e7-83733e0693fc)
This picture show connecting 7 seg to resistor, and to pin and show example that turn on "a".

### Discussion

 #### 1. Draw the truth table for the BCD 7-segment decoder with the 4-bit input 
![image](https://github.com/ZZERO1009/EC-ymkim-141/assets/144536736/80980b2a-6fb0-4828-ade3-eda60a7b00ba)

 #### 2. What are the common cathode and common anode of 7-segment display? 
Common anode is connected to VCC, and common cathode is connected to GND.

if want to turn on specific led on common anode 7seg, have to load "LOW" to pin of that number.

if want to turn on specific led on common cathode 7seg, have to load "HIGH" to pin of that number.

#### 3. Does the LED of a 7-segment display (common anode) pin turn ON when 'HIGH' is given to the LED pin from the MCU??

## Problem 2: Display 0~9 with button press

In this part, we have to set 7seg without decoder.

### Procedure
 #### Declare and Define the following functions in library
 First, in GPIO.h, i added code for define pins and function
 ```c++
// Pin definition, following configuration table
#define SEG_A_PIN 5
#define SEG_B_PIN 6
#define SEG_C_PIN 7
#define SEG_D_PIN 6
#define SEG_E_PIN 7
#define SEG_F_PIN 9
#define SEG_G_PIN 8
#define SEG_H_PIN 10

//Fuction for 7seg
void sevensegment_init(GPIO_TypeDef *Port, int pin, int Output);
//it's for setting pins to out
void sevensegment_decoder(uint8_t  num);
// it's for the number state
```
Next, on Gpio.c, adding led out to each number state.
 ```c++
void sevensegment_init(GPIO_TypeDef *Port, int pin, int Output){
	 GPIO_init(Port, pin, Output);
}

void sevensegment_decoder(uint8_t num) {
    if (num == 0) {
        GPIO_write(GPIOA, SEG_A_PIN, LOW);
        GPIO_write(GPIOA, SEG_B_PIN, LOW);
        GPIO_write(GPIOA, SEG_C_PIN, LOW);
        GPIO_write(GPIOB, SEG_D_PIN, LOW);
        GPIO_write(GPIOC, SEG_E_PIN, LOW);
        GPIO_write(GPIOA, SEG_F_PIN, LOW);
				GPIO_write(GPIOA, SEG_G_PIN, HIGH);
    } else if (num == 1) {
        GPIO_write(GPIOA, SEG_A_PIN, HIGH);
        GPIO_write(GPIOA, SEG_B_PIN, LOW);
        GPIO_write(GPIOA, SEG_C_PIN, LOW);
        GPIO_write(GPIOB, SEG_D_PIN, HIGH);
        GPIO_write(GPIOC, SEG_E_PIN, HIGH);
        GPIO_write(GPIOA, SEG_F_PIN, HIGH);
 // others follow truth tabe of 7seg too.
```

 #### Declare and Define main code.
Set main code for 7 seg runnig.
```c++
#include "..\..\include\ecGPIO.h"
#include "stm32f4xx.h"
#include "..\..\include\ecRCC.h"

unsigned int cnt = 0;
//cnt is 0~9, it goes to display. 
int buttonState = 0;
int lastButtonState = 0;
//This codes is gor define state for number

void setup(void);
void updateDisplay(void);
// we will put "void sevensegment_decoder(uint8_t  num);" to here, when cnt is change, this fucntion will put cnt to "num"

int main(void) {
    setup();
    
    while (1) {
        buttonState = GPIO_read(GPIOC, BUTTON_PIN);
         // by GPIO read, buttonState will updated by button
        if (buttonState != lastButtonState) {
            if (buttonState == LOW) {
                cnt++;
                if (cnt > 9) {
                    cnt = 0;
               // cnt followe the act of button by code here, when it over 9, goes to zero and cycle is reset
                }
            }
            lastButtonState = buttonState;
        }
        updateDisplay();
    }
}

void setup(void) {
    RCC_HSI_init();
		GPIO_init(GPIOC, BUTTON_PIN, 0UL);
    GPIO_pupd(GPIOC, BUTTON_PIN, 2UL);
    sevensegment_init(GPIOA, SEG_A_PIN, 1UL);
    sevensegment_init(GPIOA, SEG_B_PIN, 1UL);
    sevensegment_init(GPIOA, SEG_C_PIN, 1UL);
    sevensegment_init(GPIOB, SEG_D_PIN, 1UL);
    sevensegment_init(GPIOC, SEG_E_PIN, 1UL);
    sevensegment_init(GPIOA, SEG_F_PIN, 1UL);
    sevensegment_init(GPIOA, SEG_G_PIN, 1UL);
    sevensegment_init(GPIOB, SEG_H_PIN, 1UL);
    // button is set by code ( following configuration), seach mcu pins we have to use are setted to out.
}

void updateDisplay(void) {
    sevensegment_decoder(cnt);
  // This code update num(following cnt).
}

```
### Result
<https://youtube.com/shorts/zvV-xIsL1n0>

## Problem 3: Using both 7-Segment Decoder and 7-segment display
In this part, add 74LS47, connect mcu out to decoder in, decoder out to 7 seg in.

### Procedure

### Connection Diagram
![스크린샷 2023-10-08 000335](https://github.com/ZZERO1009/EC-ymkim-141/assets/144536736/7de0820c-4190-4769-8863-dc6d9bdb0dc7)
Because tinker dosen't have nucleo f411re, set mcu pin that start at d5 in this picture.

 #### Declare and Define the following functions in library
 First, Change GPIO.h for pin definition and display on.
 ```c++
// Pin definition, following configuration table
#define SEG_A_PIN 7
#define SEG_B_PIN 6
#define SEG_C_PIN 7
#define SEG_D_PIN 9
```
And then, chage GPIO.c to change fuction

 ```c++
void sevensegment_decoder(uint8_t num) {
// Because we use decoder, change decoder fuction to use 4 input
  
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


 // others follow truth tabe of 7seg too.
```

 #### Change main code.
We just have to chanege setup code
```c++
void setup(void) {
    RCC_HSI_init();
		GPIO_init(GPIOC, BUTTON_PIN, 0UL);
    GPIO_pupd(GPIOC, BUTTON_PIN, 2UL);
    sevensegment_init(GPIOA, SEG_A_PIN, 1UL);
    sevensegment_init(GPIOA, SEG_B_PIN, 1UL);
    sevensegment_init(GPIOA, SEG_C_PIN, 1UL);
    sevensegment_init(GPIOB, SEG_D_PIN, 1UL);
}
```
### Result
<https://youtube.com/shorts/lCrwXU5NIAs?feature=share>

