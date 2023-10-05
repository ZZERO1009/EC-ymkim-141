# LAB: GPIO Digital InOut 7-segment

**Date: 2023.10.4.**

**Author/Partner:** 21900141 Youngmin Kim

**Github**: <https://github.com/ZZERO1009/LAB1_mini_fan/blob/aa66b9efee4b68b8aaafcd5549caab45be376240/EC_LAB_miniFan_report_21900141_Youngmin_Kim.md>


## Introduction

n this lab, I createD a simple program to control a 7-segment display to show a decimal number (0~9) that increases by pressing a push-button.
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


### Connection Diagram
![스크린샷 2023-10-06 002149](https://github.com/ZZERO1009/EC-ymkim-141/assets/144536736/8f1775c0-d2a1-45d5-96e7-83733e0693fc)


### Discussion
 #### 1. Draw the truth table for the BCD 7-segment decoder with the 4-bit input 
Trigger:
  * Generate a trigger pulse as PWM to the sensor
  * Pin: D10 (TIM4 CH1)
  * PWM out: 50ms period, 10us pulse-width

Echo:
  * Receive echo pulses from the ultrasonic sensor
  * Pin: D7 (Timer1 CH1)
  * Input Capture: Input mode
  * Measure the distance by calculating pulse-width of the echo pulse.

#### USART
  * Display measured distance in [cm] on serial monitor of Tera-Term.
  * Baudrate 9600

#### DC Motor
  * PWM: PWM1, set 10ms of period by default
  * Pin: D11 (Timer1 CH1N)

## Circuit/Wiring Diagram

![Sizzling Fyyran-Tumelo p](https://github.com/ZZERO1009/LAB1_mini_fan/assets/144536736/e715fbb8-840c-4bf8-8810-df31ad490b2f)

## Algorithm

### Overview

![스크린샷(5)](https://github.com/ZZERO1009/LAB1_mini_fan/assets/144536736/0c661558-2b76-491a-b537-7ac51408d5d7)


### Mealy FSM Table

![스크린샷(4)](https://github.com/ZZERO1009/LAB1_mini_fan/assets/144536736/eb7bf604-6bbd-4e7c-864a-5d35422cb300)

### Description with Code

##### State definition
Based on the investigation results, it has been determined that a configuration consisting of only essential states is appropriate. 

This configuration comprises three states: S0, S1, and S2, defined as follows: S0 = 0ff, S1 = 50% (122), and S2 = 100% (255).

```c++
// State definition
#define S0  0
#define S1  1
#define S2  2
```

#### FSM

Mealy FSM to code

```c++
// State table definition
typedef struct {
  unsigned int next[4];       
  // next state = FSM[state].next[input]
  unsigned int out[4][2];     
  // output = FSM[state].out[input]
} State_t;

State_t FSM[3] = {
    {{S0, S0, S1, S1}, {{0, LOW}, {0, LOW}, {0, HIGH}, {122, HIGH}}},
    {{S1, S1, S2, S2}, {{0, HIGH}, {122, HIGH}, {0, HIGH}, {225, HIGH}}},
    {{S2, S2, S0, S0}, {{0, HIGH}, {225, HIGH}, {0, LOW}, {0, LOW}}},
};
```
#### Settings for in and out
```c++

const int ledPin = 13;
const int pwmPin = 11;
const int btnPin = 3;
const int trigPin = 10;
const int echoPin = 7;

unsigned char state = S0;
unsigned char input[2] = {0, 0};
unsigned char pwmOut = 0;
unsigned char ledOut = LOW;

unsigned long duration;
float distance;
int thresh = 5;

// Timer variables (for led blink)
unsigned long previousMillis = 0;
const long interval = 1000;
```

#### Loop
this part of the code,measures the distance using sensor and stores it in the 'distance' variable. 

Then, it calls the 'nextState()' to calculate and update the next state. also calls the 'stateOutput()'fetch the PWM out and LED based on the current state. 

Additionally, it periodically prints the distance + PWM ratio to the  monitor at regular time intervals.

```c++

void loop() {
  // Generate PWM signal on trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);

  // Calculate distance using the duration of the echo pulse
  duration = pulseIn(echoPin, HIGH);
  distance = (float)duration / 58.0;

  // Calculate next state, then update state
  nextState();

  // State output
  stateOutput();
  
  // Print distance and PWM duty ratio to Tera-Term console every 1 second
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.print("Distance = ");
    Serial.print(distance);
    Serial.print(" [cm], PWM duty cycle = ");
    Serial.print(pwmOut);
    Serial.println("%");
  }
  
  analogWrite(pwmPin, pwmOut);
}
```
#### Btn pressed
In the overall code, it's evident that the transition to the next state is triggered by the 'btn' press. 

This transition involves moving from one state to another, 

such as transitioning from 'S0' to 'S1'(rather than returning to the same state like 'S0' to 'S0').

```c++
void pressed(){
  input[0] = 1;
  nextState();
  input[0] = 0;
}
```
#### Next state
calculates the next state using the FSM code.
```c++
void nextState(){
  if (distance < thresh)
    input[1] = 1;
  else
    input[1] = 0;
    
  // Get next state
  state = FSM[state].next[input[0] * 2 + input[1]];  
  // Modified input calculation
}
```

#### Output


based on present state and input, It configures the movement of the motor and controls the blinking of the LED.
```c++
void stateOutput(){
  pwmOut = FSM[state].out[input[0] * 2 + input[1]][PWM];  
  // Modified input calculation
  ledOut = FSM[state].out[input[0] * 2 + input[1]][LED];  
  // Modified input calculation
}
```

#### Timer
this part is for the led blink(distance is close enough, sec 1).

```c++
// Code for led blink
void timerInterrupt() {
  // Check the current MODE and control the LED accordingly
 
  if (state == S0) {
    digitalWrite(ledPin, LOW);  
    // Turn off LED when MODE is OFF
  } else {
    static unsigned long 
    previousMillis = 0;
    
    unsigned long currentMillis = millis();
    
    if (currentMillis - previousMillis >= interval / 2) {
      previousMillis = currentMillis;
      digitalWrite(ledPin, !digitalRead(ledPin));  
      // Blink the LED every 1 second
    }
  }
}
```

## Results and Analysis

### Result

The final code has been written to meet various conditions, and it has been updated to the Nucleo board for testing its real-world functionality. 

Additionally, hardware configurations such as pin connections have been set up to ensure proper operation.

![KakaoTalk_20230910_015238566_02](https://github.com/ZZERO1009/LAB1_mini_fan/assets/144536736/7c8b2a7a-f8da-4f47-862e-5fcdc2596656)
![KakaoTalk_20230910_015238566_03](https://github.com/ZZERO1009/LAB1_mini_fan/assets/144536736/83b2409f-ee04-4b6d-9a6c-cde054b3313f)
![KakaoTalk_20230910_015238566_01](https://github.com/ZZERO1009/LAB1_mini_fan/assets/144536736/a70e26ad-b65d-45fb-ac39-284faf51e8e5)

### Demo video

<https://youtu.be/k742ZgKgNyA>

### Analysis

In the final stage, the code based on the state table was confirmed to be operational through hardware connections. To ensure accuracy, the process was divided into two parts in the video recording.

The first part involved pressing the button when the distance was close to verify the results. 

In the second part, the button was pressed when the distance was far, and then the distance was reduced to ensure that the states were functioning correctly.

However, there were some challenges encountered during the experiment, particularly with the ultrasonic sensor and the DC motor. 

As observed in the video, the sensor occasionally displayed a distance reading of 7 cm even when there was nothing in front of it, and at times, 

it would suddenly display 6 or 7 cm readings when the distance was intentionally reduced.


Overall, the experimentation revealed some challenges with the ultrasonic sensor, and further troubleshooting may be necessary to address these issues and ensure accurate results.
