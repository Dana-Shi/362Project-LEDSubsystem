/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon, Dana Shi
  * @date    Nov 2024
  * @brief   ECE 362 Project - LED Subsystem
  ******************************************************************************
*/

/*******************************************************************************/ 

#include "stm32f0xx.h"
#include <math.h>   // for M_PI

void nano_wait(int);

// 16-bits per digit.
// The most significant 8 bits are the digit number.
// The least significant 8 bits are the segments to illuminate.
uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };
extern const char font[];
// Print an 8-character string on the 8 digits
void print(const char str[]);
// Print a floating-point value.
void printfloat(float f);

//============================================================================
// PWM Lab Functions
//============================================================================

void setup_tim1(void) {
    // Generally the steps are similar to those in setup_tim3
    // except we will need to set the MOE bit in BDTR. 
    // Be sure to do so ONLY after enabling the RCC clock to TIM1.
    RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;   //turn on TIM1
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;    //turn on GPIOA

    GPIOA -> MODER &= 0xff00ffff;   //clear PA8-PA11 
    GPIOA -> MODER |= 0x00aa0000;   //set PA8-PA11 to alternate function
    GPIOA -> AFR[1] |= 0x2;     //TIM1, channel 1, PA8, AF2
    GPIOA -> AFR[1] |= 0x2 << (1*4);     //TIM1, channel 2, PA9, AF2
    GPIOA -> AFR[1] |= 0x2 << (2*4);     //TIM1, channel 3, PA10, AF2
    GPIOA -> AFR[1] |= 0x2 << (3*4);     //TIM1, channel 4, PA11, AF2

    TIM1 -> BDTR |= TIM_BDTR_MOE;   //enable MOE bit in TIM1

    TIM1 -> PSC = 1 - 1;
    TIM1 -> ARR = 2400 - 1;

    TIM1 -> CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;   //channel 1, PWM mode 1 = 110 //RED
    TIM1 -> CCMR1 &= ~TIM_CCMR1_OC1M_0;

    TIM1 -> CCER |= TIM_CCER_CC1E;  //enable channel 1 output

    TIM1 -> CR1 |= TIM_CR1_CEN;     //enable timer 1
}

void dialer(void);

// Parameters for the wavetable size and expected synthesis rate.
#define N 1000
#define RATE 20000
short int wavetable[N];
int step0 = 0;
int offset0 = 0;
int step1 = 0;
int offset1 = 0;

//============================================================================
// enable_ports()
//============================================================================
void enable_ports(void) {
    RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC -> AHBENR |= RCC_AHBENR_GPIOCEN;

    GPIOB -> MODER &= 0xffc00000;
    GPIOB -> MODER |= 0x00155555;

    GPIOC -> MODER &= 0xffff0000;
    GPIOC -> MODER |= 0x00005500;

    GPIOC -> OTYPER = (0xF << 4);

    GPIOC -> PUPDR &= 0xffffff00;
    GPIOC -> PUPDR |= 0x00000055;        
}

uint8_t col; // the column being scanned

//===========================================================================
void init_wavetable(void) {
    for(int i=0; i < N; i++)
        wavetable[i] = 32767 * sin(2 * M_PI * i / N);
}

int main(void) {
    internal_clock();

    enable_ports();
    init_wavetable();

    setup_tim1();

    float count = 0;

    for(;;) {
        // Breathe in...

        for(float x=1; x<100000; x *= (1.1*count)) {
            TIM1->CCR1 = 100000-x;
            nano_wait(100000000);

            count = count + 0.5; 
        }
        // ...and out...
        for(float x=100000; x>=1; x /= (1.1*count)) {
            TIM1->CCR1 = 100000-x;
            nano_wait(100000000);
        }

        // ...and start over.
    }

    // Have fun.
    dialer();
}
