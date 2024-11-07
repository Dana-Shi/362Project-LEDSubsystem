/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon
  * @date    Jan 31 2024
  * @brief   ECE 362 Lab 5 Student template
  ******************************************************************************
*/

/*******************************************************************************/

// Fill out your username, otherwise your completion code will have the 
// wrong username!
const char* username = "shi562";

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

void autotest(void);

//============================================================================
// PWM Lab Functions
//============================================================================
void setup_tim3(void) {
    RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN;   //turn on TIM3
    RCC -> AHBENR |= RCC_AHBENR_GPIOCEN;    //turn on GPIOC

    GPIOC -> MODER &= 0xfff00fff;   //clear PC6-PC9 
    GPIOC -> MODER |= 0x000aa000;   //set PC6-PC9 to alternate function
    GPIOC -> AFR[0] &= (~0xf) << (4*6);     //TIM3, channel 1, PC6, AF0, RED LED
    GPIOC -> AFR[0] &= (~0xf) << (4*7);     //TIM3, channel 2, PC7, AF0, YELLOW LED
    GPIOC -> AFR[1] &= (~0xf);     //TIM3, channel 3, PC8, AF0, GREEN LED
    GPIOC -> AFR[1] &= (~0xf) << 4;     //TIM3, channel 4, PC9, AF0, BLUE LED

    TIM3 -> PSC = 48000 - 1;
    TIM3 -> ARR = 1000 - 1;

    TIM3 -> CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;   //channel 1, PWM mode 1 = 110
    TIM3 -> CCMR1 &= ~TIM_CCMR1_OC1M_0;
    TIM3 -> CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;   //channel 2, PWM mode 1 = 110
    TIM3 -> CCMR1 &= ~TIM_CCMR1_OC2M_0;
    TIM3 -> CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;   //channel 3, PWM mode 1 = 110
    TIM3 -> CCMR2 &= ~TIM_CCMR2_OC3M_0;
    TIM3 -> CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;   //channel 4, PWM mode 1 = 110
    TIM3 -> CCMR2 &= ~TIM_CCMR2_OC4M_0; 

    TIM3 -> CCER |= TIM_CCER_CC1E;  //enable channel 1 output
    TIM3 -> CCER |= TIM_CCER_CC2E;  //enable channel 2 output
    TIM3 -> CCER |= TIM_CCER_CC3E;  //enable channel 3 output
    TIM3 -> CCER |= TIM_CCER_CC4E;  //enable channel 4 output
    TIM3 -> CR1 |= TIM_CR1_CEN;     //enable timer 3

    TIM3 -> CCR1 = 800;     
    TIM3 -> CCR2 = 600;
    TIM3 -> CCR3 = 400;
    TIM3 -> CCR4 = 200; 
}

void setup_tim1(void) {
    // Generally the steps are similar to those in setup_tim3
    // except we will need to set the MOE bit in BDTR. 
    // Be sure to do so ONLY after enabling the RCC clock to TIM1.
    RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;   //turn on TIM1
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;    //turn on GPIOA

    GPIOA -> MODER &= 0xff00ffff;   //clear PA8-PA11 
    GPIOA -> MODER |= 0x00aa0000;   //set PA8-PA11 to alternate function
    //GPIOA -> AFR[1] &= (~0xf); 
    GPIOA -> AFR[1] |= 0x2;     //TIM1, channel 1, PA8, AF2
    //GPIOA -> AFR[1] &= (~0xf) << (1*4); 
    GPIOA -> AFR[1] |= 0x2 << (1*4);     //TIM1, channel 2, PA9, AF2
    //GPIOA -> AFR[1] &= (~0xf) << (2*4); 
    GPIOA -> AFR[1] |= 0x2 << (2*4);     //TIM1, channel 3, PA10, AF2
    //GPIOA -> AFR[1] &= (~0xf) << (3*4); 
    GPIOA -> AFR[1] |= 0x2 << (3*4);     //TIM1, channel 4, PA11, AF2

    TIM1 -> BDTR |= TIM_BDTR_MOE;   //enable MOE bit in TIM1

    TIM1 -> PSC = 1 - 1;
    TIM1 -> ARR = 2400 - 1;

    TIM1 -> CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;   //channel 1, PWM mode 1 = 110
    TIM1 -> CCMR1 &= ~TIM_CCMR1_OC1M_0;
    TIM1 -> CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;   //channel 2, PWM mode 1 = 110
    TIM1 -> CCMR1 &= ~TIM_CCMR1_OC2M_0;
    TIM1 -> CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;   //channel 3, PWM mode 1 = 110
    TIM1 -> CCMR2 &= ~TIM_CCMR2_OC3M_0;
    TIM1 -> CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;   //channel 4, PWM mode 1 = 110
    TIM1 -> CCMR2 &= ~TIM_CCMR2_OC4M_0; 
    TIM1 -> CCMR2 |= TIM_CCMR2_OC4PE;   //channel 4, enalbe "output compare preload enable"

    TIM1 -> CCER |= TIM_CCER_CC1E;  //enable channel 1 output
    TIM1 -> CCER |= TIM_CCER_CC2E;  //enable channel 2 output
    TIM1 -> CCER |= TIM_CCER_CC3E;  //enable channel 3 output
    TIM1 -> CCER |= TIM_CCER_CC4E;  //enable channel 4 output
    TIM1 -> CR1 |= TIM_CR1_CEN;     //enable timer 1
}

int getrgb(void);

// Helper function for you
// Accept a byte in BCD format and convert it to decimal
uint8_t bcd2dec(uint8_t bcd) {
    // Lower digit
    uint8_t dec = bcd & 0xF;

    // Higher digit
    dec += 10 * (bcd >> 4);
    return dec;
}

void setrgb(int rgb) {
    uint8_t b = bcd2dec(rgb & 0xFF);
    uint8_t g = bcd2dec((rgb >> 8) & 0xFF);
    uint8_t r = bcd2dec((rgb >> 16) & 0xFF);

    // TODO: Assign values to TIM1->CCRx registers
    // Remember these are all percentages
    // Also, LEDs are on when the corresponding PWM output is low
    // so you might want to invert the numbers.

    TIM1 -> CCR1 = ((TIM1 -> ARR) + 1) - ((r/100.0) * ((TIM1 -> ARR) + 1));
    TIM1 -> CCR2 = ((TIM1 -> ARR) + 1) - ((g/100.0) * ((TIM1 -> ARR) + 1));
    TIM1 -> CCR3 = ((TIM1 -> ARR) + 1) - ((b/100.0) * ((TIM1 -> ARR) + 1));
} 



//============================================================================
// Lab 4 code
// Add in your functions from previous lab
//============================================================================

// Part 3: Analog-to-digital conversion for a volume level.
int volume = 2400;

// Variables for boxcar averaging.
#define BCSIZE 32
int bcsum = 0;
int boxcar[BCSIZE];
int bcn = 0;

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

//============================================================================
// setup_dma() + enable_dma()
//============================================================================
void setup_dma(void) {
    RCC -> AHBENR |= RCC_AHBENR_DMA1EN;
 
    DMA1_Channel5 -> CMAR = (uint32_t)msg;
    DMA1_Channel5 -> CPAR = (uint32_t)(&(GPIOB -> ODR));
    DMA1_Channel5 -> CNDTR = 8;
    DMA1_Channel5 -> CCR |= DMA_CCR_DIR;
    DMA1_Channel5 -> CCR |= DMA_CCR_MINC;
    DMA1_Channel5 -> CCR &= ~DMA_CCR_MSIZE; 
    DMA1_Channel5 -> CCR |= DMA_CCR_MSIZE_0;
    DMA1_Channel5 -> CCR &= ~DMA_CCR_PSIZE;
    DMA1_Channel5 -> CCR |= DMA_CCR_PSIZE_0;
    DMA1_Channel5 -> CCR |= DMA_CCR_CIRC;
}

void enable_dma(void) {
    DMA1_Channel5 -> CCR |= DMA_CCR_EN;
}

//============================================================================
// init_tim15()
//============================================================================
void init_tim15(void) {
    RCC -> APB2ENR |= RCC_APB2ENR_TIM15EN;
    TIM15 -> PSC = 2400 - 1;
    TIM15 -> ARR = 20 - 1;
    TIM15 -> DIER |= TIM_DIER_UDE;
    TIM15 -> CR1 |= TIM_CR1_CEN;
}

//=============================================================================
// Part 2: Debounced keypad scanning.
//=============================================================================

uint8_t col; // the column being scanned

void drive_column(int);   // energize one of the column outputs
int  read_rows();         // read the four row inputs
void update_history(int col, int rows); // record the buttons of the driven column
char get_key_event(void); // wait for a button event (press or release)
char get_keypress(void);  // wait for only a button press event.
float getfloat(void);     // read a floating-point number from keypad
void show_keys(void);     // demonstrate get_key_event()

//============================================================================
// The Timer 7 ISR
//============================================================================
void TIM7_IRQHandler(){
    TIM7 -> SR &= ~TIM_SR_UIF;  // acknowledge the interrupt 
    int rows = read_rows();
    update_history(col, rows);
    col = (col + 1) & 3;
    drive_column(col);
}


//============================================================================
// init_tim7()
//============================================================================
void init_tim7(void) {
    RCC -> APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7 -> PSC = 2400 - 1;
    TIM7 -> ARR = 20 - 1;
    TIM7 -> DIER |= TIM_DIER_UIE;
    NVIC -> ISER[0] = 1 << TIM7_IRQn;
    TIM7 -> CR1 |= TIM_CR1_CEN;
}

//============================================================================
// setup_adc()
//============================================================================
void setup_adc(void) {
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;    //enable clock to GPIOA
    GPIOA -> MODER |= 0xC;
    RCC -> APB2ENR |= RCC_APB2ENR_ADCEN;    //enable clock to ADC peripheral
    RCC -> CR2 |= RCC_CR2_HSI14ON;  //turn on high speed internal 14MHz clock
    while(!(RCC -> CR2 & RCC_CR2_HSI14RDY));     //wait for 14MHz clock to be ready
    ADC1 -> CR |= ADC_CR_ADEN;  //enable ADC
    while(!(ADC1 -> ISR & ADC_ISR_ADRDY));   //wait for ADC to be ready
    ADC1 -> CHSELR = 0;
    ADC1 -> CHSELR = 1 << 1;    //PA1
    while(!(ADC1 -> ISR & ADC_ISR_ADRDY));   //wait for ADC to be ready
    ADC1 -> CR |= ADC_CR_ADSTART;   //start ADC
    while(!(ADC1 -> ISR & ADC_ISR_EOC));    //wait for end of conversion
}


//============================================================================
// Timer 2 ISR
//============================================================================
// Write the Timer 2 ISR here.  Be sure to give it the right name.
void TIM2_IRQHandler(){
    TIM2 -> SR &= ~TIM_SR_UIF;  // acknowledge the interrupt 
    ADC1 -> CR |= ADC_CR_ADSTART;
    while(!(ADC1 -> ISR & ADC_ISR_EOC));    //wait until the EOC bit is set in the ISR
    bcsum -= boxcar[bcn];
    bcsum += boxcar[bcn] = ADC1->DR;
    bcn += 1;
    if (bcn >= BCSIZE){
        bcn = 0;
    }
    volume = bcsum / BCSIZE;

    for(int x=0; x<10000; x++)  //step 6
    ;
}

//============================================================================
// init_tim2()
//============================================================================
void init_tim2(void) {
    RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2 -> PSC = 24000 - 1;
    TIM2 -> ARR = 200 - 1;
    TIM2 -> DIER |= TIM_DIER_UIE;
    NVIC -> ISER[0] = 1 << TIM2_IRQn;
    TIM2 -> CR1 |= TIM_CR1_CEN;
    NVIC_SetPriority(TIM2_IRQn, 3);
}



//============================================================================
// setup_dac()
//============================================================================
void setup_dac(void) {
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA -> MODER &= ~0x300;
    GPIOA -> MODER |= 0x300;    //set analog config for DAC_OUT1 (PA4)
    RCC -> APB1ENR |= RCC_APB1ENR_DACEN;  //enable RCC clock for DAC
    DAC -> CR &= ~DAC_CR_EN1; 
    DAC -> CR &= ~DAC_CR_TSEL1;    //set TIM6 TRG0 trigger for TSEL
    DAC -> CR |= DAC_CR_TEN1;   //enable trigger for DAC
    DAC -> CR |= DAC_CR_EN1;    //enable DAC

}

//============================================================================
// Timer 6 ISR
//============================================================================
// Write the Timer 6 ISR here.  Be sure to give it the right name.
void TIM6_DAC_IRQHandler(){
    TIM6 -> SR &= ~TIM_SR_UIF;  // acknowledge the interrupt 
    offset0 += step0;
    offset1 += step1;

    if(!(offset0 < (N << 16)))
    {
        offset0 -= (N << 16);
    }

    if(!(offset1 < (N << 16)))
    {
        offset1 -= (N << 16);
    }

    int samp = wavetable[offset0 >> 16] + wavetable[offset1 >> 16];
    // samp = samp * volume;
    // samp = samp >> 17;
    // samp += 2048;
    samp = ((samp * volume)>>18) + 1200;
    TIM1 -> CCR4 = samp;
}

//============================================================================
// init_tim6()
//============================================================================
void init_tim6(void) {
    RCC -> APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6 -> PSC = (100000 / RATE) - 1;
    TIM6 -> ARR = 480 - 1;
    TIM6 -> DIER |= TIM_DIER_UIE;
    NVIC -> ISER[0] = 1 << TIM6_DAC_IRQn;
    TIM6 -> CR1 |= TIM_CR1_CEN;
    TIM6 -> CR2 &= ~TIM_CR2_MMS_1;
    //TIM6 -> CR2 |= TIM_CR2_MMS_1;  //enable TRG0 on update event
} 

//===========================================================================
// init_wavetable()
// Write the pattern for a complete cycle of a sine wave into the
// wavetable[] array.
//===========================================================================
void init_wavetable(void) {
    for(int i=0; i < N; i++)
        wavetable[i] = 32767 * sin(2 * M_PI * i / N);
}


//============================================================================
// set_freq()
//============================================================================
void set_freq(int chan, float f) {
    if (chan == 0) {
        if (f == 0.0) {
            step0 = 0;
            offset0 = 0;
        } else
            step0 = (f * N / RATE) * (1<<16);
    }
    if (chan == 1) {
        if (f == 0.0) {
            step1 = 0;
            offset1 = 0;
        } else
            step1 = (f * N / RATE) * (1<<16);
    }
}


//============================================================================
// All the things you need to test your subroutines.
//============================================================================
int main(void) {
    internal_clock();

    // Uncomment autotest to get the confirmation code.
    autotest();

    // Demonstrate part 1
// #define TEST_TIMER3
#ifdef TEST_TIMER3
    setup_tim3();
    for(;;) { }
#endif

    // Initialize the display to something interesting to get started.
    msg[0] |= font['E'];
    msg[1] |= font['C'];
    msg[2] |= font['E'];
    msg[3] |= font[' '];
    msg[4] |= font['3'];
    msg[5] |= font['6'];
    msg[6] |= font['2'];
    msg[7] |= font[' '];

    enable_ports();
    setup_dma();
    enable_dma();
    init_tim15();
    init_tim7();
    setup_adc();
    init_tim2();
    init_wavetable();
    init_tim6();

    setup_tim1();

    // demonstrate part 2
// #define TEST_TIM1
#ifdef TEST_TIM1
    for(;;) {
        // Breathe in...
        for(float x=1; x<2400; x *= 1.1) {
            TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 2400-x;
            nano_wait(100000000);
        }
        // ...and out...
        for(float x=2400; x>=1; x /= 1.1) {
            TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 2400-x;
            nano_wait(100000000);
        }
        // ...and start over.
    }
#endif

    // demonstrate part 3
//#define MIX_TONES
#ifdef MIX_TONES
    set_freq(0, 1000);
    for(;;) {
        char key = get_keypress();
        if (key == 'A')
            set_freq(0,getfloat());
        if (key == 'B')
            set_freq(1,getfloat());
    }
#endif

    // demonstrate part 4
//#define TEST_SETRGB
#ifdef TEST_SETRGB
    for(;;) {
        char key = get_keypress();
        if (key == 'A')
            set_freq(0,getfloat());
        if (key == 'B')
            set_freq(1,getfloat());
        if (key == 'D')
            setrgb(getrgb());
    }
#endif

    // Have fun.
    dialer();
}
