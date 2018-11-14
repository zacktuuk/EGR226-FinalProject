#include "msp.h"
#include <stdio.h>

// IR Input Code.
// Input 10Hz on P2.4
// TA0.1

uint16_t current, last = 0, period, firsttime = 1,newreading = 0;

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    P2->SEL0 |=  BIT5;
    P2->SEL1 &= ~BIT5;
    P2->DIR  |=  BIT5;
      P1->SEL0 &=  ~BIT0;            //start light off
      P1->SEL1 &= ~BIT0;
      P1->DIR  |=  BIT0;
      P1->OUT &=~BIT0;

    // 3000000 > 65535, so we need dividers
    TIMER_A0->CTL = 0b0000001011010100;  //SMCLK, UP mode, divide by 8
    TIMER_A0->CCR[0] = 37500-1;  //10 Hz at 3MHz with Divider 8

    TIMER_A0->CCR[2] = 18750-1;  //50% Duty Cycle
//    TIMER_A0->CCR[1] = TIMER_A0->CCR[0] >> 1;  // This line works too
    TIMER_A0->CCTL[2] = 0b11100000;

    float localperiod;
    float hz;

    P5->SEL0 |=  BIT6;
    P5->SEL1 &= ~BIT6;
    P5->DIR  &= ~BIT6;
//    P2->IE   |=  BIT4;  /Don't do this since it isn't a pin interrupt

    // 3000000 > 65535, so we need dividers
    TIMER_A2->CTL = 0b0000001011010100;  //SMCLK, UP mode, divide by 8
    TIMER_A2->CCR[0] = 65535;  //Set to max for maximum capability (can capture up to 0.16 seconds)

//   TIMER_A0->CCR[1] = MaxsVariable;  //10Hz Value
//    TIMER_A0->CCR[1] = TIMER_A0->CCR[0] >> 1;  // This line works too
    TIMER_A2->CCTL[1] = 0b0100100100010000;

    NVIC_EnableIRQ(TA2_N_IRQn);
    __enable_interrupt();

    while(1) {
        if(!(newreading)){
        P1->OUT &=~BIT0;
        }
        if(newreading)
        {
            localperiod = period / (3000000.0/8);  //divider of 8
            printf("period (ms) = %f, frequency (Hz) = %f\n",localperiod*1000,(1.0/localperiod));
            hz = (1.0/localperiod);
            newreading = 0;
            if(((9 <= hz) && (hz <= 11))||((13 <= hz) && (hz <= 15))){
                P1->OUT |= BIT0;
                __delay_cycles(1000);

            }

        }
    }
}


//Not yet complete.  Will complete on Wednesday.
void TA2_N_IRQHandler()
{
    if(TIMER_A2->CCTL[1] & BIT0) {
        TIMER_A2->CCTL[1] &= ~BIT0;  //Clears the interrupt flag
        current = TIMER_A2->CCR[1];
        if(firsttime==0)
        {
            period = current - last;
            newreading = 1;
    }
        else
            firsttime = 0;
        last = current;
    }
}
