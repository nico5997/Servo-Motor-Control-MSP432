#include "msp.h"

#define Period_0 1638 // f = 32768 Hz -> 50ms: 32768*0.05 = 1638
#define Period_2 656 // f = 32768 Hz -> 20ms: 32768*0.05 = 656


#define Ratio_duty_cycle_0 0.5 
#define Ratio_duty_cycle_2_0 (1-0.075)
#define Ratio_duty_cycle_2_90 (1-0.1)
#define Ratio_duty_cycle_2_m90 (1-0.05)

#define Pulse_width_0 Ratio_duty_cycle_0*Period_0
#define Pulse_width_2_0 Ratio_duty_cycle_2_0*Period_2
#define Pulse_width_2_90 Ratio_duty_cycle_2_90*Period_2
#define Pulse_width_2_m90 Ratio_duty_cycle_2_m90*Period_2

volatile uint32_t ADCvalue;

/**
 * main.c
 */

void TimerA0(uint16_t period, uint16_t pulse_width) {

    TIMER_A0->CTL |= TIMER_A_CTL_TASSEL_1 | // Choice of ACLK for the clock
                     TIMER_A_CTL_MC__UP | // Picking the UP mode
                     TIMER_A_CTL_ID_0 | // There is no pre-division of the clock
                     TIMER_A_CTL_IE; // Enabling the TimerA0 interrupt

    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CAP; // Disabling capture mode
    TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE; // Capture/compare interrupt enabling
    TIMER_A0->CCTL[1] |= TIMER_A_CCTLN_OUTMOD_3; // Output mode Set/reset choice

    TIMER_A0->CCR[0] |= period; // Value of compare register set to 1638 to have 50ms interrupts
    TIMER_A0->CCR[1] |= pulse_width; 

    NVIC_EnableIRQ(ADC14_IRQn); // Enable of the ADC14 Interrupt
    NVIC_EnableIRQ(TA2_0_IRQn); // Enable of the Timer A2 Interrupt
    NVIC_EnableIRQ(TA0_0_IRQn); // Enable of the Timer A0 Interrupt

}

void TA0_0_IRQHandler(void) {

    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; // Resets the flag to zero

    while (ADC14->CTL0 & ADC14_CTL0_BUSY) {}; // Waiting for the Busy to be at zero

    ADC14->CTL0 |= ADC14_CTL0_SC; // Starting a conversion

}

void TimerA2(uint16_t period_2, uint16_t pulse_width_2_0) {

    TIMER_A2->CTL |= TIMER_A_CTL_TASSEL_1 | // Choice of ACLK for the clock
                         TIMER_A_CTL_MC__UP | // Picking the UP mode
                         TIMER_A_CTL_ID_0| // There is no pre-division of the clock
                         TIMER_A_CTL_IE; // Enabling the TimerA2 interrupt

    TIMER_A2->CCTL[0] &= ~TIMER_A_CCTLN_CAP; // Disabling capture mode
    TIMER_A2->CCTL[0] |= TIMER_A_CCTLN_CCIE; // Capture/compare interrupt enabling
    TIMER_A2->CCTL[1] |= TIMER_A_CCTLN_OUTMOD_3; // Output mode Set/reset choice

    TIMER_A2->CCR[0] = period_2; // Value of compare register set to 656 to have 20ms interrupts
    TIMER_A2->CCR[1] = pulse_width_2_0; // Half of the value of compare register as a start

}

void TA2_0_IRQHandler(void) {

    TIMER_A2->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; // Resets the flag to zero

}

void ADC14_Fct() {

    P4->SEL0 |= (1<<0); // Analog output set to pin P4.0
    P4->SEL1 |= (1<<0);

    ADC14->CTL0 &= ~ADC14_CTL0_ENC; // Enabling modifications to ADC settings

    while (ADC14->CTL0 & ADC14_CTL0_BUSY) {}; // Waiting for the Busy to be at zero: indicates an active sample or conversion operatio

    ADC14->CTL0 |= ADC14_CTL0_PDIV_0 | // No pre-division
                      ADC14_CTL0_SHS_1 | // TA0.1 signal 
                      ADC14_CTL0_SHP | // Signal is sourced from the sampling timer.
                      ADC14_CTL0_DIV_0 | // No pre-division for the clock
                      ADC14_CTL0_SSEL__ACLK | // Choice of ACLK for the clock
                      ADC14_CTL0_CONSEQ_2 | // Single-channel, single-conversion
                      ADC14_CTL0_ON; // Power on

    ADC14->CTL1 |= ADC14_CTL1_RES__14BIT; 

    ADC14->MCTL[0] &= ~ADC14_MCTLN_INCH_MASK; 
    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_13; // Channel 13. P4.0

    ADC14->IER0 |= ADC14_IER0_IE0; // Enabling ADC14IFG0 for interrupt

    ADC14->CTL0 |= ADC14_CTL0_ENC; // Enable, stop modifications

}

void ADC14_IRQHandler(void) {

    ADCvalue = ADC14->MEM[0] & 0xffff; // ADCvalue is put into MEM0

    if(ADCvalue < 5000){
                P2->OUT |=  (1<<0);
                P2->OUT &= ~(1<<1); // Red LED on
                TIMER_A2->CCR[1] = Pulse_width_2_90; // Angle for servo = 90

            }
            else if(ADCvalue > 12000) {
                P2->OUT |=  (1<<1);
                P2->OUT &= ~(1<<0);// Green LED on
                TIMER_A2->CCR[1] = Pulse_width_2_m90; // Angle for servo = -90
            }
            else {
                P2->OUT |= (1<<1);
                P2->OUT |= (1<<0); // Yellow LED on
                TIMER_A2->CCR[1] = Pulse_width_2_0; // Angle for servo = 0
            }

}

void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	P5->DIR = 0x40; // Taking pin P5.6 as an output
	P5->SEL0 |= 0x40; // Associating Timer2A function to the pin P5.6
	P5->SEL1 &= ~0x40;

	P2->SEL0 &= ~(1<<0); // Settings for Red LED
	P2->SEL1 &= ~(1<<0);
	P2->DIR |=(1<<0);
	P2->OUT &= ~(1<<0);

	P2->SEL0 &= ~(1<<1); // Settings for Green LED
	P2->SEL1 &= ~(1<<1);
	P2->DIR |=(1<<1);
	P2->OUT &= ~(1<<1);

	ADC14_Fct();
	TimerA2(Period_2, Pulse_width_2_0);
    TimerA0(Period_0, Pulse_width_0);
    __enable_irq();

	while(1){

	}

}
