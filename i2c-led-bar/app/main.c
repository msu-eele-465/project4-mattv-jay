/**
 * @file main.c
 * @brief Main file to run all code.
 */

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

unsigned int pattern_num = 0;
uint8_t pattern = 0b00000000;

bool unlocked = false;

int main(void)
{
    char key;
    int trans_period = 15625 - 1;
    const float PER_FRACTIONS[] = {1.0, 1.0, 0.5, 0.5, 0.25, 1.5, 0.5, 1.0};

    uint8_t pattern_store[] = {
        {0b10101010},
        {0b10101010},
        {0b00000000},
        {0b00011000},
        {0b11111111},
        {0b00000001},
        {0b01111111},
        {0b00000000}
    };

    const uint8_t PATTERNS[] = {
        {0b10101010},
        {0b10101010},
        {0b00000000},
        {0b00011000},
        {0b11111111},
        {0b00000001},
        {0b01111111},
        {0b00000000}
    };

    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer

    //set ports 3.0-3.7 as outputs
    //set P3OUT = 0b00000000

    P3DIR |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;
    P3OUT = 0b00000000;

    // Configure switches
    P4DIR &= ~BIT1;         // Clear P4.1 (S1) direction = in
    P4REN |= BIT1;          // Enable pull up/down resistor
    P4OUT |= BIT1;          // Make resistor a pull up
    P4IES &= ~BIT1;         // Config IRQ Sensitivity L-to-H

    P2DIR &= ~BIT3;         // Clear P2.3 (S2) direction = in
    P2REN |= BIT3;          // Enable pull up/down resistor
    P2OUT |= BIT3;          // Make resistor a pull up
    P2IES &= ~BIT3;         // Config IRQ Sensitivity L-to-H

    // Disable the GPIO power-on default high-impedance mdoe to activate
    // previously configure port settings
    PM5CTL0 &= ~LOCKLPM5;

    TB1CTL = TBSSEL__SMCLK | MC_1 | TBCLR | ID__8;  // SMCLK, up mode, clear TBR
    TB1EX0 = TBIDEX__8;

    // Set up 1.0s period
    TB1CCR0 = trans_period;

    TB1CCTL0 &= ~CCIFG;     // Clear CCR0 Flag
	TB1CCTL0 |= CCIE;       // Enable TB1 CCR0 Overflow IRQ
	__enable_interrupt();   // Enable Maskable IRQs

    while(true)
    {
        
        if (unlocked)
        {
            if (key == 'A')
            {
                if (trans_period != 3906)
                {
                    trans_period -= 3906;
                }
            }
            else if (key == 'B') 
            {
                trans_period += 3906;
            }
            else if ((key - '0') >= 0 && (key - '0') < 8) 
            {
                if (pattern_num == key - '0')
                {
                    pattern = PATTERNS[(unsigned int)(key - '0')];
                }
                else 
                {
                    pattern_store[pattern_num] = pattern;
                    pattern = pattern_store[(unsigned int)(key - '0')];
                    pattern_num = (unsigned int)(key - '0');
                }
                P3OUT = pattern;
            }
        }

        if (pattern_num < 8)
        {
            TB1CCR0 = (unsigned int)(trans_period * PER_FRACTIONS[pattern_num]);
        }
    }
}

#pragma vector = TIMER1_B0_VECTOR
__interrupt void ISR_TB1_CCR0(void) 
{
switch (pattern_num)
    {
        case 1:
            pattern = ~pattern;
            break;
        case 2:
            pattern++;
            break;
        case 3:
            // pulse LED bar in and out;
            break;
        case 4:
            pattern--;
            break;
        case 5:
            if (pattern == 0b10000000)
            {
                pattern ^= 0b10000001;
            }
            else
            {
                pattern = pattern << 1;
            }
            break;
        case 6:
            if (pattern == 0b11111110)
            {
                pattern = 0b01111111;
            }
            else 
            {
                pattern = pattern >> 1;
                pattern ^= 0b10000000;
            }
            break;
        case 7:
            if (pattern == 0b11111111)
            {
                pattern = ~pattern;
            }
            pattern = pattern << 1;
            pattern ^= 0b00000001;
            break;
    }
    P3OUT = pattern;

    TB0CCTL0 &= ~CCIFG;     // Clear CCR0 Flag
}