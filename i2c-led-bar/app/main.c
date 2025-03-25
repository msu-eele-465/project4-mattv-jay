/**
 * @file main.c
 * @brief Main file to run all code.
 */

#include <msp430fr2310.h>
#include <stdint.h>
#include <stdbool.h>

#define I2C_ADDR 0x48

unsigned int pattern_num = 0;
uint8_t pattern = 0b00000000;

bool unlocked = false;
char key = '\0';

void init(void)
{
    // Set ports 1.0, 1.1, 1.4-1.7, 2.0, 2.6, 2.7 as outputs
    P1DIR |= BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7;
    P2DIR |= BIT0 | BIT6 | BIT7;

    // Set outputs to zero
    P1OUT &= ~(BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7);
    P2OUT &= ~(BIT0 | BIT6 | BIT7);

    // I2C pins
    P1SEL0 |= BIT2 | BIT3;
    P1SEL1 &= ~(BIT2 | BIT3);

    // Disable the GPIO power-on default high-impedance mdoe to activate
    // previously configure port settings
    PM5CTL0 &= ~LOCKLPM5;

    TB0CTL = TBSSEL__SMCLK | MC_1 | TBCLR | ID__8; // SMCLK, up mode, clear TBR
    TB0EX0 = TBIDEX__8;
    TB0CCR0 = 15624; // Set up 1.0s period
    TB0CCTL0 &= ~CCIFG; // Clear CCR0 Flag
    TB0CCTL0 |= CCIE; // Enable TB0 CCR0 Overflow IRQ

    UCB0CTLW0 = UCSWRST; // Software reset enabled
    UCB0CTLW0 |= UCMODE_3 | UCSYNC; // I2C mode, sync mode
    UCB0I2COA0 = I2C_ADDR | UCOAEN; // Own Address and enable
    UCB0CTLW0 &= ~UCSWRST; // clear reset register
    UCB0IE |= UCRXIE;
    __enable_interrupt(); // Enable Maskable IRQs
}

int main(void)
{
    int trans_period = 15625 - 1;
    const float PER_FRACTIONS[] = { 1.0, 1.0, 0.5, 0.5, 0.25, 1.5, 0.5, 1.0 };

    uint8_t pattern_store[] = { { 0b10101010 }, { 0b10101010 }, { 0b00000000 }, { 0b00011000 },
                                { 0b11111111 }, { 0b00000001 }, { 0b01111111 }, { 0b00000000 } };

    const uint8_t PATTERNS[] = { { 0b10101010 }, { 0b10101010 }, { 0b00000000 }, { 0b00011000 },
                                 { 0b11111111 }, { 0b00000001 }, { 0b01111111 }, { 0b00000000 } };

    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    // Initialize ports and other subsystems
    init();

    while (true)
    {
        if (unlocked)
        {
            if (key == 'A')
            {
                if (trans_period != 3906)
                {
                    trans_period -= 3906;
                    key = '\0';
                }
            }
            else if (key == 'B')
            {
                trans_period += 3906;
                key = '\0';
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
                // Set LED bar outputs
                P1OUT = (P1OUT & 0b11111100) | (pattern & 0b00000011);
                P1OUT = (P1OUT & 0b00001111) | ((pattern & 0b00111100) << 2);
                P2OUT = (P2OUT & 0b00111111) | (pattern & 0b11000000);
                key = '\0';
            }
            TB0CCR0 = (unsigned int)(trans_period * PER_FRACTIONS[pattern_num]);
        }
    }
}

#pragma vector = TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void)
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

    // Set LED bar outputs
    P1OUT = (P1OUT & 0b11111100) | (pattern & 0b00000011);
    P1OUT = (P1OUT & 0b00001111) | ((pattern & 0b00111100) << 2);
    P2OUT = (P2OUT & 0b00111111) | (pattern & 0b11000000);

    TB0CCTL0 &= ~CCIFG; // Clear CCR0 Flag
}

#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void)
{
    key = UCB0RXBUF;
    if (key == 'U')
    {
        unlocked = true;
    }
}
