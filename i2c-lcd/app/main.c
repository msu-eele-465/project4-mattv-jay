/**
 * @file main.c
 * @brief Main file to run all code.
 */

#include <msp430fr2310.h>
#include <stdint.h>
#include <stdbool.h>

#define I2C_ADDR 0x49

unsigned int pattern_num = 0; // Tracks which pattern is active

bool unlocked = false;
char key = '\0';

unsigned int time_since_active = 3;

/**
 * Initializes all GPIO ports.
 */
void initGPIO(void)
{
    // Set ports 1.4-1.7, 2.0, 2.6, 2.7 as outputs
    P1DIR |= BIT4 | BIT5 | BIT6 | BIT7;
    P2DIR |= BIT0 | BIT6 | BIT7;

    // Set GPIO outputs to zero
    P1OUT &= ~(BIT4 | BIT5 | BIT6 | BIT7);
    P2OUT &= ~(BIT0 | BIT6 | BIT7);

    // I2C pins
    P1SEL0 |= BIT2 | BIT3;
    P1SEL1 &= ~(BIT2 | BIT3);

    // Disable the GPIO power-on default high-impedance mdoe to activate
    // previously configure port settings
    PM5CTL0 &= ~LOCKLPM5;
}

/**
 * Initializes all timers.
 */
void initTimer(void)
{
    TB0CTL = TBSSEL__ACLK | MC_1 | TBCLR | ID__2; // ACLK, up mode, clear TBR, divide by 2
    TB0CCR0 = 16384; // Set up 1.0s period
    TB0CCTL0 &= ~CCIFG; // Clear CCR0 Flag
    TB0CCTL0 |= CCIE; // Enable TB0 CCR0 Overflow IRQ

    TB1CTL = TBSSEL__ACLK | MC_2 | TBCLR | ID__8 | CNTL_1; // ACLK, continuous mode, clear TBR, divide by 8, length
                                                           // 12-bit
    TB1CTL &= ~TBIFG; // Clear CCR0 Flag
    TB1CTL |= TBIE; // Enable TB1 Overflow IRQ
}

/**
 * Sets all I2C parameters.
 */
void initI2C(void)
{
    UCB0CTLW0 = UCSWRST; // Software reset enabled
    UCB0CTLW0 |= UCMODE_3 | UCSYNC; // I2C mode, sync mode
    UCB0I2COA0 = I2C_ADDR | UCOAEN; // Own Address and enable
    UCB0CTLW0 &= ~UCSWRST; // clear reset register
    UCB0IE |= UCRXIE; // Enable I2C read interrupt
}

/**
 * Main function.
 *
 * A longer description, with more discussion of the function
 * that might be useful to those using or modifying it.
 */
int main(void)
{
    const char *PATTERNS[] = { "static",       "toggle",        "up counter",     "in and out",
                                "down counter", "rotate 1 left", "rotate 7 right", "fill left" };

    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    // Initialize ports and other subsystems
    initGPIO();
    initTimer();
    initI2C();

    __enable_interrupt(); // Enable Maskable IRQs

    while (true)
    {
        if (unlocked)
        {
            if ((key - '0') >= 0 && (key - '0') < 8)
            {
                pattern_num = key - '0';
                // output PATTERNS[pattern_num] to LCD;
                key = '\0';
            }
        }
    }
}

/**
 * Timer B0 Compare Interrupt.
 *
 * Runs periodically according to the transistion
 * period ("trans_period") and the corresponding
 * patterns fractional period. Updates LED bar
 * display pattern based on currently selected
 * pattern.
 */
#pragma vector = TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void)
{
    TB0CCTL0 &= ~CCIFG; // Clear CCR0 Flag
}

/**
 * Timer B1 Overflow Interrupt.
 *
 * Runs every second. Starts flashing status LED
 * 3 seconds after receiving something over I2C.
 */
#pragma vector = TIMER1_B1_VECTOR
__interrupt void ISR_TB1_OVERFLOW(void)
{
    if (time_since_active >= 3)
    {
        P2OUT ^= BIT0;
    }
    time_since_active++;

    TB1CTL &= ~TBIFG; // Clear CCR0 Flag
}

/**
 * I2C RX Interrupt.
 *
 * Stores value received over I2C in global var "key".
 * If 'U' is received over I2C, set the "unlocked" var.
 */
#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void)
{
    key = UCB0RXBUF;
    if (key == 'U')
    {
        unlocked = true;
    }
    P2OUT |= BIT0;
    time_since_active = 0;
}
