#include <msp430fr2355.h>
#include <stdbool.h>

int main(void)
{
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    P1OUT &= ~BIT0;
    P1DIR |= BIT0;

    // Disable low-power mode / GPIO high-impedance
    PM5CTL0 &= ~LOCKLPM5;

    while (true)
    {
        P1OUT ^= BIT0;

        // Delay for 100000*(1/MCLK)=0.1s
        __delay_cycles(100000);
    }
}
