/**
 * @file rgb.c
 * @brief Test file to run all rgb related code.
 */

#include <msp430fr2355.h>
#include <stdint.h>
#include <stdbool.h>

#define PERIOD 1000 - 1

unsigned int to_ccr(unsigned int color_val);

int state = 2;

int main(void)
{
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer

    uint8_t rgb_value[] = {196, 62, 29};

    // Configure GPIO
    P6DIR |= BIT0 | BIT1 | BIT2;            // P6.0 P6.1 P6.2 output
    P6SEL0 |= BIT0 | BIT1 | BIT2;           // P6.0 P6.1 P6.2 options select (TB3.1 -> P6.0, etc...)

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configure port settings
    PM5CTL0 &= ~LOCKLPM5;

    // Setup Timer3_B
    TB3CCR0 = PERIOD;                       // PWM Period
    TB3CCTL1 = OUTMOD_7;                    // CCR1 reset/set
    TB3CCR1 = 0;                           // CCR1 PWM duty cycle
    TB3CCTL2 = OUTMOD_7;                    // CCR2 reset/set
    TB3CCR2 = 0;                           // CCR2 PWM duty cycle
    TB3CCTL3 = OUTMOD_7;                    // CCR3 reset/set
    TB3CCR3 = 0;                           // CCR3 PWM duty cycle
    TB3CTL = TBSSEL__SMCLK | MC_1 | TBCLR;  // SMCLK, up mode, clear TBR

    while (true)
    {
        switch (state)
        {
            case 0:
                // red
                rgb_value[0] = 196;
                rgb_value[1] = 16;
                rgb_value[2] = 5;
                break;
            case 1:
                // orange
                rgb_value[0] = 196;
                rgb_value[1] = 96;
                rgb_value[2] = 5;
                break;
            case 2:
                // blue
                rgb_value[0] = 29;
                rgb_value[1] = 162;
                rgb_value[2] = 196;
                break;
            default:
                // red
                rgb_value[0] = 255;
                rgb_value[1] = 255;
                rgb_value[2] = 255;
                break;
        }
        SYSCFG0 = FRWPPW;                    // FRAM write enable
        TB3CCR1 = to_ccr(rgb_value[0]);
        TB3CCR2 = to_ccr(rgb_value[1]);
        TB3CCR3 = to_ccr(rgb_value[2]);
        SYSCFG0 = FRWPPW | PFWP;             // FRAM write disable
    }
}


unsigned int to_ccr(unsigned int color_val)
{
    return color_val * (float)(PERIOD)/255.0;  // TB3CCR0/255 = 999/255 = 3.91765;
}
