/**
 * @file keypad.c
 * @brief Test file to test keypad input code.
 */

#include <msp430fr2355.h>
#include <stdint.h>
#include <string.h>

#define UNLOCK_CODE "1234" // 4-digit unlock code
#define CODE_LENGTH 4
#define PERIOD 1000 - 1

char entered_code[CODE_LENGTH + 1]; // Stores user inp
unsigned int code_index = 0;
int unlocked = 0; // 0 = locked, 1 = unlocking, 2 = unlocked

void setup()
{
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    // Keypad setup
    P3DIR |= BIT0 | BIT1 | BIT2 | BIT3; // Rows as outputs
    P3OUT |= BIT0 | BIT1 | BIT2 | BIT3; // Set all rows high (inactive)

    P3DIR &= ~(BIT4 | BIT5 | BIT6 | BIT7); // Columns as inputs
    P3REN |= BIT4 | BIT5 | BIT6 | BIT7; // Enable pull-up resistors
    P3OUT |= BIT4 | BIT5 | BIT6 | BIT7;

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
}

unsigned int to_ccr(unsigned int color_val);
char scan_keypad(void);
void keypad_input(char key);

int main()
{
    uint8_t rgb_value[] = {196, 62, 29};

    setup();

    while (1)
    {
        char key = scan_keypad(); // Retrieve pressed key
        if (key)
        {
            keypad_input(key);
        }
        switch (unlocked)
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

char scan_keypad(void)
{
    const char keys[4][4] = {
        { '1', '2', '3', 'A' }, { '4', '5', '6', 'B' }, { '7', '8', '9', 'C' }, { '*', '0', '#', 'D' }
    };

    int row;
    for (row = 0; row < 4; row++)
    {
        P3OUT |= BIT0 | BIT1 | BIT2 | BIT3; // Set all rows high
        P3OUT &= ~(1 << row); // Pull current row low

        __delay_cycles(1000); // delay

        int col;
        for (col = 0; col < 4; col++)
        {
            if (!(P3IN & (BIT4 << col)))
            { // If key is pressed
                __delay_cycles(10000); // Debounce delay
                while (!(P3IN & (BIT4 << col)))
                    ; // Wait for key release
                return keys[row][col];
            }
        }
    }
    return 0;
}

void keypad_input(char key)
{
    if (key == 'D')
    { // Lock system again
        unlocked = 0;
        code_index = 0;
    }

    if (unlocked == 2)
    {
        // UCTXBUF = key;
        return; // Send char over I2C if unlocked?
    }

    if (key >= '0' && key <= '9')
    {
        entered_code[code_index++] = key;
        unlocked = 1;
        if (code_index == CODE_LENGTH)
        {
            entered_code[code_index] = '\0'; // Null terminate
            if (strcmp(entered_code, UNLOCK_CODE) == 0)
            {
                unlocked = 2;
            }
            else
            {
                unlocked = 0;
                code_index = 0; // Reset on incorrect input
            }
        }
    }
}
