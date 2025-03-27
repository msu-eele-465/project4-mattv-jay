/**
 * @file keypad.c
 * @brief Test file to test keypad input code.
 */

#include <msp430fr2355.h>
#include <string.h>

#define UNLOCK_CODE "1234" // 4-digit unlock code
#define CODE_LENGTH 4

char entered_code[CODE_LENGTH + 1]; // Stores user inp
unsigned int code_index = 0;
int unlocked = 0; // 0 = locked, 1 = unlocked

void setup()
{
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    // Keypad setup
    P3DIR |= BIT0 | BIT1 | BIT2 | BIT3; // Rows as outputs
    P3OUT |= BIT0 | BIT1 | BIT2 | BIT3; // Set all rows high (inactive)

    P3DIR &= ~(BIT4 | BIT5 | BIT6 | BIT7); // Columns as inputs
    P3REN |= BIT4 | BIT5 | BIT6 | BIT7; // Enable pull-up resistors
    P3OUT |= BIT4 | BIT5 | BIT6 | BIT7;

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configure port settings
    PM5CTL0 &= ~LOCKLPM5;
}

void update_rgb()
{
    // if (unlocked) {
    //     P2OUT = BIT2;  // Set RGB LED to blue (unlocked)
    // } else {
    //     P2OUT = BIT0;  // Set RGB LED to red (locked)
    // }
}

char scan_keypad(void);
void keypad_input(char key);

int main()
{
    setup();
    update_rgb();

    while (1)
    {
        char key = scan_keypad(); // Retrieve pressed key
        if (key)
        {
            keypad_input(key);
        }
    }
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
        update_rgb();
    }

    if (unlocked)
    {
        // UCTXBUF = key;
        return; // Send char over I2C if unlocked?
    }

    if (key >= '0' && key <= '9')
    {
        entered_code[code_index++] = key;
        if (code_index == CODE_LENGTH)
        {
            entered_code[code_index] = '\0'; // Null terminate
            if (strcmp(entered_code, UNLOCK_CODE) == 0)
            {
                unlocked = 1;
                update_rgb();
            }
            else
            {
                code_index = 0; // Reset on incorrect input
            }
        }
    }
}
