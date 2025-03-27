#include <msp430.h>
#include <stdint.h>

// I2C control pins
#define SDA_PIN BIT2
#define SCL_PIN BIT3

// I2C Addresses
#define LED_PERIPHERAL_ADDR 0x20
#define LCD_PERIPHERAL_ADDR 0x21

// LCD Commands
#define LCD_CLEAR 0x01
#define LCD_CURSOR_ON 0x0E
#define LCD_CURSOR_OFF 0x0C
#define LCD_BLINK_ON 0x0D
#define LCD_BLINK_OFF 0x0C

//RGB constants
#define PERIOD 1000 - 1

// Unlock sys vars
volatile uint8_t unlocked = 0;
volatile uint8_t cursor_on = 1;
volatile uint8_t blink_on = 0;
const char unlock_code[4] = { '1', '2', '3', '4' };
volatile uint8_t code_index = 0;

// I2C TX and RX Buffers
volatile uint8_t i2c_tx_data[2];
volatile uint8_t i2c_tx_index = 0;
volatile uint8_t i2c_rx_data = 0;

void i2c_init(void);
void i2c_write_byte_interrupt(uint8_t addr, uint8_t byte);
void lcd_init(void);
void lcd_command(uint8_t cmd);
void lcd_write_char(uint8_t data);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_clear(void);
void keypad_init(void);
uint8_t keypad_get_key(void);
void rgb_init(void);
unsigned int to_ccr(unsigned int color_val);
void handle_keypress(uint8_t key);
void update_lcd_display(uint8_t key);
void send_to_led(uint8_t pattern);

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog

    i2c_init(); // Initialize I2C with int
    lcd_init(); // Initialize LCD
    keypad_init(); // Initialize Keypad
    rgb_init();

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configure port settings
    PM5CTL0 &= ~LOCKLPM5;

    TB3CCR0 = PERIOD;                       // PWM Period
    TB3CCTL1 = OUTMOD_7;                    // CCR1 reset/set
    TB3CCR1 = 0;                           // CCR1 PWM duty cycle
    TB3CCTL2 = OUTMOD_7;                    // CCR2 reset/set
    TB3CCR2 = 0;                           // CCR2 PWM duty cycle
    TB3CCTL3 = OUTMOD_7;                    // CCR3 reset/set
    TB3CCR3 = 0;                           // CCR3 PWM duty cycle
    TB3CTL = TBSSEL__SMCLK | MC_1 | TBCLR;  // SMCLK, up mode, clear TBR

    lcd_clear(); // Start cleared LCD
    lcd_command(LCD_CURSOR_OFF); // Cursor is off

    uint8_t rgb_value[] = {196, 62, 29};

    while (1)
    {
        uint8_t key = keypad_get_key();
        if (key != 0xFF)
        {
            handle_keypress(key); // Handle any detected keypress
        }
        __delay_cycles(100000); // delay
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

void rgb_init(void) 
{
    // Configure GPIO
    P6DIR |= BIT0 | BIT1 | BIT2;            // P6.0 P6.1 P6.2 output
    P6SEL0 |= BIT0 | BIT1 | BIT2;           // P6.0 P6.1 P6.2 options select (TB3.1 -> P6.0, etc...)
}

unsigned int to_ccr(unsigned int color_val)
{
    return color_val * (float)(PERIOD)/255.0;  // TB3CCR0/255 = 999/255 = 3.91765;
}

void i2c_init(void)
{
    P1SEL0 |= SDA_PIN | SCL_PIN; // Set SDA and SCL pins
    UCB0CTLW0 = UCSWRST; // Put eUSCI_B0 in reset mode
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSYNC; // I2C master mode
    UCB0BRW = 10; // Set I2C clock prescaler
    UCB0CTLW0 &= ~UCSWRST; // Release eUSCI_B0 from reset

    UCB0IE |= UCTXIE0 | UCRXIE0; // Enable TX and RX interrupts
    __enable_interrupt(); // Enable global interrupts
}

void i2c_write_byte_interrupt(uint8_t addr, uint8_t byte)
{
    i2c_tx_data[0] = addr;
    i2c_tx_data[1] = byte;
    i2c_tx_index = 0;

    UCB0I2CSA = addr; // Set I2C slave address
    UCB0CTLW0 |= UCTR | UCTXSTT; // Set to transmit mode and send start condition
}

#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_ISR(void)
{
    switch (__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
    {
        case USCI_NONE:
            break;
        case USCI_I2C_UCTXIFG0:
            if (i2c_tx_index < 2)
            {
                UCB0TXBUF = i2c_tx_data[i2c_tx_index++]; // Load TX buffer
            }
            else
            {
                UCB0CTLW0 |= UCTXSTP; // Send stop condition
                UCB0IFG &= ~UCTXIFG0; // Clear TX interrupt flag
            }
            break;

        case USCI_I2C_UCRXIFG0:
            i2c_rx_data = UCB0RXBUF; // Read received byte
            break;

        default:
            break;
    }
}

void lcd_init(void)
{
    lcd_command(0x38); // Function set: 2-line disp, 5x8 font
    lcd_command(0x0C); // Display on, cursor off
    lcd_command(0x01); // Clear display
}

void lcd_command(uint8_t cmd)
{
    i2c_write_byte_interrupt(LCD_PERIPHERAL_ADDR, cmd); // Use interrupt-based I2C
}

void lcd_write_char(uint8_t data)
{
    i2c_write_byte_interrupt(LCD_PERIPHERAL_ADDR, data); // Use interrupt-based I2C
}

void lcd_set_cursor(uint8_t row, uint8_t col)
{
    uint8_t pos = (row == 0) ? col : 0x40 + col; // Calculate DDRAM address
    lcd_command(0x80 | pos); // Set DDRAM address with command 0x80
}

void lcd_clear(void)
{
    lcd_command(LCD_CLEAR); // Clear the LCD screen
}

void keypad_init(void)
{
    P3DIR |= BIT0 | BIT1 | BIT2 | BIT3; // Rows as outputs
    P3OUT |= BIT0 | BIT1 | BIT2 | BIT3; // Set all rows high (inactive)

    P3DIR &= ~(BIT4 | BIT5 | BIT6 | BIT7); // Columns as inputs
    P3REN |= BIT4 | BIT5 | BIT6 | BIT7; // Enable pull-up resistors
    P3OUT |= BIT4 | BIT5 | BIT6 | BIT7;
}

uint8_t keypad_get_key(void)
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
void handle_keypress(uint8_t key)
{
    if (unlocked != 2)
    {
        if (key == unlock_code[code_index])
        {
            code_index++;
            unlocked = 1;
            if (code_index >= 4)
            {
                unlocked = 2;
                lcd_command(LCD_CURSOR_ON);
                lcd_clear();
                code_index = 0;
            }
        }
        else if (key != '\0')
        {
            code_index = 0;
            unlocked = 0;
        }
    }
    else
    {
        update_lcd_display(key);
    }
}
void update_lcd_display(uint8_t key)
{
    lcd_set_cursor(0, 0);
    lcd_write_char(key);
}
void send_to_led(uint8_t pattern)
{
    i2c_write_byte_interrupt(LED_PERIPHERAL_ADDR, pattern);
}
