#include <msp430.h>
#include <stdint.h>

// I2C control pins
#define SDA_PIN BIT1
#define SCL_PIN BIT2

// I2C Addresses
#define LED_PERIPHERAL_ADDR 0x20
#define LCD_PERIPHERAL_ADDR 0x21

// LCD Commands
#define LCD_CLEAR 0x01
#define LCD_CURSOR_ON 0x0E
#define LCD_CURSOR_OFF 0x0C
#define LCD_BLINK_ON 0x0D
#define LCD_BLINK_OFF 0x0C

// Unlock sys vars
volatile uint8_t unlocked = 0;
volatile uint8_t cursor_on = 1;
volatile uint8_t blink_on = 0;
const uint8_t unlock_code[4] = {1, 2, 3, 4};
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
void handle_keypress(uint8_t key);
void update_lcd_display(uint8_t key);
void send_to_led(uint8_t pattern);

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog

    i2c_init();       // Initialize I2C with int
    lcd_init();       // Initialize LCD
    keypad_init();    // Initialize Keypad

    lcd_clear();                   // Start cleared LCD
    lcd_command(LCD_CURSOR_OFF);   // Cursor is off

    while (1) {
        uint8_t key = keypad_get_key();
        if (key != 0xFF) {
            handle_keypress(key);  // Handle any detected keypress
        }
        __delay_cycles(100000);  // delay
    }
}

void i2c_init(void) {
    P1SEL0 |= SDA_PIN | SCL_PIN;         // Set SDA and SCL pins
    UCB0CTLW0 = UCSWRST;                 // Put eUSCI_B0 in reset mode
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSYNC;  // I2C master mode
    UCB0BRW = 10;                        // Set I2C clock prescaler
    UCB0CTLW0 &= ~UCSWRST;               // Release eUSCI_B0 from reset

    UCB0IE |= UCTXIE0 | UCRXIE0;         // Enable TX and RX interrupts
    __enable_interrupt();                // Enable global interrupts
}

void i2c_write_byte_interrupt(uint8_t addr, uint8_t byte) {
    i2c_tx_data[0] = addr;
    i2c_tx_data[1] = byte;
    i2c_tx_index = 0;

    UCB0I2CSA = addr;        // Set I2C slave address
    UCB0CTLW0 |= UCTR | UCTXSTT;  // Set to transmit mode and send start condition
}

#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_ISR(void) {
    switch (__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG)) {
        case USCI_NONE: break;
        case USCI_I2C_UCTXIFG0:
            if (i2c_tx_index < 2) {
                UCB0TXBUF = i2c_tx_data[i2c_tx_index++];  // Load TX buffer
            } else {
                UCB0CTLW0 |= UCTXSTP;  // Send stop condition
                UCB0IFG &= ~UCTXIFG0;  // Clear TX interrupt flag
            }
            break;

        case USCI_I2C_UCRXIFG0:
            i2c_rx_data = UCB0RXBUF;  // Read received byte
            break;

        default: break;
    }
}

void lcd_init(void) {
    lcd_command(0x38);  // Function set: 2-line disp, 5x8 font
    lcd_command(0x0C);  // Display on, cursor off
    lcd_command(0x01);  // Clear display
}

void lcd_command(uint8_t cmd) {
    i2c_write_byte_interrupt(LCD_PERIPHERAL_ADDR, cmd);  // Use interrupt-based I2C
}

void lcd_write_char(uint8_t data) {
    i2c_write_byte_interrupt(LCD_PERIPHERAL_ADDR, data);  // Use interrupt-based I2C
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t pos = (row == 0) ? col : 0x40 + col;  // Calculate DDRAM address
    lcd_command(0x80 | pos);  // Set DDRAM address with command 0x80
}

void lcd_clear(void) {
    lcd_command(LCD_CLEAR);  // Clear the LCD screen
}

void keypad_init(void) {
    P2DIR &= ~(BIT0 | BIT1 | BIT2 | BIT3);  // Configure rows as input
    P2REN |= (BIT0 | BIT1 | BIT2 | BIT3);   // Enable pull-up/down resistors
    P2OUT |= (BIT0 | BIT1 | BIT2 | BIT3);   // Pull-up resistors enabled
    P3DIR |= (BIT0 | BIT1 | BIT2 | BIT3);   // Configure columns as output
}

uint8_t keypad_get_key(void) {
    uint8_t row, col;
    for (col = 0; col < 4; col++) {
        P3OUT = ~(1 << col);            // Set current col to low
        __delay_cycles(1000);           // delay
        row = P2IN & (BIT0 | BIT1 | BIT2 | BIT3);  // Read row inp
        if (row != (BIT0 | BIT1 | BIT2 | BIT3)) {
            break;  // Break if a row inp is detected
        }
    }
    if (col == 4) return 0xFF;  // No key pressed

    uint8_t keys[4][4] = {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
    };
    return keys[3 - __builtin_ctz(row)][col];  // Return detected key
}
void handle_keypress(uint8_t key) {
    if (!unlocked) {
        if (key == unlock_code[code_index]) {
            code_index++;
            if (code_index >= 4) {
                unlocked = 1;
                lcd_command(LCD_CURSOR_ON);
                lcd_clear();
                code_index = 0;
            }
        } else {
            code_index = 0;
        }
    } else {
        update_lcd_display(key);
    }
}
void update_lcd_display(uint8_t key) {
    lcd_set_cursor(0, 0);
    lcd_write_char(key);
}
void send_to_led(uint8_t pattern) {
    i2c_write_byte_interrupt(LED_PERIPHERAL_ADDR, pattern);
}
