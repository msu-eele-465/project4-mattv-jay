#include <msp430.h>
#include <stdint.h>

//I2C control pins (MSP)
#define SDA_PIN BIT1
#define SCL_PIN BIT2

//I2C Addresses
#define LED_PERIPHERAL_ADDR 0x20
#define LCD_PERIPHERAL_ADDR 0x21

//LCD Commands
#define LCD_CLEAR 0x01
#define LCD_CURSOR_ON 0x0E
#define LCD_CURSOR_OFF 0x0C
#define LCD_BLINK_ON 0x0D
#define LCD_BLINK_OFF 0x0C

volatile uint8_t unlocked = 0;
volatile uint8_t cursor_on = 1;
volatile uint8_t blink_on = 0;
const uint8_t unlock_code[4] = {1, 2, 3, 4};
volatile uint8_t code_index = 0;

void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write_byte(uint8_t byte);
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
    WDTCTL = WDTPW | WDTHOLD;  // Stop timer

    i2c_init();
    lcd_init();
    keypad_init();

    lcd_clear();
    lcd_command(LCD_CURSOR_OFF);  // LCD starts empty

    while (1) {
        uint8_t key = keypad_get_key();
        if (key != 0xFF) {
            handle_keypress(key);
        }
        __delay_cycles(100000);  // delay
    }
}
void i2c_init(void) {
    P1SEL0 |= SDA_PIN | SCL_PIN;
    UCB0CTLW0 = UCSWRST;
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSYNC;  // I2C master
    UCB0BRW = 10;
    UCB0CTLW0 &= ~UCSWRST;
}
void i2c_start(void) {
    UCB0CTLW0 |= UCTXSTT;
    while (UCB0CTLW0 & UCTXSTT);
}
void i2c_stop(void) {
    UCB0CTLW0 |= UCTXSTP;
    while (UCB0CTLW0 & UCTXSTP);
}
void i2c_write_byte(uint8_t byte) {
    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = byte;
}
void lcd_init(void) {
    lcd_command(0x38);  // 2-line
    lcd_command(0x0C);  // Display on, cursor off
    lcd_command(0x01);  // Clear disp
}
void lcd_command(uint8_t cmd) {
    i2c_start();
    i2c_write_byte(LCD_PERIPHERAL_ADDR);
    i2c_write_byte(0x00);  // Control byte for com
    i2c_write_byte(cmd);
    i2c_stop();
}
void lcd_write_char(uint8_t data) {
    i2c_start();
    i2c_write_byte(LCD_PERIPHERAL_ADDR);
    i2c_write_byte(0x40);  // Control byte for dat
    i2c_write_byte(data);
    i2c_stop();
}
void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t pos = (row == 0) ? col : 0x40 + col;
    lcd_command(0x80 | pos);
}
void lcd_clear(void) {
    lcd_command(LCD_CLEAR);
}
void keypad_init(void) {
    P2DIR &= ~(BIT0 | BIT1 | BIT2 | BIT3);  // Rows as input
    P2REN |= (BIT0 | BIT1 | BIT2 | BIT3);   // Enable pull-up/down resistors
    P2OUT |= (BIT0 | BIT1 | BIT2 | BIT3);   // Pull-up
    P3DIR |= (BIT0 | BIT1 | BIT2 | BIT3);   // Columns as output
}

uint8_t keypad_get_key(void) {
    uint8_t row, col;
    for (col = 0; col < 4; col++) {
        P3OUT = ~(1 << col);
        __delay_cycles(1000);
        row = P2IN & (BIT0 | BIT1 | BIT2 | BIT3);
        if (row != (BIT0 | BIT1 | BIT2 | BIT3)) {
            break;
        }
    }
    if (col == 4) return 0xFF;

    uint8_t keys[4][4] = {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
    };
    return 0;
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
        switch (key) {
            case 'C':
                cursor_on = !cursor_on;
                lcd_command(cursor_on ? LCD_CURSOR_ON : LCD_CURSOR_OFF);
                break;
            case '9':
                blink_on = !blink_on;
                lcd_command(blink_on ? LCD_BLINK_ON : LCD_BLINK_OFF);
                break;
            case '*':
                send_to_led(0x01);
                break;
            case '#':
                lcd_clear();
                break;
            default:
                send_to_led(key);
                break;
        }
    }
}
void update_lcd_display(uint8_t key) {
    lcd_set_cursor(1, 15);
    lcd_write_char(key);
    lcd_set_cursor(0, 0);
    switch (key) {
        case '0': lcd_write_char('s'); lcd_write_char('t'); lcd_write_char('a'); lcd_write_char('t'); lcd_write_char('i'); lcd_write_char('c'); break;
        case '1': lcd_write_char('t'); lcd_write_char('o'); lcd_write_char('g'); lcd_write_char('g'); lcd_write_char('l'); lcd_write_char('e'); break;
        default: lcd_write_char('u'); lcd_write_char('n'); lcd_write_char('k'); lcd_write_char('n'); lcd_write_char('o'); lcd_write_char('w'); break;
    }
}
void send_to_led(uint8_t pattern) {
    i2c_start();
    i2c_write_byte(LED_PERIPHERAL_ADDR);
    i2c_write_byte(pattern);
    i2c_stop();
}
