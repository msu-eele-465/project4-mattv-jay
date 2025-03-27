/**
 * @file lcd_driver.c
 * @brief Test file to develop code for writing to LCD.
 */

#include "intrinsics.h"
#include <msp430fr2310.h>
#include <stdint.h>
#include <stdbool.h>

#define I2C_ADDR 0x49

#define RS_HIGH P2OUT |= BIT6
#define RS_LOW P2OUT &= ~BIT6
#define E_HIGH P2OUT |= BIT7
#define E_LOW P2OUT &= ~BIT7

#define LCD_DATA P1OUT
#define DB4 BIT4
#define DB5 BIT5
#define DB6 BIT6
#define DB7 BIT7

unsigned int pattern_num = 0; // Tracks which pattern is active

bool unlocked = false;
char key = '\0';

unsigned int time_since_active = 3;

void enable_lcd(void);
void send_cmd(unsigned char cmd);
void send_char(unsigned char character);
void send_string(const char *str);

/**
 * Initializes all GPIO ports.
 */
void init_gpio(void)
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
void init_timer(void)
{
    // TB0CTL = TBSSEL__ACLK | MC_1 | TBCLR | ID__2; // ACLK, up mode, clear TBR, divide by 2
    // TB0CCR0 = 16384; // Set up 1.0s period
    // TB0CCTL0 &= ~CCIFG; // Clear CCR0 Flag
    // TB0CCTL0 |= CCIE; // Enable TB0 CCR0 Overflow IRQ

    TB1CTL = TBSSEL__ACLK | MC_2 | TBCLR | ID__8 | CNTL_1; // ACLK, continuous mode, clear TBR, divide by 8, length
                                                           // 12-bit
    TB1CTL &= ~TBIFG; // Clear CCR0 Flag
    TB1CTL |= TBIE; // Enable TB1 Overflow IRQ
}

/**
 * Sets all I2C parameters.
 */
void init_i2c(void)
{
    UCB0CTLW0 = UCSWRST; // Software reset enabled
    UCB0CTLW0 |= UCMODE_3 | UCSYNC; // I2C mode, sync mode
    UCB0I2COA0 = I2C_ADDR | UCOAEN; // Own Address and enable
    UCB0CTLW0 &= ~UCSWRST; // clear reset register
    UCB0IE |= UCRXIE; // Enable I2C read interrupt
}

void init_lcd(void)
{
    __delay_cycles(30000); // Wait 30 ms
    LCD_DATA |= DB5;
    enable_lcd();
    send_cmd(0x2C);
    // LCD_DATA |= DB5;
    // LCD_DATA |= DB7 | DB6;
    __delay_cycles(100); // Wait 100 micro seconds
    send_cmd(0x0C);
    // LCD_DATA &= ~(DB7 | DB6 | DB5 | DB4);
    // LCD_DATA |= (DB7 | DB6);
    __delay_cycles(100); // Wait 100 micro seconds
    send_cmd(0x01);
    // LCD_DATA &= ~(DB7 | DB6 | DB5 | DB4);
    // LCD_DATA |= DB4;
    __delay_cycles(1600); // Wait 1.6 ms
    send_cmd(0x06);
    // LCD_DATA &= ~(DB7 | DB6 | DB5 | DB4);
    // LCD_DATA |= (DB6 | DB5);
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
    init_gpio();
    init_timer();
    init_i2c();
    
    init_lcd();
    // send_cmd(0x01);
    __delay_cycles(10000);
    send_char('0');
    send_string(PATTERNS[0]);

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

void enable_lcd(void)
{
    E_HIGH;
    // __delay_cycles(450);
    E_LOW;
    // __delay_cycles(1000);
}

void send_cmd(unsigned char cmd)
{
    RS_LOW;
    LCD_DATA = (LCD_DATA & 0x0F) | (cmd & 0xF0);
    enable_lcd();
    LCD_DATA = (LCD_DATA & 0x0F) | ((cmd & 0x0F) << 4);
    enable_lcd();
}

void send_char(unsigned char character)
{
    RS_HIGH;
    LCD_DATA = (LCD_DATA & 0x0F) | (character & 0xF0);
    enable_lcd();
    LCD_DATA = (LCD_DATA & 0x0F) | ((character & 0x0F) << 4);
    enable_lcd();
}

void send_string(const char *str)
{
    unsigned int i;
    for (i = 0; str[i] != 0; i++)
    {
        send_char(str[i]);
    }
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
