/* 
 * File:   Stroboscope.c
 * Author: Umair
 *
 * Created on July 22, 2023, 6:52 PM
 */
/* 
 * File:   DigitalRotaryEncoder.c
 * Author: Umair
 *
 * Created on July 16, 2023, 5:45 PM
 * 
 * Read the position on a digital rotary encoder.
 * A count is kept on a MAX7219 8-digit 7-segment display
 */

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "../include/proc/pic16f616.h"

#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select bit (MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config IOSCFS = 8MHZ    // Internal Oscillator Frequency Select bit (8 MHz)
#pragma config BOREN = OFF      // Brown-out Reset Selection bits (BOR Disabled)

#define _XTAL_FREQ 8000000

#define DIN PORTCbits.RC2   // Pin 8 (Orange)
#define CS  PORTCbits.RC1   // Pin 9 (Brown)
#define CLK PORTCbits.RC0   // Pin 10 (White)
#define BUT PORTAbits.RA2   // Pin 11 (Green)
#define INB PORTAbits.RA1   // Pin 12 (Blue)
#define INA PORTAbits.RA0   // Pin 13 (Purple)
#define INA_F INTCONbits.RAIF   // Interrupt flag
#define OUT_LED PORTCbits.RC3   // LED Output

volatile uint8_t int_F = 0;
volatile uint32_t millisCount = 0;
uint32_t prevTime = 0;
uint32_t prevTime_button = 0;
uint32_t prevTime_print = 0;
uint32_t prevTime_LED = 0;
long speedRPM = 0;
int resolution_i = 3;
uint8_t resolutions[5] = {1, 5, 50, 500, 5000};
uint32_t TimeHIGH;
bit LED_state = 0;


void SPI_send(uint8_t addr, uint8_t data);
void MAX7219_Setup();
void display_digit(int digit_X, int val);
void display_num(uint32_t val);
void millisInit();
uint32_t millis();


void __interrupt() ISR_function(void){
    if (INA_F){     // If the IOC has triggered
        int_F = 1;  // set the indicator flag
        INA_F = 0;  // Clear the flag bit in register INTCON
    }
    if (INTCONbits.T0IF) {
        INTCONbits.T0IF = 0;
        millisCount++;
        TMR0 = initialValue;
    }
}

int main(int argc, char** argv) {
    TRISCbits.TRISC2 = 0;   // DIN
    TRISCbits.TRISC1 = 0;   // CS
    TRISCbits.TRISC0 = 0;   // CLK
    TRISCbits.TRISC3 = 0;   // LED
    ANSEL = 0;              // All pins as Digital IO
    IOCAbits.IOC0 = 1;      // IOC enabled for RA0
    INTCONbits.GIE = 1;     // "Enables all unmasked interrupts"
    INTCONbits.RAIE = 1;    // "Enables the PORTA change interrupt"
    
    CLK = 0;
    CS = 1;
    DIN = 0;
    MAX7219_Setup();
    millisInit();
    
    uint16_t counter = 5;
    uint8_t dir;            // Direction
    int_F = 0;              // Interrupt Flag
    display_num(12345678);
    
    while(1){
        if (int_F){
            dir = (INA + INB)%2;        // XOR doesn't seem to work...
            counter += dir*2 - 1;
            display_num(counter);
            int_F = 0;
        }
        
        if (int_F){
            dir = (INA + INB)%2;
            speedRPM += resolutions[resolution_i-1]*dir;
            if (speedRPM <= 1){
                speedRPM = 10;
            }
            int_F = 0;
            TimeHIGH = 60000000/speedRPM;	// µs 
        }
        
        if(millis() - prevTime_print >= 500){   // Update display every 500 ms
            display_num(speedRPM);
            // I can implement a speedRPM_old
            // so that it only updates if changed
        }
        
    }
    
    return (EXIT_SUCCESS);
}

void SPI_send(uint8_t addr, uint8_t data){
    int i;
    CS = 0;
    for (i=0; i<8; i++){
        CLK = 0;
        DIN = (addr & 0b10000000) ? 1:0 ;
        CLK = 1;
        addr = addr << 1;
    }
    for (i=0; i<8; i++){
        CLK = 0;
        DIN = (data & 0b10000000) ? 1:0 ;
        CLK = 1;
        data = data << 1;
    }
    CLK = 0;
    CS = 1;
}

void MAX7219_Setup(){
    SPI_send(0x0F,0x00);        // Display normal operation
    SPI_send(0x09,0xFF);        // Decode mode ON
    SPI_send(0x0A,0x08);        // Intensity level
    SPI_send(0x0B,0x07);        // Scan-Limit
    SPI_send(0x0C,0x01);        // Shutdown (turn it on)
}

void display_digit(int digit_X, int val){
    SPI_send(digit_X, val);
}

void display_num(uint32_t val){
    int i;
    for (i=1; i<=8; i++){
        display_digit(i, val%10);
        val /= 10;
    }
}

void millisInit() {
	OPTION_REGbits.T0CS = 0;    // Use internal clock (Fosc/4)
    OPTION_REGbits.PSA = 0;     // Assign prescaler to Timer0
    OPTION_REGbits.PS = 0b101;  // Prescaler 1:64
    TMR0 = 21;
    INTCONbits.T0IE = 1;        // Enable Timer0 interrupt
    INTCONbits.GIE = 1;         // Enable global interrupts
}

uint32_t millis() {
    INTCONbits.GIE = 0;
    uint32_t result = millisCount;    
    INTCONbits.GIE = 1;
    return result;
}