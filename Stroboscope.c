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

// I think the actual frequency is 7.360 MHz from measurements
// But that estimate is not accurate because I didn't consider overhead 
#define _XTAL_FREQ 8000000

#define DIN PORTCbits.RC2   // Pin 8 (Orange)
#define CS  PORTCbits.RC1   // Pin 9 (Brown)
#define CLK PORTCbits.RC0   // Pin 10 (White)
#define BUTTON_PIN PORTAbits.RA2   // Pin 11 (Green)
#define INB PORTAbits.RA1   // Pin 12 (Blue)
#define INA PORTAbits.RA0   // Pin 13 (Purple)
#define INA_F INTCONbits.RAIF   // Interrupt flag
#define LED_PIN PORTCbits.RC3   // LED Output

// 163 and 0b001 gives 200 us resolution
#define initialValue 163

// Global Variables
volatile bit int_F = 0;
volatile uint32_t millisCount = 0;
//uint32_t prevTime = 0;
uint16_t prevTime_button = 0;
uint16_t prevTime_display = 0;
uint16_t prevTime_LED = 0;
uint16_t speedRPM = 0;
uint16_t TimePeriod;
uint8_t resolution_i = 3;
//uint16_t resolutions[5] = {1, 5, 50, 500, 5000};

bit LED_state = 0;
bit button_pressed = 0;

// Function Protoypes
void SPI_send(uint8_t addr, uint8_t data);
void MAX7219_Setup();
void display_digit(int digit_X, int val);
void display_num(uint32_t val);
void millisInit();
uint32_t millis();
uint32_t micros();
void ButtonBounce();

// ISR function
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

int main(int argc, char** argv) {\
    // IO init
    TRISCbits.TRISC2 = 0;   // DIN
    TRISCbits.TRISC1 = 0;   // CS
    TRISCbits.TRISC0 = 0;   // CLK
    TRISCbits.TRISC3 = 0;   // LED
    ANSEL = 0;              // All pins as Digital IO
    
    // IOC init
    IOCAbits.IOC0 = 1;      // IOC enabled for RA0
    INTCONbits.GIE = 1;     // "Enables all unmasked interrupts"
    INTCONbits.RAIE = 1;    // "Enables the PORTA change interrupt"
    
    // millis (timer 0) init
    OPTION_REGbits.T0CS = 0;    // Use internal clock (Fosc/4)
    OPTION_REGbits.PSA = 0;     // Assign prescaler to Timer0
    OPTION_REGbits.PS = 0b001;  // Prescaler 1:64
    INTCONbits.T0IE = 1;        // Enable Timer0 interrupt
    INTCONbits.GIE = 1;         // Enable global interrupts
    
    CLK = 0;
    CS = 1;
    DIN = 0;
    TMR0 = initialValue;
    
    MAX7219_Setup();
        
    uint8_t dir;            // Direction
    int_F = 0;              // Interrupt Flag
    display_num(12345678);  // confirm it's working
    __delay_ms(2000);       // 2 second wait
    
    while(1){
        // Polling button
        ButtonBounce();
        
        // Every time the button is pressed and released
        // And only apply once every 250 ms
        if ((millis() - prevTime_button >= 250) && button_pressed){
            resolution_i = (resolution_i++)%5+1;
            prevTime_button = millis();
        }
        
        // Every time the IRE turns one click, update value
        if (int_F){
            dir = ((INA + INB)%2)*2 - 1;    // +1 or -1
            // speedRPM += resolutions[resolution_i-1]*dir;
            // Select speed increase/decrease
            switch (resolution_i) {
                case 1:
                    speedRPM += 1*dir;
                    break;
                case 2:
                    speedRPM += 5*dir;
                    break;
                case 3:
                    speedRPM += 50*dir;
                    break;
                case 4:
                    speedRPM += 500*dir;
                    break;
                case 5:
                    speedRPM += 5000*dir;
                    break;
            }
            
            if (speedRPM <= 1){
                speedRPM = 10;
            }
            int_F = 0;
            TimePeriod = 60000000L/speedRPM;	// 탎
        }
        
        // Time Period is the period of time per blink
        // After converting RPM to microseconds
        
        if ((micros() - prevTime_LED >= 9.8*TimePeriod) && (!LED_state)){
            LED_PIN = 1;                // Turn it on
            LED_state = 1;              // Set the flag
            prevTime_LED = micros();    // Update time
        }
        else if ((micros() - prevTime_LED >= 0.2*TimePeriod) && (LED_state)){
            LED_PIN = 0;                // Turn it off
            LED_state = 0;              // Clear the flag
            prevTime_LED = micros();    // Update time
        }
        
        // Update display every 500 ms
        if(millis() - prevTime_display >= 500){   
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
    TMR0 = initialValue;
    INTCONbits.T0IE = 1;        // Enable Timer0 interrupt
    INTCONbits.GIE = 1;         // Enable global interrupts
}

uint32_t millis() {
    INTCONbits.GIE = 0;
    uint32_t result = millisCount/5; // One overflow every 200 탎
    INTCONbits.GIE = 1;
    return result;
}

// Resolution for micros is not very good
uint32_t micros() {
    INTCONbits.GIE = 0;
    // One overflow happens every 200 탎
    // and each TIMER0 count adds 2.17 탎
    // (200 탎)/(255-163 count) = 2.173913043 탎/count
    uint32_t result = millisCount*200 + (TMR0-initialValue)*2.17;
    INTCONbits.GIE = 1;
    return result;
}

void ButtonBounce(){
	static long Button_f = 0;
    
    // Increment counter while button is held down
    Button_f += !BUTTON_PIN;

    // If held down long enough, reset the speed
	if (Button_f >= 60000){
		speedRPM = 1;
		Button_f = 0;
	}
    // If button was pressed and released
	else if (Button_f && BUTTON_PIN){
		button_pressed = 1;
		Button_f = 0;
	}
    // If button not pressed or released
	else {
		button_pressed = 0;
	}
}

