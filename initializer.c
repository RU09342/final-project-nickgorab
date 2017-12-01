/*
 *            initializer.c
 *
 *   Created on:  November 12, 2017
 *  Last Edited:  November 12, 2017
 *       Author:  Nick Gorab
 *        Board:  MSP430FR5994
 */

/***********************************************************************************************************\
*                                                                                                           *
*  The purpose of this file is to enable all of the relevant registers used in this code to avoid clutter.  *
*                                                                                                           *
\***********************************************************************************************************/

#include "initializer.h"



/************************\
 *                      *
 *  ADC Initialization  *
 *                      *
\************************/

void adcInit(void){
    P8SEL0    |= BIT7;          // ADC readings taken on A4 (Pin 1.1)
    ADC12MCTL0 = ADC12INCH_4;   // ADC input channel 4 (A4)
    ADC12CTL0  = ADC12ON        // Turns on ADC12
               + ADC12SHT0_8    // Sets sampling time
               + ADC12MSC;      // Sets up multiple sample conversion
    ADC12CTL1  = ADC12SHP       // Signal comes from sampling timer instead of input
               + ADC12CONSEQ_2; // Repeats reading on single channel
    ADC12CTL0 |= ADC12ENC       // Enables conversions
              |  ADC12SC;       // Starts conversions
}




/************************\
*                        *
*   LED Initialization   *
*                        *
\************************/

void ledInit(void){
    P1DIR |= BIT0       // Red LED
          |  BIT1;      // Green LED
}



/************************\
 *                      *
 *  DCO Initialization  *
 *                      *
\************************/

void dcoInit(void){
  CSCTL0_H = CSKEY_H;       // Unlocks clock registers for writing
  CSCTL1   = DCOFSEL_3      // Sets DCO at 8MHz
           | DCORSEL;       // Sets DCO at 8MHz
  CSCTL2   = SELA__VLOCLK   // Selects ACLK source of VLOCLK
           | SELS__DCOCLK   // SMCLK source as DCOCLK
           | SELM__DCOCLK;  // MCLK source as DCOCLK
  CSCTL3   = DIVA__1        // ACLK source divider of 2
           | DIVS__1        // SMCLK source divider of 2
           | DIVM__1;       // MCLK source divider of 2
  CSCTL0_H = 0;             // Locks clock registers 
}



/************************\
*                        *
* TIMER_B Initialization *
*                        *
\************************/

void timerInit(void){
    TB0CCTL0 = CCIE;        // Enables timer interrupts
    TB0CTL   = TBSSEL_1     // SMCLK
             + MC_1;        // Runs in up-mode
    TB0CCTL1 = OUTMOD_7;    // Set/Reset Mode M4
    TB0CCTL2 = OUTMOD_7;    // Set/Reset Mode M3
    TB0CCTL3 = OUTMOD_7;    // Set/Reset Mode M2
    TB0CCTL4 = OUTMOD_7;    // Set/Reset Mode M1
    TB0CCR0  = 100;         // Counts up to 100
}



/*************************\
 *                       *
 *  UART Initialization  *
 *                       *
\*************************/

void uartInit(void) {
    P4SEL0    |= BIT3;      // UART TX
    P4SEL0    |= BIT2;      // UART RX
    UCA0CTLW0 |= UCSWRST;   // State machine reset
    UCA0CTLW0 |= UCSSEL1;   // Uses SMCLK as source
    UCA0BR0    = 52;        // Modulation
    UCA0MCTLW  = UCBRF_1    // Modulation
              | UCOS16      // Modulation
              | 0x4900;     // Modulation
    UCA0BR1    = 0x00;      // Modulation
    UCA0CTLW0 &= ~UCSWRST;  // Starts state machine
    UCA0IE |= UCRXIE;       // Enable USCI_A0 RX interrupt
}



/************************\
*                        *
*   I2C Initialization   *
*                        *
\************************/

void i2cInit(void){
    P3SEL0    |= BIT0       // SDA line for I2C using UCB0
               + BIT1;      // SCL line for I2C using UCB0
    P3SEL1    |= BIT0       // SDA line for I2C using UCB0
               + BIT1;      // SCL line for I2C using UCB0
    UCB0CTL1  |= UCSWRST;   // USCI held in reset for initialization
    UCB0CTLW0  = UCMST      // Master Mode
               + UCMODE_3   // I2C Mode
               + UCSYNC;    // Synchronus Mode
    UCB2BRW    = 0x0008;    // SMCLK/8
    UCB2CTLW1 |= UCASTP_2;  // Sends stop bit when UCTBCNT is reached
    UCB2CTL1  &= ~UCSWRST;  // Starts state machine
}



/*****************************\
*                             *
*   VCNL4200 Initialization   *
*                             *
\*****************************/

void vcnl4200Init(void){

}



/****************************\
*                            *
*   LSM6DSL Initialization   *
*                            *
\****************************/

void lsm6dslInit(void){

}