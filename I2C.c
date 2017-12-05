/*
 *            i2c.c
 *
 *   Created on:  November 12, 2017
 *  Last Edited:  November 12, 2017
 *       Author:  Nick Gorab
 *        Board:  MSP430FR5994
 */

#include "I2C.h"

int     dataGot;            // Determines if all of the extected data is received
char    height[2],          // Creates an array to store height data
        attitude[6],        // Creates an array to store attitude data
        *pointer;            // Creates a pointer to access the array 

void getHeight(void){
    dataGot  = 0;                           // Resets the data received variable
    *pointer = height;                      // Sets the pointer to the height array

    UCB0I2CSA = VCNL4200_ADDRESS;           // Sets slave address
    UCB0TBCNT = 0x01;                       // Expecting to send 1 data byte
    UCB0CTL1 &= ~UCSWRST;                    // Leaves reset state, USCI starts operation
    UCB0CTL1 |= UCTR                        // Enables TX Mode
             |  UCTXSTT;                    // Sends start condition
    __bis_SR_register(LPM0_bits | GIE);     // Enters Low-Power mode and enables global interrupt
    
    UCB0CTL1 |= UCWRST;                     // Enters reset state, USCI stops operation
    UCB0TBCNT = 0x02;                       // Expecting to receive 2 bytes of data
    UCB0CTL1 &= ~UCTR;                      // Enters RX Mode
    UCB0CTL1 &= ~UCSWRST;                    // Leaves reset state, USCI starts operation
    UCB0CTL1 |= UCTXSTT;                    // Sends start condition
    while(~dataGot){                        // While the entire message is not received
        __bis_SR_register(LPM0_bits | GIE); // Enters Low-Power mode and enables global interrupt
        __no_operation();                   // For debugger
}  }

void getAccel(void){
    dataGot  = 0;                           // Resets the data received variable
    *pointer = attitude;                    // Sets the pointer to the attitude array

    UCB0I2CSA = LSM6DSL_ADDRESS;            // Sets the slave address
    UCB0TBCNT = 0x01;                       // Expecting to send 1 data byte
    UCB0CTL1 &= ~UCSWRST;                    // Leaves reset state, USCI starts operation
    UCB0CTL1 |= UCTR                        // Enables TX Mode
             |  UCTXSTT;                    // Sends start condition
    __bis_SR_register(LPM0_bits | GIE);     // Enters Low-Power mode and enables global interrupt
    
    UCB0CTL1 |= UCWRST;                     // Enters reset state, USCI stops operation
    UCB0TBCNT = 0x06;                       // Expecting to receive 2 bytes of data
    UCB0CTL1 &= ~UCTR;                      // Enters RX Mode
    UCB0CTL1 &= ~UCSWRST;                    // Leaves reset state, USCI starts operation
    UCB0CTL1 |= UCTXSTT;                    // Sends start condition
    while(~dataGot){                        // While the entire message is not received
        __bis_SR_register(LPM0_bits | GIE); // Enters Low-Power mode and enables global interrupt
        __no_operation();                   // For debugger
}  }


#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void) {
    switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
    {
        case USCI_I2C_UCNACKIFG:                    // NACK Interrupt
            UCB0CTL1 |= UCTXSTT;                    // I2C start condition
        break;

        case USCI_I2C_UCRXIFG0:                     // I2C RX Interrupt
            *pointer = UCB0RXBUF;                   // Points to and sets array index value
            poniter++;                              // Increments the array pointer
            __bic_SR_register_on_exit(LPM0_bits);   // Exits Low-Power mode 
        break;

        case USCI_I2C_UCTXIFG0:                     // I2C TX Interrupt
            UCB0TXIFG = 
            __bic_SR_register_on_exit(LPMO_bits);   // Exits Low-Power mode
        break; 

        case USCI_I2C_UCBCNTIFG:                    // Byte counter interrupt
            dataGot = 1;                            // Data received flag
            pointer = 0;                            // Resets the pointer value to first value
        break;
        default: break;
}   }