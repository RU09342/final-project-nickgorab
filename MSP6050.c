/*
 *            MPU6050.c
 *
 *   Created on:  December 14, 2017
 *  Last Edited:  December 14, 2017
 *       Author:  Nick Gorab
 *        Board:  MSP430F5994
 */

#include "MPU6050.h"

#define MPU6050_ADDRESS 0x68;

int     RX_Byte_Ctr,                    // Coutner to make sure all of the information is received
        TX_Byte_Ctr;                    // Counter to make sure all of the information is sent
char    accelData[6],                   // Creates an array to store accelerometer data
        txData[3] = {0x6B, 0x00, 0x3B}, // Array which contains data being sent to the accelerometer
       *txPtr,                          // Pointer which points to the txData array
       *accelPtr;                       // Pointer which points to the accelData array



/***********************************\
*                                   *
*  Acceleration Acquiring Function  *
*                                   *
\***********************************/

char getAccel(void){
    txPtr = &txData[2];                     // Points to the third value of the txData array
    RX_Byte_Ctr = 6;                        // Sets the amount of bytes to be sent
    TX_Byte_Ctr = 1;                        // Sets the amount of bytes to send
    UCB0TBCNT = 0x02;                       // Byte counter set to 1 higher than expected to avoid stop condition
    UCB0I2CSA = MPU6050_ADDRESS;            // Sets slave address

    UCB0CTL1 |= UCTR                        // Enables TX Mode
             |  UCTXSTT;                    // Sends start condition
    __bis_SR_register(LPM0_bits | GIE);     // Waits in low power mode until all data is sent
    UCB0TBCNT  = 0x06;                      // Sends stop condition when all of the data is sent
    __delay_cycles(20);                     // Waits for a little until the buffer is loaded
    UCB0CTL1 &= ~UCTR;                      // Enters RX Mode
    UCB0CTL1 |= UCTXSTT;                    // Sends start condition
    __bis_SR_register(LPM0_bits | GIE);     // Enters Low-Power mode and enables global interrupt
    return accelData[6];                    // Returns the accelerometer data for use in the main code
}



/**************************\
*                          *
*  MPU6050 Initialization  *
*                          *
\**************************/

void mpu6050Init(void){
    txPtr = &txData[0];                     // Established the txData pointer
    accelPtr = &accelData[0];               // Establishes the accelData pointer
    TX_Byte_Ctr = 2;                        // Sets the amount of bytes to be sent
    UCB0I2CSA = MPU6050_ADDRESS;            // Sets slave address
    UCB0TBCNT  = 0x02;                      // Expecting to receive 2 bytes of data
    UCB0CTL1 |= UCTR                        // Enables TX Mode
             |  UCTXSTT;                    // Sends start condition
    __bis_SR_register(LPM0_bits | GIE);     // Enters Low-Power mode until all bytes sent
}



#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void) {
    switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG)) {
        case USCI_I2C_UCNACKIFG:                            // NACK Interrupt
            UCB0CTL1 |= UCTXSTT;                            // I2C start condition
        break;

        case USCI_I2C_UCRXIFG0:                             // I2C RX Interrupt
            if(RX_Byte_Ctr > 1){                            // Checks if there is more data to be received
               *accelPtr = UCB0RXBUF;                       // Loads the data array
                accelPtr++;                                 // Increments the pointer
                RX_Byte_Ctr--;                              // Decrement RX byte counter
            } else if (RX_Byte_Ctr == 1){                   // If there is only one byte left
               *accelPtr = UCB0RXBUF;                       // Loads the data array
                accelPtr = 0;                               // Resets the pointer to overwrite the data
                __bic_SR_register_on_exit(LPM0_bits);       // Exit LPM0
            }
        break;

        case USCI_I2C_UCTXIFG0:                             // I2C TX Interrupt
            if(TX_Byte_Ctr > 1){                            // If there is more than 1 byte to be received
                UCB0TXBUF = *txPtr;                         // Sends the data being pointed to
                txPtr++;                                    // Increments the address pointed to
                TX_Byte_Ctr--;                              // Decrements the byte counter
            } else if (TX_Byte_Ctr == 1){                   // If there is only one more byte to be received
                UCB0TXBUF = *txPtr;                         // Sends the last byte of data
                __bic_SR_register_on_exit(LPM0_bits);       // Exit LPM0
        }
        break;
}   }
