/*
 *            I2C.h
 *
 *   Created on:  November 7, 2017
 *  Last Edited:  December 11, 2017
 *       Author:  Nick Gorab
 *        Board:  MSP430F5994
 */



#define MPU6050_ADDRESS 0x68;

int     RX_Byte_Ctr,                    // Coutner to make sure all of the information is received
        TX_Byte_Ctr;                    // Counter to make sure all of the information is sent
char    accelData[6],                   // Creates an array to store accelerometer data
        txData[3] = {0x6B, 0x00, 0x3B}, // Array which contains data being sent to the accelerometer
       *txPtr,                          // Pointer which points to the txData array
       *accelPtr;                       // Pointer which points to the accelData array



void getAccel(){
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
}
