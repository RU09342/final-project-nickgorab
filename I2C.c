/*
 *            I2C.h
 *
 *   Created on:  November 7, 2017
 *  Last Edited:  December 11, 2017
 *       Author:  Nick Gorab
 *        Board:  MSP430F5994
 */


#define VCNL4200_ADDRESS 0x50;  // Prox sensor address
#define LSM6DSL_ADDRESS  0x6A; // Accelerometer sensor address

int     RX_Byte_Ctr,            // Coutner to make sure all of the information is received
        TX_Byte_Ctr,            // Counter to make sure all of the information is sent
        i;                      // Integer used for counting sent bytes
char    height[5],              // Creates an array to store height data
        sending[2],             // Creates an array to store the sent data
        *pointer;               // Creates a pointer to access the array

void getHeight(void){

    pointer = height;                       // Sets the pointer to the height array
    RX_Byte_Ctr = 5;                        // Determines the number of bytes received 
    TX_Byte_Ctr = 2;                        // Determines the number of bytes sent
    UCB0I2CSA = VCNL4200_ADDRESS;           // Sets slave address to the proximity sensor

    UCB0CTL1 |= UCTR                        // Enables TX Mode
             |  UCTXSTT;                    // Sends start condition
    __bis_SR_register(LPM0_bits | GIE);     // Enters Low-Power mode and enables global interrupt

    UCB0CTL1 &= ~UCTR;                      // Enters RX Mode
    UCB0CTL1 |= UCTXSTT;                    // Sends start condition
    __bis_SR_register(LPM0_bits | GIE);     // Enters Low-Power mode and enables global interrupt
    i = 0;
    RX_Byte_Ctr = 0;
    TX_Byte_Ctr = 0;
}


void getAccel(void){

    pointer = accel;                        // Sets the pointer to the acceleration array
    RX_Byte_Ctr = 6;                        // Expecting to receive 6 bytes of data
    TX_Byte_Ctr = 1;                        // Only should send 1 byte of data
    UCB0I2CSA = LSM6DSL_ADDRESS;            // Sets the slave address to the accelerometer

    UCB0CTL1 |= UCTR                        // Enables TX Mode
             |  UCTXSTT;                    // Sends start condition
    __bis_SR_register(LPM0_bits | GIE);     // Enters Low-Power mode and enables global interrupt

    UCB0CTL1 &= ~UCTR;                      // Enters RX Mode
    UCB0CTL1 |= UCTXSTT;                    // Sends start condition
    __bis_SR_register(LPM0_bits | GIE);     // Enters Low-Power mode and enables global interrupt
    i = 0;
    RX_Byte_Ctr = 0;
    TX_Byte_Ctr = 0;
}


#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void) {
    switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG)) {
        case USCI_I2C_UCNACKIFG:                            // NACK Interrupt
            UCB0CTL1 |= UCTXSTT;                            // I2C start condition
        break;

        case USCI_I2C_UCRXIFG0:                             // I2C RX Interrupt
            if (RX_Byte_Ctr){                               // Checks if there is more data to be received
                *pointer = UCB0RXBUF;                       // Loads the data array
                pointer++;                                  // Increments the pointer
                RX_Byte_Ctr--;                              // Decrement RX byte counter
            } else {                                        // If all of the data is received
                __bic_SR_register_on_exit(LPM0_bits);       // Exit LPM0
            }
        break;

        case USCI_I2C_UCTXIFG0:                             // I2C TX Interrupt
            if(TX_Byte_Ctr){                                // If there is more data to send
                UCB0TXBUF = sending[i];                     // Loads TX buffer from array
                i++;                                        // Increments array position
                TX_Byte_Ctr--;                              // Decrements TX byte counter
            } else {                                        // If there is nothing left to say
                __bic_SR_register_on_exit(LPM0_bits);       // Exits Low-Power mode
            }
        break;

        case USCI_I2C_UCBCNTIFG:                // Vector 26: BCNTIFG
            __bic_SR_register_on_exit(LPM0_bits);
        break;
}   }