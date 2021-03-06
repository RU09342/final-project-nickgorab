

#include <msp430.h>
#define VCNL4200_ADDRESS 0x50;

int     RX_Byte_Ctr,        // Coutner to make sure all of the information is received
        TX_Byte_Ctr,        // Counter to make sure all of the information is sent
        i;                  // Integer used for counting sent bytes
char    height[5],          // Creates an array to store height data
        sending[2],         // Creates an array to store the sent data
        *pointer;           // Creates a pointer to access the array



void i2cInit(void){
    P1SEL0    |= BIT6       // SDA line for I2C using UCB0
              |  BIT7;      // SCL line for I2C using UCB0
    UCB0CTLW0 |= UCSWRST;   // Enters reset state, USCI stops operation
    UCB0TBCNT = 0x06;       // Expecting to receive 2 bytes of data
    UCB0CTLW1 |= UCASTP_2;  // Sends stop bit when UCTBCNT is reached
    UCB0CTLW0 |= UCMST      // Master Mode
              |  UCMODE_3   // I2C Mode
              |  UCSSEL_3;  // Sets SMCLK as source
    UCB0BRW    = 0x000A;    // SMCLK/10
    UCB0CTLW0 &= ~UCSWRST;  // Exits reset mode, USCI enters operation
    UCB0IE    |= UCTXIE0    // Data received interrupt enable
              |  UCRXIE0    // Data ready to transmit interrupt enable
              |  UCNACKIE;  // NACK interrupt enable
}




void getHeight(void){

    pointer = height;                       // Sets the pointer to the height array
    RX_Byte_Ctr = 5;                        // Determines the number of bytes received 
    TX_Byte_Ctr = 2;                        // Determines the number of bytes sent
    UCB0I2CSA = VCNL4200_ADDRESS;           // Sets slave address

    UCB0CTL1 |= UCTR                        // Enables TX Mode
             |  UCTXSTT;                    // Sends start condition
    __bis_SR_register(LPM0_bits | GIE);     // Enters Low-Power mode and enables global interrupt

    UCB0CTL1 &= ~UCTR;                      // Enters RX Mode
    UCB0CTL1 |= UCTXSTT;                    // Sends start condition
    __bis_SR_register(LPM0_bits | GIE);     // Enters Low-Power mode and enables global interrupt
}


void main(void){
    WDTCTL = WDTPW | WDTHOLD;       // DIsables the Watchdog Timer
    PM5CTL0 &= ~LOCKLPM5;           // Exits High-Impedance mode

    i2cInit();                      // Initializes I2C communication

    getHeight();                    // Get data 
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
}   }