#include <msp430.h>
#define RM25_ADDRESS 0x50;

int     RX_Byte_Ctr;        // Coutner to make sure all of the information is received
int     TX_Byte_Ctr;        // Counter to make sure all of the information is sent
int i  =0;
int v = 0;
int MSB, LSB;
char    rxData[6],          // Creates an array to store height data
        txData[6],
        dataBuff[6];        // Data buffer which stores data to be sent




void i2cInit(void){
    P1SEL0    |= BIT6           // SDA line for I2C using UCB0
              |  BIT7;          // SCL line for I2C using UCB0
    UCB0CTLW0 |= UCSWRST;       // Enters reset state, USCI stops operation
    UCB0CTLW1 |= UCASTP_2;      // Sends stop bit when UCTBCNT is reached
    UCB0CTLW0 |= UCMST          // Master Mode
              |  UCMODE_3       // I2C Mode
              |  UCSSEL__SMCLK; // Synchronous Mode
    UCB0BRW    = 0x000A;        // SMCLK/10
    UCB0CTLW0 &= ~UCSWRST;      // Exits reset mode, USCI enters operation
    UCB0IE    |= UCTXIE0        // Data received interrupt enable
              |  UCRXIE0;       // Data ready to transmit interrupt enable
}


void singleWrite(int address, char *txData){
    UCB0I2CSA = RM25_ADDRESS;                            // Sets slave address
    MSB = address & 0xFF;                            // Sets most significant byte
    LSB = address >> 8;                             // Sets least significant byte
    TX_Byte_Ctr = 3;                                // Sets the amount of bytes to be sent
    UCB0TBCNT  = 0x03;          // Expecting to receive 2 bytes of data

    dataBuff[0] = MSB;
    dataBuff[1] = LSB;
    dataBuff[2] = 0xF2;              // Loads information into data buffer

    UCB0CTL1 |= UCTR                                // Enables TX Mode
             |  UCTXSTT;                            // Sends start condition
    __bis_SR_register(LPM0_bits | GIE);             // Enters Low-Power mode and enables global interrupt
}


void singleRead(int address){
    UCB0I2CSA = RM25_ADDRESS;                            // Sets slave address
    MSB = address & 0xFF;                            // Sets most significant byte
    LSB = address >> 8;                             // Sets least significant byte
    RX_Byte_Ctr = 1;                                // Sets the amount of bytes to be sent
    TX_Byte_Ctr = 2;
    i = 0;
    dataBuff[0] = MSB;
    dataBuff[1] = LSB;
  // Enters Low-Power mode and enables global interrupt
    UCB0CTL1 |= UCTR                                // Enables TX Mode
             |  UCTXSTT;                            // Sends start condition
     __bis_SR_register(LPM0_bits | GIE);
    UCB0TBCNT  = 0x01;
    __delay_cycles(8);
    UCB0CTL1 &= ~UCTR;                              // Enters RX Mode
    UCB0CTL1 |= UCTXSTT;                            // Sends start condition
    __bis_SR_register(LPM0_bits | GIE);             // Enters Low-Power mode and enables global interrupt
}





void main(void){
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;

    i2cInit();
    txData[3] = 0x49;

    singleWrite(0xAAAA, txData);
    singleRead(0xAAAA);

}



#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void) {
    switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG)) {
        case USCI_I2C_UCNACKIFG:                    // NACK Interrupt
            UCB0CTL1 |= UCTXSTT;                    // I2C start condition
        break;

        case USCI_I2C_UCRXIFG0:                     // I2C RX Interrupt
            if(RX_Byte_Ctr > 1){                     // Checks if there is more data to be received
                dataBuff[v] = UCB0RXBUF;               // Loads the data array
                v++;                            // Increments the pointer
                RX_Byte_Ctr--;                      // Decrement TX byte counter
            } else if (RX_Byte_Ctr == 1){
                dataBuff[v] = UCB0RXBUF;               // Loads the data array
                v++;                            // Increments the pointer
                RX_Byte_Ctr--;
                v=0;
                    __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
            }

        break;

        case USCI_I2C_UCTXIFG0:                     // I2C TX Interrupt
            if(TX_Byte_Ctr > 1){
                UCB0TXBUF = dataBuff[i];                       // Example send
                i++;
                TX_Byte_Ctr--;
            } else if (TX_Byte_Ctr == 1){
                UCB0TXBUF = dataBuff[i];                       // Example send
                TX_Byte_Ctr--;
                i = 0;
                __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
        }
            break;
}   }

