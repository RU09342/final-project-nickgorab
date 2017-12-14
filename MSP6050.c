#include <msp430.h>
#define MPU6050_ADDRESS 0x68;
int     RX_Byte_Ctr;        // Coutner to make sure all of the information is received
int     TX_Byte_Ctr;        // Counter to make sure all of the information is sent
int i  =0;
char    Rx_Data[6],          // Creates an array to store height data
        *pointer;           // Creates a pointer to access the array
char    Tx_Data[];         // Creates an array to store the sent data



void i2cInit(void){
    P1SEL0    |= BIT6       // SDA line for I2C using UCB0
              |  BIT7;      // SCL line for I2C using UCB0
    UCB0CTLW0 |= UCSWRST;   // Enters reset state, USCI stops operation
    UCB0TBCNT = 0x05;       // Expecting to receive 2 bytes of data
    UCB0CTLW1 |= UCASTP_2;  // Sends stop bit when UCTBCNT is reached
    UCB0CTLW0 |= UCMST      // Master Mode
              |  UCMODE_3   // I2C Mode
              |  UCSSEL__SMCLK;    // Synchronus Mode
    UCB0BRW    = 0x000A;    // SMCLK/10
    UCB0CTLW0 &= ~UCSWRST;  // Exits reset mode, USCI enters operation
    UCB0IE    |= UCTXIE0    // Data received interrupt enable
              |  UCRXIE0    // Data ready to transmit interrupt enable
              |  UCNACKIE;  // NACK interrupt enable
}



void mpuInit(void){
    TX_Byte_Ctr = 2;
    Tx_Data[0] = 0x6B;
    Tx_Data[1] = 0x00;
    UCB0I2CSA = MPU6050_ADDRESS;

    UCB0CTL1 |= UCTR                        // Enables TX Mode
             |  UCTXSTT;                    // Sends start condition
    __bis_SR_register(LPM0_bits | GIE);     // Enters Low-Power mode and enables global interrupt
    i = 0;
    RX_Byte_Ctr = 0;
    TX_Byte_Ctr = 0;
}



void getHeight(void){

    pointer = Rx_Data;                       // Sets the pointer to the height array
    RX_Byte_Ctr = 6;
    TX_Byte_Ctr = 1;
    Tx_Data[0] = 0x3B;
    UCB0I2CSA = MPU6050_ADDRESS;           // Sets slave address


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


void main(void){
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;

    i2cInit();
    mpuInit();

    while(1){
        __delay_cycles(100);
        getHeight();
}   }



#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void) {
    switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG)) {
        case USCI_I2C_UCNACKIFG:                    // NACK Interrupt
            UCB0CTL1 |= UCTXSTT;                    // I2C start condition
        break;

        case USCI_I2C_UCRXIFG0:                     // I2C RX Interrupt
            while(RX_Byte_Ctr){                     // Checks if there is more data to be received
                *pointer = UCB0RXBUF;               // Loads the data array
                pointer++;                          // Increments the pointer
                RX_Byte_Ctr--;                      // Decrement TX byte counter
            }
                __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
        break;

        case USCI_I2C_UCTXIFG0:                     // I2C TX Interrupt
            while(TX_Byte_Ctr){
                UCB0TXBUF = Tx_Data[i];                       // Example send
                i++;
                TX_Byte_Ctr--;
            }
                __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
        break;

}   }

