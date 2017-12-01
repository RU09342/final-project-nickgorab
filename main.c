/*
 *            battCheck.c
 *
 *   Created on:  November 12, 2017
 *  Last Edited:  November 12, 2017
 *       Author:  Nick Gorab
 *        Board:  MSP430FR5994
 */

#include "system.h"

#define MAX_OUT

int rKp,            // Roll proportional constant
    rKi,            // Roll integral constant
    rKd,            // Roll derivative constant
    pKp,            // Pitch proportional constant
    pKi,            // Pitch integral constant
    pKd,            // Pitch derivative constant
    hKp,            // Height proportional constant
    hKi,            // Height integral constant
    hKd;            // Height derivative constant
int battStat = 0;   // 



/***********************\
 *                     *
 * Batt Check Function *
 *                     *
\***********************/

void battCheck(void){

    ADC12CTL0 |= ADC12ENC       // Enables ADC
              |  ADC12SC;       // Takes a reading

    if(ADC12MEM0 < 3105) {      // If battry voltage < 2.5 V
        battStat = 1;           // Raises low batt flag
    } else {
        battStat = 0;           // Continue normal operation
    }

    switch(battStat){
        case 0:
            P1OUT |=  BIT1; // Turns on the green LED
            P1OUT &= ~BIT0; // Turns off the red LED
        break;
        case 1:
            P1OUT |=  BIT0; // Turns on the red LED
            P1OUT &= ~BIT1; // Turns off the green LED
        break;
}   }



/***********************\
*                       *
*      PID Control      *
*                       *
\***********************/

void pwmControl(int setPoint){
    adcMem = ADC12MEM0;                     // Sets variable equal to ADC value for debugging

    if(tempChange){                         // If the ADC value needs to be adjusted
       P_error = adcMem - (setPoint+27.3);  // Sets temp to value of 1 less than it actually is
    } else {                                // If the value does not need to be adjusted
        P_error = adcMem - setPoint;        // Sets temp to actual value
    }

    D_error = (P_error - pastError)/dt;     // Calculated derivative error
    I_Error = I_Error + (P_error*dt);       // Calculated sum value for integral control
    pastError = P_error;                    // Updates past error for
    output  = (Kp*P_error)                  // Adds proportional error
         // + (Kd*D_error)                  // Adds derivative error
            + (Ki*I_Error);                 // Adds integral error
        
    if(output > MAX_OUT){                   // If output is greater than max output
        output = MAX_OUT;                   // Clamps output to max value
    } else if(output < MIN_OUT){            // If output is less than minimum output
        output = MIN_OUT;                   // Clamps output to minimum value
    }       
        
    TA0CCR1 = output;                       // Established PWM value based off of PI Controller
}



/***********************\
 *                     *
 *    Main Function    *
 *                     *
\***********************/

void main(void){
    WDTCTL = WDTPW | WDTHOLD;   // Disables Watchdog Timer
    PM5CTL0 &= ~LOCKLPM5;       // Disables high-impedance mode
    adcInit();                  // Initializes the ADC
    ledInit();                  // Initializes the LEDs
    timerInit();

    while(1){
        battCheck();
    }
}



/************************\
 *                      *
 *    UART Interrupt    *
 *                      *
\************************/

#pragma vector=USCI_A0_VECTOR           // UART RX Interrupt used for on-the-fly PID adjusting
__interrupt void USCI_A0_ISR(void) {    // Interrupt function decleration

    int byte = 0;
    
    switch(bit) {                       // Switch statement for byte position
        case 0 :                        // First byte set
            rKp = UCA0RXBUF             // Sets the roll proportional constant
        break;
        case 1 :                        // Second byte sent
            rKi = UCA0RXBUF             // Sets the roll integral constant
            break;
        case 2 :                        // Third byte sent
            rKd = UCA0RXBUF             // Sets the roll derivative constant
            break;
        case 3 :                        // Fourth byte sent
            pKp = UCA0RXBUF             // Sets the pitch proportional constant
            break;
        case 4  :                       // Fifth byte sent
            pKi = UCA0RXBUF             // Sets the pitch integral constant
            break;
        case 5  :                       // Sixth byte sent
            pKd = UCA0RXBUF             // Sets the pitch derivative constant
            break;
        case 6  :                       // Seventh byte sent
            hKp = UCA0RXBUF             // Sets the height proportional constant
            break;
        case 7  :                       // Eighth byte sent
            hKi = UCA0RXBUF             // Sets the height integral constant
            break; 
        case 8  :                       // Ninth byte sent
            hKd = UCA0RXBUF             // Sets the height derivative constant
            break;
        default:                        // Default case
        break;                          // Do nothing
    }
    byte++;                             // Increments byte counter
    if(byte == 8){                      // If transmission is finished
        byte = 0;                       // Reset byte counter and wait for next transmission
}   }

