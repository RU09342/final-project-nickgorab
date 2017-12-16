/*
 *            main.c
 *
 *   Created on:  November 12, 2017
 *  Last Edited:  November 12, 2017
 *       Author:  Nick Gorab
 *        Board:  MSP430FR5994
 */

#include "system.h"


/************************\
*                        *
*    Code Definitions    *
*                        *
\************************/

#define MAX_OUT 100         // Maximum PWM output
#define MIN_OUT 2           // Minimum PWM output
#define dt      0.000125    // Sampling time of ~100Hz

int   roll,                 // Roll value
      pitch,                // Pitch Value
      height,               // Height value
      setRoll,              // Set roll value
      setPitch,             // Set pitch value
      setHeight,            // Set height value
      rKp,                  // Roll proportional constant
      rKi,                  // Roll integral constant
      rKd,                  // Roll derivative constant
      pKp,                  // Pitch proportional constant
      pKi,                  // Pitch integral constant
      pKd,                  // Pitch derivative constant
      hKp,                  // Height proportional constant
      hKi,                  // Height integral constant
      hKd,                  // Height derivative constant
      battStat,             // Battery health variable
      kill;                 // Kill switch variable
float rP_error,             // Roll proportional error
      rI_error,             // Roll integral error
      rD_error,             // Roll derivative error
      pP_error,             // Pitch proportional error
      pI_error,             // Pitch integral error
      pD_error,             // Pitch derivative error
      hP_error,             // Height proportonal error
      hI_error,             // Height integral error
      hD_error,             // Height derivative error
      rPastError,           // Roll past error for integral control
      pPastError,           // Pitch past error for integral control
      hPastError,           // Height past error for integral control
      rOutput,              // Roll PID output
      pOutput,              // Pitch PID output
      hOutput,              // Height PID output
      p1pwm,                // Pitch control 1 PWM
      p2pwm,                // Pitch control 2 PWM
      r1pwm,                // Roll control 1 PWM
      r2pwm;                // Roll control 2 PWM
char  controlData[4];       // Array for storing accelerometer data


/***********************\
 *                     *
 * Batt Check Function *
 *                     *
\***********************/

void battCheck(void){

    ADC12CTL0 |= ADC12ENC       // Enables ADC
              |  ADC12SC;       // Takes a reading

    if(ADC12MEM0 < 3723) {      // If battry voltage < 3.0 V
        battStat = 1;           // Raises low batt flag
        if (ADC12MEM0 < 3455) { // If battery voltage is < 2.7 V
            battStat = 2;
    }   } else {                // If battery voltage is nominal
        battStat = 0;           // Continue normal operation
    }

    switch(battStat){
        case 0:                 // Battery is good
            TA0CTL |=  MC_0     // Stops timer
                   |   TACLR;   // Clears timer
            P1SEL0 &= ~BIT0;    // Sets P1.0 to GPIO output
            P1OUT   =  BIT0;    // Turns the LED on
        break;
        case 1:                 // Low battery state
            P1SEL0  = BIT0;     // Sets P1.0 to TA0.1 control
            TA0CTL |= MC_2;     // Starts the clock in continous mode
        break;
        case 2:                 // Critical battery state
            TA0CTL |=  MC_0     // Stops timer
                   |   TACLR;   // Clears timer
            P1SEL0 &= ~BIT0;    // Sets P1.0 to GPIO output
            P1OUT  &= ~BIT0;    // Turns off the LEDs
            kill = 1;           // Sets the kill flag high 
}   }


void definer(void){
    rKp = 0;        // Initialized constants to zero
    rKi = 0;        // Initialized constants to zero
    rKd = 0;        // Initialized constants to zero
    pKp = 0;        // Initialized constants to zero
    pKi = 0;        // Initialized constants to zero
    pKd = 0;        // Initialized constants to zero
    hKp = 0;        // Initialized constants to zero
    hKi = 0;        // Initialized constants to zero
    hKd = 0;        // Initialized constants to zero


}
/***********************\
*                       *
*      PID Control      *
*                       *
\***********************/

void pidControl(int status){

    controlData = getAccel();                       // Updates current attitude data
    roll        = controlData[0] + controlData[1];  // Takes the roll data from the accelerometer
    pitch       = controlData[2] + controlData[3];  // Takes the pitch data from the accelerometer

    rP_error = roll   - setRoll;                    // Calculates roll proportional error
    pP_error = pitch  - setPitch;                   // Calculates pitch proportional error
    hP_error = height - setHeight;                  // Calculates height proportional error
        
    rD_error = (rP_error - rPastError)/dt;          // Calculates roll derivative error
    pD_error = (pP_error - pPastError)/dt;          // Calculates pitch derivative error
    hD_error = (hP_error - hPastError)/dt;          // Calculates height derivative error
        
    rI_error = rI_error + (rP_error*dt);            // Calculates sum value for integral control
    pI_error = pI_error + (pP_error*dt);            // Calculates sum value for integral control
    hI_error = hI_error + (hP_error*dt);            // Calculates sum value for integral control
        
    rPastError = rP_error;                          // Updates past error for
    pPastError = pP_error;                          // Updates past error for
    hPastError = hP_error;                          // Updates past error for
        
    hOutput = (hKp*hP_error)                        // Adds proportional error
            + (hKd*hD_error)                        // Adds derivative error
            + (hKi*hI_error);                       // Adds integral error
        
    pOutput = (pKp*pP_error)                        // Adds proportional error
            + (pKd*pD_error)                        // Adds derivative error
            + (pKi*pI_error);                       // Adds integral error
        
    rOutput = (rKp*rP_error)                        // Adds proportional error
            + (rKd*rD_error)                        // Adds derivative error
            + (rKi*rI_error);                       // Adds integral error
        
    if(hOutput > MAX_OUT){                          // If height output is greater than max output
       hOutput = MAX_OUT;                           // Clamps output to max value
    } else if(hOutput < MIN_OUT){                   // If height output is less than minimum output
        hOutput = MIN_OUT;                          // Clamps output to minimum value
    }       
        
    if(pOutput > MAX_OUT){                          // If pitch output is greater than max output
        pOutput = MAX_OUT;                          // Clamps output to max value
    } else if(pOutput < MIN_OUT){                   // If pitch output is less than minimum output
        pOutput = MIN_OUT;                          // Clamps output to minimum value
    }       
        
    if(rOutput > MAX_OUT){                          // If roll output is greater than max output
        rOutput = MAX_OUT;                          // Clamps output to max value
    } else if(rOutput < MIN_OUT){                   // If roll output is less than minimum output
        rOutput = MIN_OUT;                          // Clamps output to minimum value
    }       
        
    if(rOutput > 0){                                // If there is a positive roll
      r1pwm = hOutput + (rOutput/2);                // Adds half the PWM value to the height control
      r2pwm = hOutput - (rOutput/2);                // Subtracts half PWM value from the height control
    } else {        
      r1pwm = hOutput - (rOutput/2);                // Subtracts half PWM value from the height control
      r2pwm = hOutput + (rOutput/2);                // Adds half the PWM value form the height control
    }       
        
    if(pOutput > 0){                                // If there is a positive pitch
      p1pwm = hOutput + (pOutput/2);                // Adds half the PWM value to the height control
      p2pwm = hOutput - (pOutput/2);                // Subtracts half PWM value from the height control
    } else {        
      p1pwm = hOutput - (pOutput/2);                // Subtracts half PWM value from the height control
      p2pwm = hOutput + (pOutput/2);                // Adds half the PWM value form the height control
}   }



/***********************\
 *                     *
 *    Main Function    *
 *                     *
\***********************/

void main(void){

    WDTCTL = WDTPW | WDTHOLD;         // Disables Watchdog Timer
    PM5CTL0 &= ~LOCKLPM5;             // Disables high-impedance mode
    adcInit();                        // Initializes the ADC
    ledInit();                        // Initializes the LEDs
    timerAInit();                     // Initializes TIMER_A module
    timerBInit();                     // Initializes TIMER_B module
    i2cInit();                        // Initializes I2C Communication
    definer();                        // Sets all values to zero
    mpu6050Init();                    // Sends command to MPU6050 to turn it on
    rPastError = 0;                   // Sets the past error equal to zero
    pPastError = 0;                   // Sets the past error equal to zero
    hPastError = 0;                   // Sets the past error equal to zero

    while(~kill){                     // Operates as long as kill flag isn't raised

        battCheck();                  // Checks the health status of the battery
        pidControl(battStat);         // Begins the PID control sequence
    }

    if(kill){                         // If the kill flag is raised
        P1DIR &= ~BIT4;               // Disables Motor 1
        P1DIR &= ~BIT5;               // Disables Motor 2
        P4DIR &= ~BIT4;               // Disables Motor 3
        P4DIR &= ~BIT5;               // Disables Motor 4
        __bis_SR_register(LPM0_bits); // Enters Low-Power mode and enables global interrupt
}   }



/************************\
 *                      *
 *    UART Interrupt    *
 *                      *
\************************/

#pragma vector=USCI_A0_VECTOR           // UART RX Interrupt used for on-the-fly PID adjusting
__interrupt void USCI_A0_ISR(void) {    // Interrupt function decleration

    int byte = 0;                       // Resets the byte coutner to zero

    switch(byte) {                      // Switch statement for byte position
        case 0 :                        // First byte set
            rKp = UCA0RXBUF;            // Sets the roll proportional constant
        break;
        case 1 :                        // Second byte sent
            rKi = UCA0RXBUF;            // Sets the roll integral constant
            break;
        case 2 :                        // Third byte sent
            rKd = UCA0RXBUF;            // Sets the roll derivative constant
            break;
        case 3 :                        // Fourth byte sent
            pKp = UCA0RXBUF;            // Sets the pitch proportional constant
            break;
        case 4  :                       // Fifth byte sent
            pKi = UCA0RXBUF;            // Sets the pitch integral constant
            break;
        case 5  :                       // Sixth byte sent
            pKd = UCA0RXBUF;            // Sets the pitch derivative constant
            break;
        case 6  :                       // Seventh byte sent
            hKp = UCA0RXBUF;            // Sets the height proportional constant
            break;
        case 7  :                       // Eighth byte sent
            hKi = UCA0RXBUF;            // Sets the height integral constant
            break;
        case 8  :                       // Ninth byte sent
            hKd = UCA0RXBUF;            // Sets the height derivative constant
            break;
        default:                        // Default case
        break;                          // Do nothing
    }
    byte++;                             // Increments byte counter
    if(byte == 8){                      // If transmission is finished
        byte = 0;                       // Reset byte counter and wait for next transmission
}   }


