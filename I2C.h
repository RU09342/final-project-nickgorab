/*
 *            I2C.h
 *
 *   Created on:  November 7, 2017
 *  Last Edited:  November 13, 2017
 *       Author:  Nick Gorab
 *        Board:  MSP430F5994
 */


#ifndef I2C_H_
#define I2C_H_

#include <msp430.h>

void i2cSingleWrite(char address, char TX_Data);                   // Writes single byte of data
void i2cSingleRead(char address, char *RX_Data);                  // Reads single byte of data
void i2cMultipleWrite(char address, int size, char *TX_Data);    // Writes multiple bytes of data
void i2cMultipleRead(char address, int size, char *RX_Data);    // Reads multiple bytes of data


#endif /* I2C_H_ */
