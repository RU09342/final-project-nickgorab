/*
 *            initializer.h
 *
 *   Created on:  November 12, 2017
 *  Last Edited:  November 12, 2017
 *       Author:  Nick Gorab
 *        Board:  MSP430FR5994
 */

/***********************************************************************************************************\
*                                                                                                           *
*  The purpose of this file is to enable all of the relevant registers used in this code to avoid clutter.  *
*                                                                                                           *
\***********************************************************************************************************/

#ifndef INITIALIZER_H_
#define INITIALIZER_H_

#include <msp430.h>

void adcInit();
void ledInit();
void timerInit();
void i2cInit();
void dcoInit();
void vcnl4200Init();
void lsm6dslInit();
void uartInit();

#endif /* INITIALIZER_H_ */
