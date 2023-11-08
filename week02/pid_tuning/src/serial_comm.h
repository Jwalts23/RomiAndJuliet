/*
 * For managing input over the serial stream (USB for the 32U4).
 * 
 * Be sure to set your Serial Monitor to append a NEWLINE character to each line.
 * (See the dropdown near the bottom of the Serial Monitor.)
 * 
 * Format is a single letter, followed by a number. The number represents either
 * a target speed or a PID gain; see ParseSerialInput() below for what is what.
 * 
 * Examples:
 * 
 * Sending  R10    will set the target speed of the right wheel to 10
 * Sending  I1.2   will set the Ki term to 1.2
 * 
 * IT DOES NO ERROR CHECKING! 
 * 
 * You won't break anything if the format is wrong, but neither will it work.
 */

#ifndef __SERIAL_COMM_H
#define __SERIAL_COMM_H

#include <Arduino.h>

bool CheckSerialInput(void);
void ParseSerialInput(void);

#endif
