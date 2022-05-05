#include <Arduino.h>

#ifndef MAST2010_UTIL_UTILITY_H
#define MAST2010_UTIL_UTILITY_H

namespace G16 {

typedef decltype(&DDRB) DDRPointer_IO8; // Pointer to Data Direction Register.
typedef decltype(&PORTB) PortPointer_IO8; // Pointer to Port, Pin Output Register.
typedef decltype(&OCR0A) OCRPointer_IO8; // Pointer to Output Compare Register, 8-bit.
typedef decltype(&OCR1A) OCRPointer_MEM16; // Pointer to Output Compare Register, 16-bit.

/**
 * @brief Map a value in the interval [0,100] to [outputMin, outputMax].
 * 
 * @param x             The value to be mapped, 0 <= x <= 100.
 * @param outputMin     The minimum value of the output.
 * @param outputMax     The maximum value of the output.
 * @return uint8_t 
 */
uint16_t mapFromPercent(uint8_t x, uint16_t outputMin, uint16_t outputMax);

}

#endif