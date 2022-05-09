#include <Arduino.h>

#ifndef MAST2010_UTIL_UTILITY_H
#define MAST2010_UTIL_UTILITY_H

namespace G16 {

typedef decltype(&PORTB) IO8_Pointer; // Pointer to 8-bit registers.
typedef decltype(&OCR1A) MEM16_Pointer; // Pointer to 16-bit registers.

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