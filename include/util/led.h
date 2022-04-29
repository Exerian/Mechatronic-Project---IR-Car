#include <Arduino.h>

#ifndef MAST2010_UTIL_LED_H
#define MAST2010_UTIL_LED_H

namespace LED {
enum Colours:uint32_t {
	White = 0xFFFFFF,
	Silver = 0xC0C0C0,
	Grey = 0x808080,
	Blank = 0x000000, // Black
	Red = 0xFF0000,
	Maroon = 0x800000,
	Yellow = 0xFFFF00,
	Olive = 0x808000,
	Lime = 0x00FF00,
	Green = 0x00FF00,
	Aqua = 0x00FFFF,
	Teal = 0x008080,
	Blue = 0x0000FF,
	Navy = 0x000080,
	Fuchsia = 0xFF00FF,
	Purple = 0x800080,
	Orange = 0xFF8000,
	PrussianBlue = 0x232c3f,
};

// Sets the colour of a RGB led given a hexadecimal 24-bit value (HTML colour).
void setLedColour(const uint32_t hexColour);
void setLedcolour(volatile uint8_t &port, const uint8_t pins, const uint32_t hexColour);
// Sets the colour of a RGB led given an 8-bit value for each RGB colour.
void setLedColour(const uint8_t red, const uint8_t green, const uint8_t blue);

uint32_t rgbToHex(const uint8_t red, const uint8_t green, const uint8_t blue);

}
#endif