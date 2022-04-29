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

typedef struct led {
	const uint8_t redPin;
	const uint8_t greenPin;
	const uint8_t bluePin;
	uint32_t previousChange;
}led_t;

void setLedColour(const led_t led, const uint8_t red, const uint8_t green, const uint8_t blue);
void setLedColour(const led_t led, const uint32_t hexColour);

uint32_t rgbToHex(const uint8_t red, const uint8_t green, const uint8_t blue);

}
#endif