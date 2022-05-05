#include <Arduino.h>
#include "util/utility.h"

#ifndef MAST2010_UTIL_LED_H
#define MAST2010_UTIL_LED_H

namespace G16 {
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
};

class LED
{
private:
	uint8_t redPin, greenPin, bluePin;
	uint8_t redColour, greenColour, blueColour;
	uint32_t lastUpdate = 0UL;
public:
	// Set the pin number for each RGB colour of the LED.
	LED(uint8_t redPin, uint8_t greenPin, uint8_t bluePin);
	~LED();

	// Set the LED colours to a RGB 8-bit value from 0 to 255.
	void setColour(const uint8_t redValue, const uint8_t greenValue, const uint8_t blueValue);
	// Set the LED colours to a HTML 24-bit hexadecimal code.
	void setColour(const Colours &hexValue);
	// Clear the LED.
	void clear();
	// N/A
	void blink(uint32_t interval = 2000UL);
	// N/A
	void blink(uint32_t &currentTiming, uint32_t interval = 2000UL);
	// Update the LED.
	void update();
	// Returns the time of the last update.
	uint32_t getLastUpdate();
};


typedef struct led {
	const uint8_t redPin;
	const uint8_t greenPin;
	const uint8_t bluePin;
	uint32_t previousChange;
}led_t;

void setLedColour(const led_t &led, const uint8_t red, const uint8_t green, const uint8_t blue);
void setLedColour(const led_t &led, const Colours &hexColour);

uint32_t rgbToHex(const uint8_t red, const uint8_t green, const uint8_t blue);

}
#endif