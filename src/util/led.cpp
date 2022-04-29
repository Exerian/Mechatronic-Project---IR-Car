#include "util/led.h"

namespace LED {

void setLedColour(const led_t led, const uint8_t red, const uint8_t green, const uint8_t blue)
{
    analogWrite(led.redPin, red);
    analogWrite(led.greenPin, green);
    analogWrite(led.bluePin, blue);
};

void setLedColour(const led_t led, const uint32_t hexColour)
{
    setLedColour(led, hexColour >> 16, hexColour >> 8, hexColour);
};

uint32_t rgbToHex(const uint8_t red, const uint8_t green, const uint8_t blue)
{
    return (red << 16) | (green << 8) | blue;
}

}