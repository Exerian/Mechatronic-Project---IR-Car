#include "util/led.h"

namespace LED {

void setLedColour(volatile uint8_t &port, const uint8_t pins, const uint8_t red, const uint8_t green, const uint8_t blue) 
{
    switch (port)
    {
    case 5:
        break;
    case 10:
        break;
    case 20:
        break;
    default:
        break;
    }
};

void setLedcolour(volatile uint8_t &port, const uint8_t pins, const uint32_t hexColour)
{
    setLedColour(port, pins, hexColour >> 16, hexColour >> 8, hexColour);
};

uint32_t rgbToHex(const uint8_t red, const uint8_t green, const uint8_t blue)
{
    return red << 16 | green << 8 | blue;
}

}