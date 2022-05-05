#include <wiring_private.h>
#include "util/led.h"

namespace G16 {

LED::LED(uint8_t redPin, uint8_t greenPin, uint8_t bluePin)
{
};

LED::~LED(){};

void LED::setColour(const uint8_t redValue, const uint8_t greenValue, const uint8_t blueValue)
{
    this->redColour = redValue;
    this->greenColour = greenValue;
    this->blueColour = blueValue;
    this->update();
};
void LED::setColour(const Colours &hexValue)
{
    this->redColour = hexValue >> 16;
    this->greenColour = hexValue >> 8;
    this->blueColour = hexValue;
    this->update();
};
void LED::clear()
{
    analogWrite(this->redPin, 0);
    analogWrite(this->greenPin, 0);
    analogWrite(this->bluePin, 0);
};
void LED::blink(uint32_t)
{

};
void LED::update()
{
    this->lastUpdate = millis();
    analogWrite(this->redPin, this->redColour);
    analogWrite(this->greenPin, this->greenColour);
    analogWrite(this->bluePin, this->blueColour);
};

uint32_t LED::getLastUpdate()
{
    return this->lastUpdate;
};

void setLedColour(const led_t &led, const uint8_t red, const uint8_t green, const uint8_t blue)
{
    analogWrite(led.redPin, red);
    analogWrite(led.greenPin, green);
    analogWrite(led.bluePin, blue);
};

void setLedColour(const led_t &led, const Colours &hexColour)
{
    setLedColour(led, hexColour >> 16, hexColour >> 8, hexColour);
};

uint32_t rgbToHex(const uint8_t red, const uint8_t green, const uint8_t blue)
{
    return (red << 16) | (green << 8) | blue;
};

}