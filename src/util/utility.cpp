#include "util/utility.h"


namespace G16{
    
uint16_t mapFromPercent(uint8_t x, uint16_t outputMin, uint16_t outputMax)
{
    return x * (outputMax - outputMin) / 100 + outputMin;
};

} // namespace G16


