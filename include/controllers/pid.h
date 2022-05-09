#include <Arduino.h>

#ifndef MAST2010_CONTROLLERS_PID_H
#define MAST2010_CONTROLLERS_PID_H

namespace G16
{

class PID
{
private:
    int8_t Kp, Ki, Kd;
    int8_t error = 0;
    int8_t value = 0;
    int8_t integral = 0;
    uint32_t lastUpdate = 0UL;
protected:

public:

    /**
     * @brief Construct a new PID controller.
     * 
     * Proportional-Integral-Derivative controller.
     * 
     * The coefficients can only be positive numbers in the interval [0, 127],
     *  and must be tuned to a functional value; tune Kp first.
     * 
     * @param Kp The coefficient for the proportional term.
     * @param Ki The coefficient for the integral term.
     * @param Kd The coefficient for the derivative term.
     */
    PID(int8_t Kp=1, int8_t Ki=0, int8_t Kd=0);
    ~PID();

    void update(int8_t error);
    int8_t getPID();
    uint32_t &getLastUpdate();
};

} // namespace G16

#endif