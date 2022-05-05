#include <Arduino.h>
#include "util/utility.h"

#ifndef MAST2010_MOTOR_H
#define MAST2010_MOTOR_H

#define ICR1_TOP 499 // f_clk / (2 * 1 * f_ocr) - 1 = ICR_TOP; @499 -> f_ocr = 16kHz
#define LOWEST_OCR 50

namespace G16 {

enum class MotorDutyCycle:uint8_t
{
    STOP    = 0,
    SLOW    = 10,
    MEDIUM  = 50,
    FAST    = 80,
    TOP     = 100,
    REVERSE = SLOW
};

/**
 * @brief DC Motor driver for two DC motors for Arduino Uno.
 * 
 *  Requires the enable pins to be set on pin 9 and 10, and each set
 *  of forward and reverse pin to be set on the same port (i.e. pin 2-7, 8-13, or A0-A5).
 * 
 */
class Motor
{
private:
    const PortPointer_IO8 leftPort, rightPort;
    OCRPointer_MEM16 powerLeft = &OCR1A, powerRight = &OCR1B;
    const uint8_t forwardLeftMask, reverseLeftMask, forwardRightMask, reverseRightMask;
public:
    /**
     * @brief Construct a new Motor object for two DC motors. \n
     * 
     * PORTx:
     *  A, Pin A0 to A5;
     *  B, Pin 8 to 13;
     *  D, Pin 2 to 7.
     * 
     * @param forwardLeft   Must be set on the same PORTx as 'reverseLeft'.
     * @param reverseLeft   Must be set on the same PORTx as 'forwardLeft'.
     * @param forwardRight  Must be set on the same PORTx as 'reverseRight'.
     * @param reverseRight  Must be set on the same PORTx as 'forwardRight'.
     * @param enableLeft    Must be set on pin 9 or 10, PORTB.
     * @param enableRight   Must be set on pin 9 or 10, PORTB.
     */
    Motor(uint8_t forwardLeft, uint8_t reverseLeft, uint8_t forwardRight, uint8_t reverseRight, uint8_t enableLeft=9, uint8_t enableRight=10);
    ~Motor();

    // Stops the motors.
    void stop();
    // The motors will both go in forward direction.
    void forward();
    // The motors will both go in reverse direction.
    void reverse();
    // The motors will go in opposite directions; left in reverse and right forward.
    void turnLeft();
    // The motors will go in opposite directions; left forward and right in reverse.
    void turnRight();
    // Write the duty cycle of the motors, an interval of [0,100].
    void write(uint8_t leftDutyCycle, uint8_t rightDutyCycle);
    // Write the duty cycle of the motors.
    void write(MotorDutyCycle left, MotorDutyCycle right);
    // Write the speed of the motors.
    // void write(uint16_t leftValue, uint16_t rightValue);
};

}
#endif