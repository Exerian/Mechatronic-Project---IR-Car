#include <Arduino.h>
#include "util/utility.h"

#ifndef MAST2010_CONTROLLERS_MOTOR_H
#define MAST2010_CONTROLLERS_MOTOR_H

#define ICR1_TOP 499 // (f_clk / (2 * 1 * f_ocr)) - 1 = ICR1_TOP; @499 -> f_ocr = 16kHz
#define LOWEST_OCR 50 // The standard Arduino DC motors struggle to run below this value.

namespace G16 {

enum class MotorDutyCycle:uint8_t
{
    STOP    = 0,
    SLOW    = 10,
    MEDSLO  = 25,
    MEDIUM  = 50,
    MEDFAS  = 75,
    FAST    = 90,
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
    const IO8_Pointer leftPort, rightPort;
    MEM16_Pointer leftOCR = &OCR1A, rightOCR = &OCR1B;
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

    /**
     * @brief Initialise timers.
     * 
     */
    void initialise();

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
    
    /**
     * @brief Write a value in the interval [0, TOP] to the motors, where TOP is the maximum value set in ICR1.
     * 
     *  For any values larger than TOP, TOP is written.
     * 
     * @param left Value to the left motor.
     * @param right Value to the right motor.
     */
    void write(uint16_t left, uint16_t right);

    /**
     * @brief Write the duty cycle of the motors, a number in the interval [0,100].
     * 
     * @param leftDutyCycle Duty cycle of the left motor.
     * @param rightDutyCycle Duty cycle of the right motor.
     */
    void writeDutyCycle(uint8_t leftDutyCycle, uint8_t rightDutyCycle);
    
    /**
     * @brief Write the duty cycle of the motors.
     * 
     * @param left Duty cycle of the left motor.
     * @param right Duty cycle of the right motor.
     */
    void writeDutyCycle(MotorDutyCycle left, MotorDutyCycle right);

    /**
     * @brief Get the PWM value of the left motor.
     * 
     * @return uint16_t
     */
    uint16_t getLeftPWM();

    /**
     * @brief Get the PWM value of the right motor.
     * 
     * @return uint16_t 
     */
    uint16_t getRightPWM();
};

}
#endif