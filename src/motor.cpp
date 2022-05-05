#include "motor.h"

namespace G16 {

Motor::Motor(uint8_t forwardLeft, uint8_t reverseLeft, uint8_t forwardRight, uint8_t reverseRight, uint8_t enableLeft, uint8_t enableRight)
    : leftPort(portOutputRegister(digitalPinToPort(forwardLeft)))
    , rightPort(portOutputRegister(digitalPinToPort(forwardRight)))
    , forwardLeftMask(digitalPinToBitMask(forwardLeft))
    , reverseLeftMask(digitalPinToBitMask(reverseLeft))
    , forwardRightMask(digitalPinToBitMask(forwardRight))
    , reverseRightMask(digitalPinToBitMask(reverseRight))
    //, powerLeft((enableLeft == 9) ? &OCR1A : (enableLeft == 10) ? &OCR1B : 0)
{
    switch (enableLeft ^ 0x10)
    {
    case 0x00:
        enableRight = 9;
        this->powerRight = &OCR1A;
        this->powerLeft = &OCR1B;
        break;
    case 0x03:
        enableRight = 10;
        this->powerRight = &OCR1B;
        break;
    default:
        enableLeft = 9;
        enableRight = 10;
        this->powerLeft = &OCR1A;
        this->powerRight = &OCR1B;
        break;
    }

    // Set the enable pins to output in the Data Direction Register.
    DDRB |= digitalPinToBitMask(enableLeft) | digitalPinToBitMask(enableRight);

    // Get Data Direction Register for each pin set, and set the direction pins to output.
    DDRPointer_IO8 DDRx = portModeRegister(digitalPinToPort(forwardLeft));
    *DDRx |= forwardLeftMask | reverseLeftMask;
    DDRx = portModeRegister(digitalPinToPort(forwardRight));
    *DDRx |= forwardRightMask | reverseRightMask;

    // Set Timer/Counter Control Register for PWM.
    TCCR1A = (1 << COM1A1) | (1 << COM1B1);
    TCCR1B = 0;
    TCNT1 = 0x00;
    ICR1 = ICR1_TOP;
    *this->powerLeft = 0x00;
    *this->powerRight = 0x00;
    TCCR1B = (1 << WGM13) | (1 << CS10); // Timer will start running when a CSxx bit is set.
};

Motor::~Motor(){};

void Motor::stop()
{
    *this->powerLeft = 0x00;
    *this->powerRight = 0x00;
};

void Motor::forward()
{
    // Set the forward bit high, and the reverse bit low for both motors.
    *this->leftPort = (*this->leftPort & ~reverseLeftMask) | forwardLeftMask;
    *this->rightPort = (*this->rightPort & ~reverseRightMask) | forwardRightMask;
};

void Motor::reverse()
{
    // Set the reverse bit high, and the forward bit low for both motors.
    *this->leftPort = (*this->leftPort & ~forwardLeftMask) | reverseLeftMask;
    *this->rightPort = (*this->rightPort & ~forwardRightMask) | reverseRightMask;
};

void Motor::turnLeft()
{
    // Set the reverse bit to high and low, and the forward bit to low and high respectively for left and right motor.
    *this->leftPort = (*this->leftPort & ~forwardLeftMask) | reverseLeftMask;
    *this->rightPort = (*this->rightPort & ~reverseRightMask) | forwardRightMask;
};

void Motor::turnRight()
{
    // Set the forward bit to high and low, and the reverse bit to low and high respectively for left and right motor.
    *this->leftPort = (*this->leftPort & ~reverseLeftMask) | forwardLeftMask;
    *this->rightPort = (*this->rightPort & ~forwardRightMask) | reverseRightMask;
};

void Motor::write(uint8_t ldc, uint8_t rdc)
{
    *this->powerLeft = mapFromPercent(ldc, LOWEST_OCR, ICR1_TOP);
    *this->powerRight = mapFromPercent(rdc, LOWEST_OCR, ICR1_TOP);
};

void Motor::write(MotorDutyCycle left, MotorDutyCycle right)
{
    *this->powerLeft = mapFromPercent(static_cast<uint8_t>(left), LOWEST_OCR, ICR1_TOP);
    *this->powerRight = mapFromPercent(static_cast<uint8_t>(right), LOWEST_OCR, ICR1_TOP);
};

/* void Motor::write(uint16_t leftValue, uint16_t rightValue)
{
    if (ICR1_TOP < leftValue) leftValue = ICR1_TOP;
    if (ICR1_TOP < rightValue) rightValue = ICR1_TOP;
    *this->powerLeft = leftValue;
    *this->powerRight = rightValue;
}; */

}