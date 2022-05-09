#include "controllers/motor.h"

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
    // Check if the right enable pins are given and set OCR. If wrong pins; force using the right pins.
    switch (enableLeft ^ 0x10)
    {
    case 0x00:
        enableRight = 9;
        this->rightOCR = &OCR1A;
        this->leftOCR = &OCR1B;
        break;
    case 0x03:
        enableRight = 10;
        this->rightOCR = &OCR1B;
        break;
    default:
        enableLeft = 9;
        enableRight = 10;
        this->leftOCR = &OCR1A;
        this->rightOCR = &OCR1B;
        break;
    }

    // Set the enable pins to output in the Data Direction Register.
    DDRB |= digitalPinToBitMask(enableLeft) | digitalPinToBitMask(enableRight);

    // Get Data Direction Register for each pin set, and set the direction pins to output.
    IO8_Pointer DDRx = portModeRegister(digitalPinToPort(forwardLeft));
    *DDRx |= forwardLeftMask | reverseLeftMask;
    DDRx = portModeRegister(digitalPinToPort(forwardRight));
    *DDRx |= forwardRightMask | reverseRightMask;
};

Motor::~Motor(){};

void Motor::initialise()
{
    // Clear/reset timers and registers:
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    // Set Timer/Counter Control Register for PWM.
    TCCR1A = (1 << COM1A1) | (1 << COM1B1);
    // Set TOP value of counter:
    ICR1 = ICR1_TOP;
    *this->leftOCR = 0x00;
    *this->rightOCR = 0x00;
    // Timer will start running when a CSxx bit is set.
    TCCR1B = (1 << WGM13) | (1 << CS10);
};

void Motor::stop()
{
    *this->leftOCR = 0x00;
    *this->rightOCR = 0x00;
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

void Motor::write(uint16_t left, uint16_t right)
{
    if (ICR1_TOP < left) left = ICR1_TOP;
    if (ICR1_TOP < right) right = ICR1_TOP;
    noInterrupts();
    *this->leftOCR = left;
    *this->rightOCR = right;
    interrupts();
};

void Motor::writeDutyCycle(uint8_t ldc, uint8_t rdc)
{
    if (100 < ldc) ldc = 100;
    if (100 < ldc) rdc = 100;
    noInterrupts();
    *this->leftOCR = mapFromPercent(ldc, LOWEST_OCR, ICR1_TOP);
    *this->rightOCR = mapFromPercent(rdc, LOWEST_OCR, ICR1_TOP);
    interrupts();
};

void Motor::writeDutyCycle(MotorDutyCycle left, MotorDutyCycle right)
{
    noInterrupts();
    *this->leftOCR = mapFromPercent(static_cast<uint8_t>(left), LOWEST_OCR, ICR1_TOP);
    *this->rightOCR = mapFromPercent(static_cast<uint8_t>(right), LOWEST_OCR, ICR1_TOP);
    interrupts();
};

uint16_t Motor::getLeftPWM()
{
    return *this->leftOCR;
};

uint16_t Motor::getRightPWM()
{
    return *this->rightOCR;
};

}