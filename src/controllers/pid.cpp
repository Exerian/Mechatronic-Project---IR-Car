#include "controllers/pid.h"

namespace G16
{

PID::PID(int8_t Kp, int8_t Ki, int8_t Kd)
    : Kp(abs(Kp))
    , Ki(abs(Ki))
    , Kd(abs(Kd))
{};
PID::~PID(){};

void PID::update(int8_t error)
{
    uint32_t currentTime = micros();
    uint32_t deltaTime = currentTime - this->lastUpdate; // dt
    if (deltaTime >= 500UL) // Only run the update every 500µs / 0.5ms
    {
        this->integral += error;
        this->value = (this->Kp * error) + (this->Ki * this->integral) + (this->Kd * (error - this->error)); // Real: Kp * error + Ki * I * dt + Kd * D / dt
        this->error = error;
        this->lastUpdate = currentTime;
    }
};

int8_t PID::getPID()
{
    return this->value;
};

uint32_t &PID::getLastUpdate()
{
    return this->lastUpdate;
};

}