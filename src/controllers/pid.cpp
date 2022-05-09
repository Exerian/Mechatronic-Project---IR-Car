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
    this->integral += error;
    this->value = (this->Kp * error) + (this->Ki * this->integral) + (this->Kd * (error - this->error));
    this->error = error;
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