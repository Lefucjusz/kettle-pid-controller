#pragma once

#include <stdint.h>

class PwmGenerator
{
    public:
        PwmGenerator(uint8_t pin, uint16_t frequencyHz);
        ~PwmGenerator();

        void setPwm(uint8_t percent);
        void update();

    private:
        uint8_t pin;
        uint8_t lastPercent;
        uint16_t periodMs;
        uint16_t lowTime;
        uint16_t highTime;
        uint32_t lastMillis;
        uint32_t lastPeriodStart;
};
