#pragma once

#include <stdint.h>

class Potentiometer
{
    public:
        explicit Potentiometer(uint8_t pin);

        void setOnChangeCallback(void (*onChange)(int16_t value), uint8_t hysteresis = defaultHysteresis);
        void update();

        int16_t getValue();
        uint8_t getValuePercent();

        static constexpr auto minAdcValue = 0;
        static constexpr auto maxAdcValue = 1023; // 10-bit ADC

    private:
        static constexpr auto defaultHysteresis = 20;

        uint8_t analogPin;
        uint8_t changeHysteresis = defaultHysteresis;
        int16_t currentValue;
        void (*onChangeCallback)(int16_t value) = nullptr;
};
