#include "Potentiometer.hpp"
#include <Arduino.h>

Potentiometer::Potentiometer(uint8_t analogPin) : analogPin(analogPin)
{
    pinMode(this->analogPin, INPUT);
}

void Potentiometer::setOnChangeCallback(void (*onChange)(int16_t value), uint8_t hysteresis)
{
    changeHysteresis = hysteresis;
    onChangeCallback = onChange;
}

int16_t Potentiometer::getValue()
{
    return currentValue;
}

uint8_t Potentiometer::getValuePercent()
{
    constexpr auto maxPercent = 100;
    return ((static_cast<int32_t>(currentValue) * maxPercent) / maxAdcValue);
}

void Potentiometer::update()
{
    const auto newValue = analogRead(analogPin);

    /* Apply hysteresis */
    if (abs(newValue - currentValue) < changeHysteresis) {
        return;
    }

    currentValue = newValue;
    if (onChangeCallback) {
        onChangeCallback(currentValue);
    }
}
