#include "PwmGenerator.hpp"
#include <Arduino.h>

namespace
{
    constexpr auto millisecondsPerSecond = 1000;
    constexpr auto sampleRateHz = 1000;
    constexpr auto samplingIntervalMs = millisecondsPerSecond / sampleRateHz;
}

PwmGenerator::PwmGenerator(uint8_t pin, uint16_t frequencyHz) : pin(pin), periodMs(millisecondsPerSecond / frequencyHz)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    setPwm(0);
    lastMillis = 0;
    lastPeriodStart = 0;
}

PwmGenerator::~PwmGenerator()
{
    digitalWrite(pin, LOW); // Leave pin in low state
}

void PwmGenerator::setPwm(uint8_t percent)
{
    if (lastPercent == percent) {
        return;
    }

    highTime = periodMs * percent / 100;
    lowTime = periodMs * (100 - percent) / 100;
    lastPercent = percent;
}

void PwmGenerator::update()
{
    if (lowTime == 0) {
        digitalWrite(pin, HIGH);
        return;
    }

    if (highTime == 0) {
        digitalWrite(pin, LOW);
        return;
    }

    const auto currentMillis = millis();
    if ((currentMillis - lastMillis) < samplingIntervalMs) {
        return;
    }

    if ((currentMillis - lastPeriodStart) >= (highTime + lowTime)) {
        lastPeriodStart = currentMillis;
        digitalWrite(pin, HIGH);
    } 
    else if ((currentMillis - lastPeriodStart) >= highTime) {
        digitalWrite(pin, LOW);
    }

    lastMillis = currentMillis;
}
