#include "PidController.hpp"
#include <Arduino.h>

namespace
{
    inline uint32_t frequencyToPeriodMs(uint32_t frequencyHz)
    {
        constexpr auto millisecondsPerSecond = 1000;
        return millisecondsPerSecond / frequencyHz;
    }

    inline constexpr float clamp(float input, float min, float max)
    {
        return (input > max) ? max : (input < min) ? min : input;
    }
}

PidController::PidController(float &input, float &output, float &setpoint, const Settings &settings)  
: input(input), output(output), setpoint(setpoint), settings(settings)
{
    integralTerm = 0;
    lastError = setpoint - input;
    lastDError = 0;
    lastMillis = 0;

    compute();
}

void PidController::setGains(float kp, float ki, float kd)
{
    settings.kp = kp;
    settings.ki = ki;
    settings.kd = kd;
}

void PidController::setOutputLimits(float outputMin, float outputMax)
{
    /* Sanity check */
    if (outputMin >= outputMax) {
        return;
    }

    /* Update values */
    settings.outputMin = outputMin;
    settings.outputMax = outputMax;

    /* Apply anti-windup and limits */
    integralTerm = clamp(integralTerm, settings.outputMin, settings.outputMax);
    output = clamp(output, settings.outputMin, settings.outputMax);
}

void PidController::setSampleRate(uint16_t sampleRateHz)
{
    settings.sampleRateHz = sampleRateHz;
}

bool PidController::update()
{
    const auto currentMillis = millis();
    if ((currentMillis - lastMillis) < frequencyToPeriodMs(settings.sampleRateHz)) {
        return false;
    }
    
    compute();

    lastMillis = currentMillis;
    return true;
}

void PidController::compute()
{
    /* Compute error and its derivative */
    const auto error = setpoint - input;
    const auto dError = (error - lastError) * settings.sampleRateHz; 
    const auto dErrorFiltered = applyLowPassFilter(dError, lastDError);
    Serial.print("D:");
    Serial.print(settings.kd * dErrorFiltered);
    Serial.print(',');

    /* Update I term and apply anti-windup */
    const auto dIntegral = settings.ki * error / settings.sampleRateHz;
    if (((output > settings.outputMin) && (output < settings.outputMax)) ||
        ((output <= settings.outputMin) && (dIntegral > 0.0f)) ||
        ((output >= settings.outputMax) && (dIntegral < 0.0f))) 
    {
        integralTerm += dIntegral;    
    }
    integralTerm = clamp(integralTerm, settings.outputMin, settings.outputMax);
    Serial.print("I:");
    Serial.print(integralTerm);
    Serial.print(',');

    /* Update output and apply limits */
    output = settings.kp * error + integralTerm + settings.kd * dErrorFiltered;
    output = clamp(output, settings.outputMin, settings.outputMax);

    /* Store last error and last error derivative */
    lastError = error;
    lastDError = dErrorFiltered;
}

float PidController::applyLowPassFilter(float x, float y)
{
    return ((settings.dAlpha * x) + ((1.0f - settings.dAlpha) * y)); // EMA algorithm
}
