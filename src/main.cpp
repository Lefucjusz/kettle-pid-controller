#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TM1637Display.h>
#include "PidController.hpp"
#include "PwmGenerator.hpp"
#include "Potentiometer.hpp"

namespace
{
    enum class State
    {
        Regulating,
        Setting
    };

    constexpr auto displayClkPin = A0;
    constexpr auto displayDioPin = A1;

    constexpr auto potentiometerPin = A2;

    constexpr auto oneWirePin = 2;

    constexpr auto pwmPin = 13;
    constexpr auto pwmFrequencyHz = 4;

    constexpr auto minTemperature = 50;
    constexpr auto maxTemperature = 96;

    constexpr auto settingStateTimeoutMs = 2500;

    const PidController::Settings pidSettings = {
        .outputMin = 0.0f,
        .outputMax = 100.0f,
        .sampleRateHz = 2,
        .kp = 5.8f,
        .ki = 0.015f,
        .kd = 38.0f,
        .dAlpha = 0.25f
    };

    State state = State::Setting;
    uint32_t lastValueChangeMs = 0;

    float tempMeasured;
    float pidOutputPwm;
    float tempSetpoint;

    TM1637Display display(displayClkPin, displayDioPin);
    Potentiometer pot(potentiometerPin);
    OneWire oneWire(oneWirePin);
    DallasTemperature sensor(&oneWire);
    PwmGenerator pwm(pwmPin, pwmFrequencyHz);
    PidController pid(tempMeasured, pidOutputPwm, tempSetpoint, pidSettings);

    void onPotentiometerValueChange(int16_t value)
    {
        state = State::Setting;
        lastValueChangeMs = millis();
    }

    float potValueToTemperature(int16_t potValue)
    {
        return static_cast<float>(map(potValue, Potentiometer::minAdcValue, Potentiometer::maxAdcValue, minTemperature, maxTemperature));
    }

    void displayNumber(int number)
    {
        display.showNumberDec(number, false, 3, 0);
    }
}

void setup() 
{
    sensor.begin();
    sensor.requestTemperaturesByIndex(0);
    tempMeasured = sensor.getTempCByIndex(0);
    sensor.setWaitForConversion(false);

    pot.update();
    tempSetpoint = potValueToTemperature(pot.getValue());
    pot.setOnChangeCallback(onPotentiometerValueChange);

    const uint8_t symbolC = SEG_A | SEG_D | SEG_E | SEG_F;
    display.setBrightness(4);
    display.setSegments(&symbolC, 1, 3);
}

void loop() 
{
    pot.update();

    switch (state) {
        case State::Regulating:
            if (pid.update()) { // TODO this is ugly, but I still haven't fixed it
                tempMeasured = sensor.getTempCByIndex(0);
                sensor.requestTemperaturesByIndex(0);
                displayNumber(round(tempMeasured));
            }
            pwm.setPwm(static_cast<uint8_t>(pidOutputPwm));
            pwm.update();
            break;

        case State::Setting:
            tempSetpoint = potValueToTemperature(pot.getValue());
            displayNumber(static_cast<int>(tempSetpoint));
            if ((millis() - lastValueChangeMs) >= settingStateTimeoutMs) {
                state = State::Regulating;
            }
            break;

        default:
            break;
    }
}
