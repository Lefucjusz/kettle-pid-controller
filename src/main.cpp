#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TM1637Display.h>
#include "PidController.hpp"
#include "PwmGenerator.hpp"

void setup() 
{
    Serial.begin(115200);
    pinMode(13, OUTPUT); // TODO pin definition
    digitalWrite(13, LOW);
}

void loop() 
{
    constexpr auto displayClkPin = A0;
    constexpr auto displayDioPin = A1;

    constexpr auto oneWirePin = 2;

    constexpr auto pwmPin = 13;
    constexpr auto pwmFrequencyHz = 4;
    
    PidController::Settings pidSettings = {
        .outputMin = 0.0f,
        .outputMax = 100.0f,
        .sampleRateHz = 2,
        .kp = 5.8f,
        .ki = 0.015f,
        .kd = 38.0f,
        .dAlpha = 0.25f
    };

    float tempMeasured, pidOutputPwm, tempSetpoint = 92;

    OneWire oneWire(oneWirePin);
    DallasTemperature sensor(&oneWire);
    sensor.begin();

    sensor.requestTemperaturesByIndex(0);
    tempMeasured = sensor.getTempCByIndex(0);
    sensor.setWaitForConversion(false);

    PwmGenerator pwm(pwmPin, pwmFrequencyHz);
    PidController pid(tempMeasured, pidOutputPwm, tempSetpoint, pidSettings);

    TM1637Display display(displayClkPin, displayDioPin);
    uint8_t symbolC = SEG_A | SEG_D | SEG_E | SEG_F;
    display.setBrightness(4);
    display.setSegments(&symbolC, 1, 3);

    while (1) {
        if (pid.update()) { // TODO this is ugly, but just for now
            tempMeasured = sensor.getTempCByIndex(0);
            sensor.requestTemperaturesByIndex(0);
            Serial.print("TCurrent:");
            Serial.print(tempMeasured);
            Serial.print(',');
            Serial.print("TSet:");
            Serial.print(tempSetpoint);
            Serial.print(',');
            Serial.print("Error:");
            Serial.print(tempSetpoint - tempMeasured);
            Serial.print(',');
            Serial.print("PWM:");
            Serial.println(static_cast<uint8_t>(pidOutputPwm));

            display.showNumberDec(round(tempMeasured), false, 3, 0);
        }

        pwm.setPwm(static_cast<uint8_t>(pidOutputPwm));
        pwm.update();
    }
}
