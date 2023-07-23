#pragma once

#include <stdint.h>

class PidController
{
    public:
        struct Settings
        {
            float outputMin;
            float outputMax;
            uint16_t sampleRateHz;
            float kp;
            float ki;
            float kd;
            float dAlpha;
        };

        PidController(float &input, float &output, float &setpoint, const Settings &settings);

        void setGains(float kp, float ki, float kd);
        void setOutputLimits(float outputMin, float outputMax);
        void setSampleRate(uint16_t sampleRateHz);

        bool update();

    private:
        void compute();
        float applyLowPassFilter(float x, float y);

        float &input;
        float &output;
        float &setpoint;

        Settings settings;

        float integralTerm;
        float lastError;
        float lastDError;
        uint32_t lastMillis;
};
