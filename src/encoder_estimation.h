#pragma once

#include <Arduino.h>
#include <RotaryEncoder.h>
#include "constants.h"

inline float wrapPlusMinusPi(float angleRads)
{
    angleRads = fmod(angleRads + PI, TWO_PI);
    if (angleRads < 0)
        angleRads += TWO_PI;
    return angleRads - PI;
}

constexpr float encoderTicksToRadians(float encoderTicks)
{
    return encoderTicks / ENC_TICKS_PER_REV * TWO_PI;
}

constexpr float radiansToEncoderTicks(float rads)
{
    return rads / TWO_PI * ENC_TICKS_PER_REV;
}

void attachEncoderInterrupts();

// Uses the filter to update the omega estimate if a sample period has elapsed.
// Should be called as often as possible to not skip sample periods.
void updateEncoderEstimates();

void beginEncoderUpdateTimer(unsigned int refreshPeriodMicros);

inline void initMotorEncoding()
{
    attachEncoderInterrupts();
    beginEncoderUpdateTimer(ENC_SAMPLE_PERIOD_MICROS);
}

float getMotorRadsPerSec();