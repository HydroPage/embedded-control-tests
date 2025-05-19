#pragma once

#include <Arduino.h>
#include <RotaryEncoder.h>
#include "constants.h"

// Takes an angle and maps it to [-pi, pi] to give some more useful orientation
// information
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

// Call once on program initialization to start the encoder monitoring routine
void attachEncoderInterrupts();

// Uses noise and disturbance priors to infer what the most likely current state
// of the system is. Updates angle and angular velocity estimates.
// Should be called as often as the configured sample time as accurately as possible.
void updateSystemEstimate();

// Determine what the best output is at the current time to reach the current
// desired state.
void runControlLaw();

void beginControlSampleTimer(unsigned int refreshPeriodMicros);

inline void initMotorEncoding()
{
    attachEncoderInterrupts();
    beginControlSampleTimer(SAMPLE_TIME_MICROS);
}

// Returns the raw encoder angle at the moment
float getMotorPosRadsRaw();

// Returns the most likely current raw encoder value given the system dynamics
// and prior beliefs about modeling and noise
float getMotorPosRads();

// Returns the most likely current encoder radians per second given dynamics
// and prior beliefs about modeling and noise
float getMotorRadsPerSec();