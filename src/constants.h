#pragma once
#include <Arduino.h>

#define RadPS2RPM(x) 60*x / TWO_PI
#define RPM2RadPS(x) TWO_PI*x / 60

constexpr bool COUNT_UP = true;
constexpr bool AUTORELOAD = true;
constexpr bool EDGE = true;

// Only 1, 2, and 9 can be interrupted (I think)
constexpr int ENC_PIN_1 = 1;
constexpr int ENC_PIN_2 = 2;
constexpr int MOTOR_PIN_1 = 4;
constexpr int MOTOR_PIN_2 = 5;

constexpr float ENC_TICKS_PER_REV = 1203.2 / 2;
constexpr int ENC_SAMPLE_RATE_HZ = 50;
constexpr int ENC_SAMPLE_PERIOD_MICROS = static_cast<int> (1.0f / ENC_SAMPLE_RATE_HZ * 1E6);
constexpr float dt = 1.0f / ENC_SAMPLE_RATE_HZ;

constexpr float J = 0.004;
constexpr float V_S = 23.4;
constexpr float R = 7;
constexpr float SPEED_NL = RPM2RadPS(500);
constexpr float k = V_S / SPEED_NL;
constexpr float b = 8E-4;