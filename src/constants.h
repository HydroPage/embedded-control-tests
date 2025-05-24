#pragma once
#include <Arduino.h>

#define RadPS2RPM(x) 60 * x / TWO_PI
#define RPM2RadPS(x) TWO_PI *x / 60
#define CLAMP(x, a, b) max(a, min(x, b))

constexpr uint8_t ANALOG_WRITE_RES_BITS = 10;
constexpr uint16_t ANALOG_HIGH = (1 << ANALOG_WRITE_RES_BITS) - 1;

constexpr int ANALOG_WRITE_FREQ_HZ = 10e3;

constexpr bool COUNT_UP = true;
constexpr bool AUTORELOAD = true;
constexpr bool EDGE = true;

// Only 1, 2, and 9 can be interrupted (I think)
constexpr int ENC_PIN_1 = 1;
constexpr int ENC_PIN_2 = 2;
constexpr float ENC_TICKS_PER_REV = 1203.2 / 2; // from datasheet

constexpr int MOTOR_PIN_1 = 4;
constexpr int MOTOR_PIN_2 = 5;

// Timing constants
constexpr int ENC_SAMPLE_RATE_HZ = 50;
constexpr float SAMPLE_TIME = 1.0f / ENC_SAMPLE_RATE_HZ;
constexpr float SAMPLE_TIME_MICROS = SAMPLE_TIME * 1E6;
constexpr float V_S = 24.5f;

// Math aliases
constexpr float T = SAMPLE_TIME;
constexpr float T_MICROS = SAMPLE_TIME_MICROS;

// Discrete state transition
constexpr float Ad[2][2] =
    {
        {1, 0.01523216},
        {0, 0.56449622}};

// Voltage forcing entry
constexpr float Bd[2][1] =
    {
        {0.01097577},
        {1.00254687}};

// Measurement
constexpr float C[1][2] = {{1, 0}};

// Kalman gains
constexpr float L[2][1] = 
{
	{1.005547030606374},
	{0.3939805004843886}
};

constexpr float Kp = 1.10326566637226;
constexpr float Ki = 23.1685789938203;
constexpr float Kd = 0;

constexpr float V_i_max = 10;
constexpr float i_max = 100;