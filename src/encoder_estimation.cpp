#include "encoder_estimation.h"
#include "motor_control.h"

RotaryEncoder encoder(ENC_PIN_1, ENC_PIN_2, RotaryEncoder::LatchMode::TWO03);

// Radians
float theta_e = 0;
float omega_e = 0;

IRAM_ATTR void onEncoderPulse() { encoder.tick(); }

// Call once on program initialization
void attachEncoderInterrupts()
{
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_1), onEncoderPulse, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_2), onEncoderPulse, CHANGE);
}

// Discrete state transition
constexpr float Ad[2][2] = 
{
    {1, dt},
    {0, 1 - b*dt/J - k*k*dt/(J*R)}
};

constexpr float Bd[2][1] =
{
    {0},
    {k*dt / (J*R)}
};

constexpr float C[1][2] = { {1, 0} };

// Kalman gains
constexpr float L[2] = {1.7782548238373375, 34.15614294405403};

void updateEncoderEstimates()
{
    // static unsigned long prevUpdateMicros = 0;
    static float u = getMotorVolts();

    // unsigned long now = micros();
    // unsigned long elapsed = now - prevUpdateMicros;

    // if (elapsed >= ENC_SAMPLE_PERIOD_MICROS)
    // {

    // x_e[k] = Ad*x_e[k - 1] + Bd*u[k - 1] - L(y - C*x_e[k - 1])

    // state error estimates = Ad*x + Bd*u
    float estim_theta = Ad[0][0]*theta_e + Ad[0][1]*omega_e + Bd[0][0]*u;
    float estim_omega = Ad[1][0]*theta_e + Ad[1][1]*omega_e + Bd[1][0]*u;

    // filter innovation from measurement error vs expectation
    float theta = encoderTicksToRadians(encoder.getPosition());
    float err = theta - C[0][0]*theta_e - C[0][1]*omega_e;
    float theta_correction = L[0] * err;
    float omega_correction = L[1] * err;

    // state estimates
    theta_e = estim_theta + theta_correction;
    omega_e = estim_omega + omega_correction;

    // Filter uses u[k - 1], so update u after estimation, for next sample
    u = getMotorVolts();

    //     prevUpdateMicros = now;
    // }
}

hw_timer_t* timer0_config;

IRAM_ATTR void onTimerTick()
{
    updateEncoderEstimates();
}

void beginEncoderUpdateTimer(unsigned int refreshPeriodMicros)
{
    // Subdivide to microseconds: 80 MHz -> 10 MHz
    timer0_config = timerBegin(0, 80, COUNT_UP);
    timerAttachInterrupt(timer0_config, onTimerTick, AUTORELOAD);
    timerAlarmWrite(timer0_config, refreshPeriodMicros, EDGE);
    timerAlarmEnable(timer0_config);
}

float getMotorRadsPerSec()
{
    return omega_e;
}