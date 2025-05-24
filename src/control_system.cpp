#include "constants.h"
#include "control_system.h"
#include "motor_control.h"

RotaryEncoder encoder(ENC_PIN_1, ENC_PIN_2, RotaryEncoder::LatchMode::TWO03);

// Radians
float theta_raw = 0;
float theta_e = 0;
float omega_e = 0;

float u = getMotorVolts();

void updateSystemEstimate()
{
    // "A priori" state estimates from model dynamics only. Ad*x + Bd*u
    float theta_prior = Ad[0][0] * theta_e + Ad[0][1] * omega_e + Bd[0][0] * u;
    float omega_prior = Ad[1][0] * theta_e + Ad[1][1] * omega_e + Bd[1][0] * u;

    // filter innovation from measurement error vs expectation
    theta_raw = encoderTicksToRadians(encoder.getPosition());
    float err = theta_raw - C[0][0] * theta_e - C[0][1] * omega_e;
    float theta_correction = L[0][0] * err;
    float omega_correction = L[1][0] * err;

    // "A posteriori" state estimates
    theta_e = theta_prior + theta_correction;
    omega_e = omega_prior + omega_correction;
}

void runControlLaw()
{
    static float prevOmegaErr = 0;

    // Error integral and derivative recursive filters
    static float i = 0;
    static float d = 0;

    float omegaErr = getMotorDesiredOmega() - omega_e;

    // Bilinear transform (Tustin) PID controller, recursive update
    float p = Kp * omegaErr;
    i = CLAMP(i + (T / 2) * (omegaErr + prevOmegaErr), -i_max, i_max);
    d = (2 * Kd * (omegaErr - prevOmegaErr) - (T - 2 * Kd) * d) / (T + 2 * Kd);

    // This output will be the next sample's "previous output", u_{k-1}
    u = setMotorVoltsClamp(p + Ki * i + d);

    prevOmegaErr = omegaErr;
}

IRAM_ATTR void onEncoderPulse() { encoder.tick(); }

void attachEncoderInterrupts()
{
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_1), onEncoderPulse, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_2), onEncoderPulse, CHANGE);
}

hw_timer_t *timer0_config;

// Discrete-time system sample routine
IRAM_ATTR void onTimerTick()
{
    // The order matters because of what each does with the stored system output
    updateSystemEstimate();
    runControlLaw();
}

void beginControlSampleTimer(unsigned int T_micros)
{
    // Subdivide to microseconds: 80 MHz -> 1 MHz (to count microseconds)
    timer0_config = timerBegin(0, 80, COUNT_UP);
    timerAttachInterrupt(timer0_config, onTimerTick, AUTORELOAD);
    timerAlarmWrite(timer0_config, T_micros, EDGE);
    timerAlarmEnable(timer0_config);
}

float getMotorPosRadsRaw()
{
    return theta_raw;
}

float getMotorPosRads()
{
    return theta_e;
}

float getMotorRadsPerSec()
{
    return omega_e;
}